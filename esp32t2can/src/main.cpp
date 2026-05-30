/*
 * esp32t2can / main.cpp  —  Phase 4: 무선(UDP) LFO 스트림 → 로컬 오실레이터 lock
 * ============================================================================
 * 컴퓨터 앱(controller_app) 이 60Hz 로 {run, freq, amp, phase} 를 UDP 로 던지고,
 * ESP32 는 로컬 위상 오실레이터를 그 스트림에 PLL 처럼 soft-lock 해서 ODrive 를 구동.
 *
 * 핵심(패킷손실 견고):
 *   - 오실레이터는 항상 로컬에서 돈다 → ODrive 입력 갭 0 (MISSING_INPUT 폭주 없음).
 *   - 패킷 오면 위상/주파수/진폭을 부드럽게 보정(lock). 안 오면 마지막 값으로 자유진행.
 *   - 무수신 COMM_TIMEOUT 초과 → fade-out 후 IDLE (안전, 통신 두절 대응).
 *   - run=1 → arm+run, run=0 → fade-out+IDLE (앱 start/stop = BOOT 역할).
 *   - 고주파에서 진폭은 vel_limit/accel 로 자동 클램프. 4Hz 상한.
 *
 * 보드: T-2Can V1.0 = MCP2515 16MHz, CAN 500k, autowp/arduino-mcp2515.
 * 와이어 프로토콜: controller_app/protocol.py 와 동일 (20바이트 LE).
 * ============================================================================
 */
#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include <mcp2515.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "wifi_config.h"
#include "wavetables.h"   // SIN/SWING 위치·속도 테이블 (gen_wavetables.py 생성)

/* ---- 핀맵 (MCP2515 변형) ---- */
static const uint8_t PIN_SPI_SCLK=12, PIN_SPI_MOSI=11, PIN_SPI_MISO=13;
static const uint8_t PIN_MCP_CS=10, PIN_MCP_INT=8, PIN_MCP_RST=9, PIN_BOOT=0;

/* ---- ODrive / CAN ---- */
static const uint8_t  ODRIVE_NODE_ID = 1;
static const float    POS_GAIN_SOFT  = 5.0f;     // 소프트 (저장값 50 → RAM 5)
static const float    GEAR           = 8.0f;     // 출력→모터 기어비
static const float    VEL_LIM        = 5.0f;     // rev/s (진폭 클램프 기준)
static const float    ALPHA_MAX      = 308.0f;   // turn/s² (10A 기준 가속 한계, 진폭 클램프)
static const float    AMP_MAX_TURN   = 0.70f;    // 모터 절대 진폭 상한
static const float    FREQ_MAX       = 4.0f;     // Hz 상한

/* ---- 오실레이터 / 스트림 lock ---- */
static const uint16_t UPDATE_HZ      = 200;      // ESP32→ODrive Set_Input_Pos 송신율 (무선율과 무관)
                                                 // 60→200: 위치 스텝 1/3.3 → 큰 진폭에서도 매끈. CAN 버스 ~7%.
static const float    RAMP_S         = 1.0f;     // 진폭 fade in/out [s]
static const float    MORPH_S        = 0.30f;    // 파형 전환 크로스페이드 [s] (C1 연속)
static const float    LOCK_GAIN      = 0.15f;    // 위상 soft-lock 게인 (0~1)
static const float    PARAM_LPF      = 0.08f;    // freq/amp 추종 LPF
static const uint32_t COMM_TIMEOUT_MS = 500;     // 무수신 → IDLE [ms]
static const uint32_t STATUS_MS      = 500;

/* ---- 와이어 프로토콜 (protocol.py 와 동일) ---- */
static const uint16_t WIRE_MAGIC = 0x0DCA;
static const uint16_t UDP_PORT   = 4210;
static const int      PKT_SIZE   = 20;

/* ---- can_simple cmd id ---- */
static const uint8_t CMD_HEARTBEAT=0x01, CMD_ENCODER_ESTIMATES=0x09;
static const uint8_t CMD_SET_AXIS_STATE=0x07, CMD_SET_CONTROLLER_MODE=0x0B;
static const uint8_t CMD_SET_INPUT_VEL=0x0D, CMD_SET_INPUT_POS=0x0C;
static const uint8_t CMD_SET_POS_GAIN=0x1A, CMD_CLEAR_ERRORS=0x18;

/* ---- ODrive enum ---- */
static const uint32_t AXIS_IDLE=1, AXIS_CLOSED_LOOP=8;
static const uint32_t CTRL_VELOCITY=2, CTRL_POSITION=3;
static const uint32_t INPUT_PASSTHROUGH=1, INPUT_VEL_RAMP=2;

/* ---- 파형 enum (protocol.py 와 동일) ---- */
static const uint8_t WAVE_SINE=0, WAVE_SWING=1;

static const float TWO_PI_F = 6.28318530718f;

MCP2515 mcp2515(PIN_MCP_CS, 10000000UL, &SPI);
WiFiUDP  udp;
uint8_t  udpbuf[64];

/* ---- ODrive 수신 상태 ---- */
struct St {
  bool got_hb=false; uint32_t axis_error=0; uint8_t axis_state=0;
  bool got_enc=false; float pos=0, vel=0;
  uint32_t cnt_hb=0, cnt_enc=0;
};
St st;

/* ---- 제어/오실레이터 상태 ---- */
enum Mode { M_IDLE, M_RUNNING, M_STOPPING };
Mode  g_mode = M_IDLE;
float g_center = 0;
float g_phase = 0;            // 로컬 위상 [rad]
float g_freq = 0,  g_freq_t = 0;    // 현재/목표 주파수 [Hz]
float g_amp = 0,   g_amp_t = 0;     // 현재/목표 진폭 [모터 turn] (클램프 후)
uint8_t g_waveform = WAVE_SINE;     // 목표 파형 (수신값)
float   g_morph = 0;                // 현재 블렌드 0=SINE..1=SWING (MORPH_S 로 램프)
uint8_t g_run = 0;            // 마지막 수신 run
uint16_t g_seq = 0, g_pkt_cnt = 0;
uint32_t g_last_pkt_ms = 0;
uint32_t g_run_t0 = 0, g_stop_t0 = 0;
bool g_mcp_ok = false;
bool g_have_pkt = false;

static inline uint8_t id_node(uint32_t id){ return (id>>5)&0x3F; }
static inline uint8_t id_cmd(uint32_t id){ return id&0x1F; }
static inline float wrapPi(float a){ while(a>M_PI)a-=TWO_PI_F; while(a<-M_PI)a+=TWO_PI_F; return a; }
static inline float lerpf(float a, float b, float m){ return a + (b-a)*m; }

/* ---- 웨이브테이블 lookup (선형보간, 위상 [0,2π)) ---- */
float wt_lookup(const float* tbl, float phase){
  float x = phase / TWO_PI_F * (float)WT_N;
  int i = (int)floorf(x);
  float frac = x - (float)i;
  i %= WT_N; if(i<0) i += WT_N;
  int j = (i+1) % WT_N;
  return tbl[i] + (tbl[j]-tbl[i])*frac;
}

/* ---- 출력° → 모터 turn, 진폭 안전 클램프 ----
 * peak_vel/peak_acc: 파형의 peak 속도·가속 배수(사인=1.0). 그네는 중심 통과가
 * 빨라 1보다 큼 → 그만큼 진폭을 더 깎아야 vel_limit/accel 안 넘음(MISSING_INPUT 폭주 방지). */
float deg_to_turn(float deg){ return deg/360.0f*GEAR; }
float clamp_amp(float amp_turn, float freq, float peak_vel, float peak_acc){
  if (freq > 0.01f){
    float v_max = VEL_LIM   / (TWO_PI_F*freq*peak_vel);                 // 속도 한계
    float a_max = ALPHA_MAX / (TWO_PI_F*TWO_PI_F*freq*freq*peak_acc);   // 가속 한계
    float m = (v_max < a_max) ? v_max : a_max;
    if (amp_turn > m) amp_turn = m;
  }
  if (amp_turn > AMP_MAX_TURN) amp_turn = AMP_MAX_TURN;
  if (amp_turn < 0) amp_turn = 0;
  return amp_turn;
}

/* ---- CAN TX ---- */
bool sendCmd(uint8_t cmd, const uint8_t* d8){
  struct can_frame f; f.can_id=((uint32_t)ODRIVE_NODE_ID<<5)|cmd; f.can_dlc=8;
  for(int i=0;i<8;++i) f.data[i]=d8?d8[i]:0;
  return mcp2515.sendMessage(&f)==MCP2515::ERROR_OK;
}
void tx_axis_state(uint32_t s){ uint8_t d[8]={0}; memcpy(d,&s,4); sendCmd(CMD_SET_AXIS_STATE,d); }
void tx_ctrl_mode(uint32_t c,uint32_t i){ uint8_t d[8]={0}; memcpy(d,&c,4); memcpy(d+4,&i,4); sendCmd(CMD_SET_CONTROLLER_MODE,d); }
void tx_input_vel(float v){ uint8_t d[8]={0}; float t=0; memcpy(d,&v,4); memcpy(d+4,&t,4); sendCmd(CMD_SET_INPUT_VEL,d); }
void tx_pos_gain(float g){ uint8_t d[8]={0}; memcpy(d,&g,4); sendCmd(CMD_SET_POS_GAIN,d); }
void tx_input_pos(float pos, float vel_ff){
  uint8_t d[8]={0}; memcpy(&d[0],&pos,4);
  int16_t vff=(int16_t)lroundf(vel_ff*1000.0f), tff=0;
  memcpy(&d[4],&vff,2); memcpy(&d[6],&tff,2);
  sendCmd(CMD_SET_INPUT_POS,d);
}
void tx_clear_errors(){
  struct can_frame f; f.can_id=((uint32_t)ODRIVE_NODE_ID<<5)|CMD_CLEAR_ERRORS;
  f.can_dlc=1; f.data[0]=0; mcp2515.sendMessage(&f);
}

/* ---- CAN RX (heartbeat/encoder) ---- */
void drainRx(){
  struct can_frame f;
  while(mcp2515.readMessage(&f)==MCP2515::ERROR_OK){
    uint32_t id=f.can_id&CAN_SFF_MASK;
    if(id_node(id)!=ODRIVE_NODE_ID) continue;
    if(id_cmd(id)==CMD_HEARTBEAT && f.can_dlc>=5){
      st.cnt_hb++; memcpy(&st.axis_error,&f.data[0],4); st.axis_state=f.data[4]; st.got_hb=true;
    } else if(id_cmd(id)==CMD_ENCODER_ESTIMATES && f.can_dlc>=8){
      st.cnt_enc++; memcpy(&st.pos,&f.data[0],4); memcpy(&st.vel,&f.data[4],4); st.got_enc=true;
    }
  }
}

void fault_stop(const char* why){
  tx_axis_state(AXIS_IDLE); delay(2); tx_clear_errors();
  g_mode=M_IDLE;
  Serial.print(">>> 정지+에러클리어 (IDLE) — "); Serial.println(why);
}

// VEL arm 으로 라이브 pos 확보 → POS/PASSTHROUGH soft gain 전환. center 캡처.
bool arm_and_center(){
  tx_clear_errors(); delay(5);
  tx_ctrl_mode(CTRL_VELOCITY, INPUT_VEL_RAMP); delay(5);
  tx_input_vel(0.0f); delay(5);
  tx_axis_state(AXIS_CLOSED_LOOP);
  st.got_enc=false;
  uint32_t t0=millis();
  while(millis()-t0<1000){
    drainRx();
    if(st.axis_state==AXIS_CLOSED_LOOP && st.got_enc) break;
    tx_input_vel(0.0f); delay(10);
  }
  if(st.axis_state!=AXIS_CLOSED_LOOP){
    Serial.print("[오류] CLOSED_LOOP 실패 state="); Serial.print(st.axis_state);
    Serial.print(" err=0x"); Serial.println(st.axis_error,HEX);
    fault_stop("arm 실패"); return false;
  }
  g_center=st.pos;
  tx_pos_gain(POS_GAIN_SOFT); delay(5);
  tx_ctrl_mode(CTRL_POSITION, INPUT_PASSTHROUGH); delay(5);
  tx_input_pos(g_center, 0.0f); delay(5);
  Serial.print("center="); Serial.print(g_center,4); Serial.println(" turn");
  return true;
}

void start_running(float start_phase){
  if(!arm_and_center()) return;
  g_phase = start_phase;          // 앱 위상에 맞춰 시작
  g_freq  = g_freq_t;             // 목표 주파수에서 시작 (진폭은 env 로 fade-in)
  g_amp   = 0;                    // env 와 별개로 amp 도 0 에서
  g_run_t0 = millis();
  g_mode = M_RUNNING;
  Serial.println(">>> RUNNING (UDP LFO lock)");
}

/* ---- UDP 패킷 수신/파싱 ---- */
void pollUdp(){
  int n = udp.parsePacket();
  while(n > 0){
    if(n <= (int)sizeof(udpbuf)){
      int r = udp.read(udpbuf, n);
      if(r==PKT_SIZE){
        uint16_t magic; memcpy(&magic,&udpbuf[0],2);
        if(magic==WIRE_MAGIC){
          uint16_t seq; uint8_t run, wave; float freq, amp_deg, phase;
          memcpy(&seq,&udpbuf[2],2);
          run = udpbuf[4]; wave = udpbuf[5];
          memcpy(&freq,&udpbuf[8],4);
          memcpy(&amp_deg,&udpbuf[12],4);
          memcpy(&phase,&udpbuf[16],4);
          // 안전 클램프
          if(freq<0) freq=0; if(freq>FREQ_MAX) freq=FREQ_MAX;
          g_waveform = (wave==WAVE_SWING) ? WAVE_SWING : WAVE_SINE;
          // 목표 진폭은 목표 파형의 peak 계수로 클램프 (출력 시점에 블렌드값으로 재클램프)
          float pv = (g_waveform==WAVE_SWING)?WT_PEAKVEL_SWING:WT_PEAKVEL_SINE;
          float pa = (g_waveform==WAVE_SWING)?WT_PEAKACC_SWING:WT_PEAKACC_SINE;
          g_freq_t = freq;
          g_amp_t  = clamp_amp(deg_to_turn(amp_deg), freq, pv, pa);
          g_run = run; g_seq = seq; g_pkt_cnt++;
          g_last_pkt_ms = millis();
          g_have_pkt = true;
          (void)phase;  // 단일 모터: 위상 lock 안 함 (Wi-Fi 지터로 튐/엇박 유발).
                        // 로컬 오실레이터는 freq/amp 로만 자유진행. 위상 동기는
                        // 듀얼 모터 단계에서 저속 공유기준으로 별도 구현.
        }
      }
    }
    n = udp.parsePacket();
  }
}

void setup(){
  Serial.begin(115200);
  for(int i=0;i<30 && !Serial;++i) delay(100);
  delay(200);
  Serial.println();
  Serial.println("[esp32t2can] Phase 4: UDP LFO 스트림 제어");

  pinMode(PIN_BOOT, INPUT_PULLUP);
  // MCP2515 RESET 펄스 + init
  pinMode(PIN_MCP_RST, OUTPUT);
  digitalWrite(PIN_MCP_RST,HIGH); delay(100);
  digitalWrite(PIN_MCP_RST,LOW);  delay(100);
  digitalWrite(PIN_MCP_RST,HIGH); delay(100);
  SPI.begin(PIN_SPI_SCLK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_MCP_CS);
  int e1=mcp2515.reset(), e2=mcp2515.setBitrate(CAN_500KBPS,MCP_16MHZ), e3=mcp2515.setNormalMode();
  g_mcp_ok = (e1==MCP2515::ERROR_OK && e2==MCP2515::ERROR_OK && e3==MCP2515::ERROR_OK);
  Serial.print("MCP2515 "); Serial.println(g_mcp_ok?"OK":"실패");

  // Wi-Fi STA
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Wi-Fi 연결 중");
  uint32_t t0=millis();
  while(WiFi.status()!=WL_CONNECTED && millis()-t0<15000){ Serial.print("."); delay(300); }
  if(WiFi.status()==WL_CONNECTED){
    Serial.println();
    Serial.print(">>> Wi-Fi OK. ESP32 IP = "); Serial.println(WiFi.localIP());
    Serial.print(">>> 앱에서 이 IP:"); Serial.print(UDP_PORT); Serial.println(" 로 송신하세요.");
    udp.begin(UDP_PORT);
  } else {
    Serial.println("\n[경고] Wi-Fi 실패 (wifi_config.h SSID/PASS 확인). UDP 미동작.");
  }
  Serial.println("대기 중 (앱 START 로 동작).");
}

void loop(){
  drainRx();
  pollUdp();
  uint32_t now = millis();

  // ---- run 플래그 전이 ----
  if(g_have_pkt){
    if(g_run==1 && g_mode==M_IDLE)      start_running(0.0f);
    else if(g_run==0 && g_mode==M_RUNNING){ g_mode=M_STOPPING; g_stop_t0=now; Serial.println(">>> STOP(app) fade-out"); }
  }
  // ---- 통신 두절 → IDLE ----
  if(g_mode==M_RUNNING && (now-g_last_pkt_ms)>COMM_TIMEOUT_MS){
    g_mode=M_STOPPING; g_stop_t0=now;
    Serial.println(">>> 통신 두절 → fade-out + IDLE");
  }
  // ---- BOOT = 로컬 비상정지 ----
  static int lb=HIGH; static uint32_t lbm=0;
  int b=digitalRead(PIN_BOOT);
  if(b==LOW && lb==HIGH && (now-lbm)>300){ lbm=now; if(g_mode!=M_IDLE) fault_stop("BOOT 비상정지"); }
  lb=b;
  // ---- ODrive fault (과전류 등 자체 disarm) ----
  if(g_mode!=M_IDLE && st.got_hb){
    float rt=(now-g_run_t0)/1000.0f;
    if(rt>0.4f && (st.axis_state!=AXIS_CLOSED_LOOP || st.axis_error!=0)){
      Serial.print("[ODrive fault] state="); Serial.print(st.axis_state);
      Serial.print(" err=0x"); Serial.println(st.axis_error,HEX);
      fault_stop("ODrive fault");
    }
  }

  // ---- 60Hz 오실레이터 출력 ----
  static uint32_t last_tx=0;
  if((g_mode==M_RUNNING||g_mode==M_STOPPING) && (now-last_tx)>=(1000/UPDATE_HZ)){
    last_tx=now;
    float dt=1.0f/UPDATE_HZ;
    // 파라미터 부드럽게 추종
    g_freq += (g_freq_t - g_freq)*PARAM_LPF;
    g_amp  += (g_amp_t  - g_amp )*PARAM_LPF;
    // 파형 크로스페이드 (선택 변경 시 MORPH_S 동안 SINE↔SWING 블렌드 → 튐 없음)
    float morph_t = (g_waveform==WAVE_SWING) ? 1.0f : 0.0f;
    float dm = dt / MORPH_S;
    if(g_morph < morph_t){ g_morph += dm; if(g_morph>morph_t) g_morph=morph_t; }
    else if(g_morph > morph_t){ g_morph -= dm; if(g_morph<morph_t) g_morph=morph_t; }
    // 위상 진행 (패킷 없어도 계속 = 자유진행)
    g_phase += TWO_PI_F*g_freq*dt;
    if(g_phase> M_PI*2) g_phase-=TWO_PI_F;
    // envelope
    float env;
    if(g_mode==M_RUNNING){
      float t=(now-g_run_t0)/1000.0f;
      env = (t<RAMP_S)?(t/RAMP_S):1.0f;
    } else {
      float td=(now-g_stop_t0)/1000.0f;
      env = 1.0f - td/RAMP_S;
      if(env<=0.0f){ env=0.0f; tx_input_pos(g_center,0.0f); tx_axis_state(AXIS_IDLE); g_mode=M_IDLE; Serial.println(">>> IDLE"); }
    }
    if(g_mode!=M_IDLE){
      // 블렌드된 peak 계수로 재클램프 (그네일수록 진폭 더 깎임 = 안전)
      float pv = lerpf(WT_PEAKVEL_SINE, WT_PEAKVEL_SWING, g_morph);
      float pa = lerpf(WT_PEAKACC_SINE, WT_PEAKACC_SWING, g_morph);
      float amp = clamp_amp(g_amp, g_freq, pv, pa);
      // 위치/속도 = 두 테이블 lookup 의 morph 블렌드 (C1 연속)
      float val  = lerpf(wt_lookup(WT_POS_SINE,g_phase), wt_lookup(WT_POS_SWING,g_phase), g_morph);
      float dval = lerpf(wt_lookup(WT_VEL_SINE,g_phase), wt_lookup(WT_VEL_SWING,g_phase), g_morph);
      float pos = g_center + env*amp*val;
      float vff = env*amp*(TWO_PI_F*g_freq)*dval;
      tx_input_pos(pos, vff);
    }
  }

  // ---- 상태 출력 ----
  static uint32_t lp=0;
  if(now-lp>=STATUS_MS){
    lp=now;
    const char* m=(g_mode==M_IDLE)?"idle":(g_mode==M_RUNNING)?"RUN":"STOP";
    uint32_t age = g_have_pkt ? (now-g_last_pkt_ms) : 9999;
    Serial.print("wifi=");
    Serial.print(WiFi.status()==WL_CONNECTED ? WiFi.localIP().toString() : String("X"));
    Serial.print(" ["); Serial.print(m); Serial.print("] ");
    Serial.print("f="); Serial.print(g_freq,2);
    Serial.print(" amp="); Serial.print(g_amp,3);
    Serial.print(" wav="); Serial.print(g_waveform==WAVE_SWING?"swing":"sine");
    Serial.print(" morph="); Serial.print(g_morph,2);
    Serial.print(" run="); Serial.print(g_run);
    Serial.print(" pkt="); Serial.print(g_pkt_cnt);
    Serial.print(" age="); Serial.print(age); Serial.print("ms");
    if(st.got_hb){ Serial.print(" | odrv st="); Serial.print(st.axis_state);
      Serial.print(" err=0x"); Serial.print(st.axis_error,HEX);
      Serial.print(" pos="); Serial.print(st.pos,3); }
    Serial.println();
  }
}
