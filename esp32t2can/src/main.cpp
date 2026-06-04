/*
 * esp32t2can / main.cpp  —  Phase 5: 듀얼 모터(2 ODrive) 동시 제어
 * ============================================================================
 * 컴퓨터 앱(controller_app) 이 60Hz 로 {run, freq, amp, phase} 를 UDP 로 던지고,
 * ESP32 는 로컬 위상 오실레이터를 돌려 ODrive **2대**(node_id 1, 2) 를 한 CAN-A
 * 버스로 동시에 구동. 오실레이터가 1개라 두 모터 위상은 원천적으로 100% 동기.
 *
 * 듀얼 모터 핵심:
 *   - NODE_IDS[] 두 노드에 매 틱 Set_Input_Pos 전송 (한 오실레이터, 두 출력).
 *   - MOTOR_SIGN[]   : 거울 대칭 장착 보정 (한 모터가 반대로 움직이면 -1).
 *   - MOTOR_REL_PHASE[] : 평행 swing={0,0}, Z-twist={0,π} (batten 비틀기).
 *   - 한쪽 모터라도 disarm(과전류 등) → 양쪽 동시 정지 (batten 비대칭 응력 방지).
 *
 * 패킷손실 견고(기존 유지):
 *   - 오실레이터는 항상 로컬에서 돈다 → ODrive 입력 갭 0 (MISSING_INPUT 폭주 없음).
 *   - 무수신 COMM_TIMEOUT 초과 → fade-out 후 양쪽 IDLE (통신 두절 안전).
 *   - run=1 → arm+run, run=0 → fade-out+IDLE.
 *   - 고주파에서 진폭은 vel_limit/accel 로 자동 클램프. 4Hz 상한.
 *
 * 보드: T-2Can V1.0 = MCP2515 16MHz, CAN-A 500k, autowp/arduino-mcp2515.
 *       종단: T2CAN 고정120 + ODrive 한쪽만 ON = 총 60Ω (모터2 종단 OFF 저장됨).
 * 와이어 프로토콜: controller_app/protocol.py 와 동일 (20바이트 LE).
 * ============================================================================
 */
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <math.h>
#include <mcp2515.h>
#include <Adafruit_BNO08x.h>   // IMU(BNO085) 센싱 — 모터 제어와 공존(제어엔 아직 안 씀)
#include <WiFi.h>
#include <WiFiUdp.h>
#include "wifi_config.h"
#include "wavetables.h"   // SIN/SWING 위치·속도 테이블 (gen_wavetables.py 생성)

/* ---- 핀맵 (MCP2515 변형) ---- */
static const uint8_t PIN_SPI_SCLK=12, PIN_SPI_MOSI=11, PIN_SPI_MISO=13;
static const uint8_t PIN_MCP_CS=10, PIN_MCP_INT=8, PIN_MCP_RST=9, PIN_BOOT=0;

/* ---- IMU (BNO085, QWIIC/I2C) — SPI/MCP 핀과 무충돌 ---- */
static const uint8_t PIN_I2C_SDA=1, PIN_I2C_SCL=2;
static const uint8_t BNO_ADDR=0x4A;
static const float    STILL_GYRO=0.05f;   // rad/s 이하 = 정지
static const uint32_t STILL_MS=800;       // 정지 지속 → 자동 tare
static const float    APEX_GYRO=0.10f, APEX_MIN_TILT=3.0f;
static const uint16_t TELEM_HZ=30;        // ESP32→앱 IMU 텔레메트리 송신율
static const float    RAD2DEG=57.2957795f;

/* ---- 듀얼 모터 구성 ----
 * NODE_IDS      : 한 CAN-A 버스의 두 ODrive node_id (set_node_id.py 로 1/2 저장).
 * MOTOR_SIGN    : +command 가 두 모터에서 같은 물리 방향이 되도록 하는 부호.
 *                 무대 조립 후 한 모터가 반대로 돌면 그쪽을 -1.0f 로 바꿔라.
 * MOTOR_REL_PHASE: 모터별 상대 위상[rad]. 평행 swing={0,0} (batten 수평 유지),
 *                 Z-twist={0, M_PI} (중심축 비틀기). 한 줄만 바꾸면 모드 전환. */
static const uint8_t NUM_MOTORS = 2;
static const uint8_t NODE_IDS[NUM_MOTORS]        = { 1, 2 };
static const float   MOTOR_REL_PHASE[NUM_MOTORS] = { 0.0f, 0.0f };   // 평행 swing
// 극성 부호: 앱 flags 로 런타임 제어 (START/극성변경 시 latch). 기본 = node2 미러 반전.
static const uint8_t POL_DEFAULT = 0x02;   // bit1=모터2 반전 (앱 기본 체크와 일치)

/* ---- per-motor 게인/한계 (arm 때 CAN 으로 명시 적용, RAM only) ----
 * 각 보드 저장값에 의존 안 하고 펌웨어가 직접 set → 재현성 + 라이브 튜닝.
 * 모터1 = P29 튜닝값(0.145). 모터2 는 공장 default 라 거칠어서 vel_gain 을
 * 여기서 스윕(0.10→0.07→0.05…)하며 그 개체의 부드러운 점을 찾는다.
 * vel_gain 은 영구저장 안 함(0x1F 안 보냄) — 값 바꿔 재플래시로 비교만. */
static const float   VEL_GAIN[NUM_MOTORS]   = { 0.145f, 0.10f };   // ★ 모터2 스윕 대상
static const float   VEL_INT[NUM_MOTORS]    = { 1.0f,   1.0f  };
static const float   CURRENT_LIM[NUM_MOTORS]= { 10.0f,  10.0f };   // 모터2 공장 60 → 10 통일
static const float   VEL_LIMIT_BOARD        = 12.0f;  // 보드 vel_limit(여유). 진폭은 VEL_LIM=5 로 별도 클램프

/* ---- ODrive 공통 파라미터 ---- */
static const float    POS_GAIN_SOFT  = 5.0f;     // 소프트 (저장값 50 → RAM 5)
static const float    GEAR           = 8.0f;     // 출력→모터 기어비
static const float    VEL_LIM        = 9.0f;     // rev/s 안전 vel 클램프 (1Hz·60°의 peak 8.4 를 안 깎게). 앱 진폭곡선이 실제 shaping, 이건 천장
static const float    ALPHA_MAX      = 308.0f;   // turn/s² (10A 기준 가속 한계, 진폭 클램프)
static const float    AMP_MAX_TURN   = 1.34f;    // 모터 절대 진폭 상한 (=출력 ±60°, 1.333turn). 저주파에서 bind
static const float    FREQ_MAX       = 5.0f;     // Hz 상한

/* ---- 오실레이터 / 스트림 lock ---- */
static const uint16_t UPDATE_HZ      = 200;      // ESP32→ODrive Set_Input_Pos 송신율 (무선율과 무관)
                                                 // 60→200: 위치 스텝 1/3.3 → 큰 진폭에서도 매끈.
static const float    RAMP_S         = 1.0f;     // 진폭 fade in/out [s]
static const float    MORPH_S        = 0.30f;    // 파형 전환 크로스페이드 [s] (C1 연속)
static const float    PARAM_LPF      = 0.08f;    // freq/amp 추종 LPF
static const uint32_t COMM_TIMEOUT_MS = 500;     // 무수신 → IDLE [ms]
static const uint32_t STATUS_MS      = 500;

/* ---- 와이어 프로토콜 (protocol.py 와 동일) ---- */
static const uint16_t WIRE_MAGIC = 0x0DCA;
static const uint16_t TELEM_MAGIC = 0x0DCB;   // ESP32→앱 IMU 텔레메트리
static const uint16_t UDP_PORT   = 4210;
static const int      PKT_SIZE   = 20;
static const int      TELEM_SIZE = 54;   // +m1_iq,m2_iq,vbus,ibus (전류/전원 모니터)
static const uint8_t  FLAG_REQ_TARE = 0x04;   // flags bit2: IMU 0점 재설정 요청

/* ---- can_simple cmd id ---- */
static const uint8_t CMD_HEARTBEAT=0x01, CMD_ENCODER_ESTIMATES=0x09;
static const uint8_t CMD_GET_IQ=0x14, CMD_GET_BUS_VI=0x17;   // 전류/버스 모니터(RTR 요청)
static const uint8_t CMD_SET_AXIS_STATE=0x07, CMD_SET_CONTROLLER_MODE=0x0B;
static const uint8_t CMD_SET_INPUT_VEL=0x0D, CMD_SET_INPUT_POS=0x0C;
static const uint8_t CMD_SET_POS_GAIN=0x1A, CMD_CLEAR_ERRORS=0x18;
static const uint8_t CMD_SET_LIMITS=0x0F, CMD_SET_VEL_GAINS=0x1B;
static const uint8_t CMD_SET_INPUT_TORQUE=0x0E;
static const uint8_t CMD_SET_TRAJ_VEL=0x11, CMD_SET_TRAJ_ACCEL=0x12;   // trap_traj 한계(리프트 속도)

/* ---- ODrive enum ---- */
static const uint32_t AXIS_IDLE=1, AXIS_CLOSED_LOOP=8;
static const uint32_t CTRL_TORQUE=1, CTRL_VELOCITY=2, CTRL_POSITION=3;
static const uint32_t INPUT_PASSTHROUGH=1, INPUT_VEL_RAMP=2, INPUT_TRAP_TRAJ=5;

/* ---- 파형 enum (protocol.py 와 동일) ---- */
static const uint8_t WAVE_SINE=0, WAVE_SWING=1, WAVE_PUMP=2, WAVE_LIFTDROP=3;

/* ---- 토크 펌핑 파라미터 (보수적 — 위험하니 작게 시작) ---- */
static const float PUMP_KGAIN_MAX = 1.00f;  // 펌프게인 슬라이더 100% 시 [N·m/(turn/s)]
static const float PUMP_K_BRAKE   = 1.00f;  // 진폭한계 초과 시 제동게인
static const float PUMP_T_MAX     = 0.50f;  // 모터 토크 하드 캡 [N·m] (=출력 ~4N·m). 전류 주의(폴리퓨즈)
static const float PUMP_VEL_MAX   = 6.0f;   // 모터 속도 폭주 한계 [turn/s] → 정지 (60°/0.4Hz peak~3.4 여유)
static const float PUMP_IMU_MAXTILT = 80.0f;// IMU tilt 안전 한계 [출력°] → 정지
static const float PUMP_AMP_HARD  = 1.5f;   // |각도|>한계×이배 → 정지(제동 실패 대비)
static const float PUMP_VEL_LPF   = 0.25f;  // 펌프용 속도 EMA

/* ---- 들어올림+낙하 수동(좌/우 버튼) 파라미터 ----
 * 버튼 누르고 있는 동안: 위치제어 스트리밍으로 ±LD_AMP 로 가서 대기(싱크 유지).
 * 떼면: 토크0 프리폴(중력) + twist 동기보정(양끝 안 벌어지게). 사용자가 타이밍 조절. */
static const float    LD_AMP = 2.0f;           // ±각 [모터 turn] (=출력 90°)
static const float    LD_POS_GAIN = 40.0f;     // 리프트용 위치게인(무거운 암 추종)
static const float    LD_DRIVE_SPEED = 2.0f;   // 버튼 시 목표로 가는 평균 속도 [turn/s]
static const float    LD_RELEASE_RAMP_S = 0.25f; // 떼면 게인 0으로 낮추는 시간 [s] (터턱 방지)

static const float TWO_PI_F = 6.28318530718f;

MCP2515 mcp2515(PIN_MCP_CS, 10000000UL, &SPI);
WiFiUDP  udp;
uint8_t  udpbuf[64];

/* ---- IMU 상태 (센싱 전용, 제어엔 안 씀) ---- */
Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t sv;
bool  g_imu_ok=false; uint32_t g_imu_last_init=0;
float qw=1,qi=0,qj=0,qk=0, gx=0,gy=0,gz=0;   // 쿼터니언 + 자이로
bool  got_q=false;
float q0w=1,q0i=0,q0j=0,q0k=0; bool rest_set=false;   // rest(0점)
uint32_t still_since=0; bool is_still=false; bool prev_below_apex=true;
float g_tilt=0, g_pitch=0, g_roll=0;   // rest 대비 (telemetry 용)
bool  g_apex_pulse=false;              // 정점 1회성 플래그(telemetry 에 1번 실어보냄)
bool  prev_tare_bit=false;             // 앱 tare 요청 edge 감지

/* ---- 앱 주소(텔레메트리 회신용) — 마지막 수신 패킷의 송신자 ---- */
IPAddress g_app_ip; uint16_t g_app_port=0; bool g_app_known=false;
uint16_t g_telem_seq=0;

void imu_tare(const char* why);   // 전방 선언 (pollUdp 가 먼저 호출)

/* ---- ODrive 수신 상태 (모터별) ---- */
struct St {
  bool got_hb=false; uint32_t axis_error=0; uint8_t axis_state=0;
  bool got_enc=false; float pos=0, vel=0;
  float iq=0, vbus=0, ibus=0;   // 전류(Iq)/버스전압/버스전류 (RTR 응답)
  uint32_t cnt_hb=0, cnt_enc=0;
};
St st[NUM_MOTORS];

/* ---- 제어/오실레이터 상태 (오실레이터는 공유, center 만 모터별) ---- */
enum Mode { M_IDLE, M_RUNNING, M_STOPPING };
Mode  g_mode = M_IDLE;
float g_center[NUM_MOTORS] = {0};   // 모터별 arm 시점 위치 (중심)
float g_phase = 0;            // 로컬 위상 [rad] (공유)
float g_freq = 0,  g_freq_t = 0;    // 현재/목표 주파수 [Hz]
float g_amp = 0,   g_amp_t = 0;     // 현재/목표 진폭 [모터 turn] (클램프 후)
uint8_t g_waveform = WAVE_SINE;     // 목표 파형 (수신값)
float   g_morph = 0;                // 현재 블렌드 0=SINE..1=SWING (MORPH_S 로 램프)
/* ---- 토크 펌핑 상태 ---- */
bool    g_armed_pump = false;       // 현재 arm 이 토크펌핑 모드인가
float   g_pump_gain = 0;            // 펌프게인 [N·m/(turn/s)] (앱 phase 필드 0..1 × MAX)
float   g_pump_amp_turn = 0;        // 진폭(±) [모터 turn] (앱 amp_deg 에서) — 펌프·리프트 공용
float   g_pump_swingvel = 0;        // 공통(강체 swing) 속도 EMA
/* ---- 들어올림+낙하 상태 ---- */
float   g_ld_cmd = 0;               // 현재 스트리밍 swing 위치 [turn] (떼면 여기서 유지)
float   g_ld_from = 0, g_ld_to = 0; // 드라이브 궤적 시작/끝 [swing turn]
float   g_ld_drive_s = 1.0f;        // 드라이브 소요시간 [s]
uint32_t g_ld_drive_t0 = 0;        // 드라이브 시작
int8_t  g_ld_btn = 0;               // 현재 held 버튼: -1=좌(뒤) 0=없음 +1=우(앞)
int8_t  g_ld_prev_btn = 0;
bool    g_ld_armed = false;          // lift-drop arm 상태 (떼면 IDLE=비여자, 누르면 재arm)
uint8_t g_run = 0;            // 마지막 수신 run
float   g_motor_sign[NUM_MOTORS] = { +1.0f, -1.0f };  // 적용 중 극성 (기본 = 미러)
uint8_t g_rx_flags = POL_DEFAULT;   // 마지막 수신 극성 flags
uint8_t g_pol_flags = POL_DEFAULT;  // 현재 적용된(latch된) 극성 flags
bool g_restart_pending = false;     // 극성 변경 → fade-out 후 자동 재시작 대기
uint16_t g_seq = 0, g_pkt_cnt = 0;
uint32_t g_last_pkt_ms = 0;
uint32_t g_run_t0 = 0, g_stop_t0 = 0;
bool g_mcp_ok = false;
bool g_have_pkt = false;

static inline uint8_t id_node(uint32_t id){ return (id>>5)&0x3F; }
static inline uint8_t id_cmd(uint32_t id){ return id&0x1F; }
static inline float wrapPi(float a){ while(a>M_PI)a-=TWO_PI_F; while(a<-M_PI)a+=TWO_PI_F; return a; }
static inline float lerpf(float a, float b, float m){ return a + (b-a)*m; }

/* node_id → 모터 인덱스 (우리 노드 아니면 -1) */
static inline int motor_index(uint8_t node){
  for(int i=0;i<NUM_MOTORS;++i) if(NODE_IDS[i]==node) return i;
  return -1;
}

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

/* ---- CAN TX (노드 지정) ---- */
bool sendCmd(uint8_t node, uint8_t cmd, const uint8_t* d8){
  struct can_frame f; f.can_id=((uint32_t)node<<5)|cmd; f.can_dlc=8;
  for(int i=0;i<8;++i) f.data[i]=d8?d8[i]:0;
  return mcp2515.sendMessage(&f)==MCP2515::ERROR_OK;
}
void tx_axis_state(uint8_t node, uint32_t s){ uint8_t d[8]={0}; memcpy(d,&s,4); sendCmd(node,CMD_SET_AXIS_STATE,d); }
void tx_ctrl_mode(uint8_t node, uint32_t c,uint32_t i){ uint8_t d[8]={0}; memcpy(d,&c,4); memcpy(d+4,&i,4); sendCmd(node,CMD_SET_CONTROLLER_MODE,d); }
void tx_input_vel(uint8_t node, float v){ uint8_t d[8]={0}; float t=0; memcpy(d,&v,4); memcpy(d+4,&t,4); sendCmd(node,CMD_SET_INPUT_VEL,d); }
void tx_pos_gain(uint8_t node, float g){ uint8_t d[8]={0}; memcpy(d,&g,4); sendCmd(node,CMD_SET_POS_GAIN,d); }
void tx_limits(uint8_t node, float vlim, float ilim){ uint8_t d[8]={0}; memcpy(d,&vlim,4); memcpy(d+4,&ilim,4); sendCmd(node,CMD_SET_LIMITS,d); }
void tx_vel_gains(uint8_t node, float vg, float vi){ uint8_t d[8]={0}; memcpy(d,&vg,4); memcpy(d+4,&vi,4); sendCmd(node,CMD_SET_VEL_GAINS,d); }
void tx_input_torque(uint8_t node, float t){ uint8_t d[8]={0}; memcpy(d,&t,4); sendCmd(node,CMD_SET_INPUT_TORQUE,d); }
void tx_traj_vel(uint8_t node, float v){ uint8_t d[8]={0}; memcpy(d,&v,4); sendCmd(node,CMD_SET_TRAJ_VEL,d); }
void tx_traj_accel(uint8_t node, float a, float dec){ uint8_t d[8]={0}; memcpy(d,&a,4); memcpy(d+4,&dec,4); sendCmd(node,CMD_SET_TRAJ_ACCEL,d); }
void tx_input_pos(uint8_t node, float pos, float vel_ff){
  uint8_t d[8]={0}; memcpy(&d[0],&pos,4);
  int16_t vff=(int16_t)lroundf(vel_ff*1000.0f), tff=0;
  memcpy(&d[4],&vff,2); memcpy(&d[6],&tff,2);
  sendCmd(node,CMD_SET_INPUT_POS,d);
}
void tx_clear_errors(uint8_t node){
  struct can_frame f; f.can_id=((uint32_t)node<<5)|CMD_CLEAR_ERRORS;
  f.can_dlc=1; f.data[0]=0; mcp2515.sendMessage(&f);
}

/* ---- CAN RX (heartbeat/encoder, 모터별 분배) ---- */
void drainRx(){
  struct can_frame f;
  while(mcp2515.readMessage(&f)==MCP2515::ERROR_OK){
    uint32_t id=f.can_id&CAN_SFF_MASK;
    int mi = motor_index(id_node(id));
    if(mi<0) continue;
    if(id_cmd(id)==CMD_HEARTBEAT && f.can_dlc>=5){
      st[mi].cnt_hb++; memcpy(&st[mi].axis_error,&f.data[0],4); st[mi].axis_state=f.data[4]; st[mi].got_hb=true;
    } else if(id_cmd(id)==CMD_ENCODER_ESTIMATES && f.can_dlc>=8){
      st[mi].cnt_enc++; memcpy(&st[mi].pos,&f.data[0],4); memcpy(&st[mi].vel,&f.data[4],4); st[mi].got_enc=true;
    } else if(id_cmd(id)==CMD_GET_IQ && f.can_dlc>=8){
      memcpy(&st[mi].iq,&f.data[4],4);   // Iq_Measured @4
    } else if(id_cmd(id)==CMD_GET_BUS_VI && f.can_dlc>=8){
      memcpy(&st[mi].vbus,&f.data[0],4); memcpy(&st[mi].ibus,&f.data[4],4);
    }
  }
}

/* 전류/버스 모니터 RTR 요청 (rate 기본 0 이라 on-demand 요청) */
void req_monitor(){
  for(int i=0;i<NUM_MOTORS;++i){
    for(uint8_t cmd : {CMD_GET_IQ, CMD_GET_BUS_VI}){
      struct can_frame f; f.can_id=(((uint32_t)NODE_IDS[i]<<5)|cmd)|CAN_RTR_FLAG;
      f.can_dlc=8; for(int k=0;k<8;++k) f.data[k]=0;
      mcp2515.sendMessage(&f);
    }
  }
}

/* 양쪽 모터 IDLE + 에러클리어 (어떤 정지 경로든 둘 다 끈다) */
void fault_stop(const char* why){
  for(int i=0;i<NUM_MOTORS;++i){ if(g_armed_pump) tx_input_torque(NODE_IDS[i],0.0f); tx_axis_state(NODE_IDS[i], AXIS_IDLE); }
  delay(2);
  for(int i=0;i<NUM_MOTORS;++i){ tx_clear_errors(NODE_IDS[i]); }
  g_mode=M_IDLE; g_restart_pending=false; g_armed_pump=false;
  Serial.print(">>> 정지+에러클리어 (양쪽 IDLE) — "); Serial.println(why);
}

// 두 모터 모두 VEL arm 으로 라이브 pos 확보 → POS/PASSTHROUGH soft gain 전환. center 캡처.
bool arm_and_center(){
  // 1) 두 모터 동시에 VEL_RAMP arm 준비 + per-motor 게인/한계 명시 적용 (RAM)
  for(int i=0;i<NUM_MOTORS;++i){
    tx_clear_errors(NODE_IDS[i]); delay(5);
    tx_limits(NODE_IDS[i], VEL_LIMIT_BOARD, CURRENT_LIM[i]); delay(5);
    tx_vel_gains(NODE_IDS[i], VEL_GAIN[i], VEL_INT[i]); delay(5);
    tx_ctrl_mode(NODE_IDS[i], CTRL_VELOCITY, INPUT_VEL_RAMP); delay(5);
    tx_input_vel(NODE_IDS[i], 0.0f); delay(5);
    Serial.print("config node"); Serial.print(NODE_IDS[i]);
    Serial.print(": vel_gain="); Serial.print(VEL_GAIN[i],3);
    Serial.print(" curr_lim="); Serial.print(CURRENT_LIM[i],1);
    Serial.print(" vel_lim="); Serial.println(VEL_LIMIT_BOARD,1);
    st[i].got_enc=false;
  }
  for(int i=0;i<NUM_MOTORS;++i){ tx_axis_state(NODE_IDS[i], AXIS_CLOSED_LOOP); }

  // 2) 두 모터 모두 CLOSED_LOOP + 엔코더 수신될 때까지 대기 (input_vel 0 계속 먹임)
  uint32_t t0=millis();
  while(millis()-t0<1500){
    drainRx();
    bool all=true;
    for(int i=0;i<NUM_MOTORS;++i)
      if(!(st[i].axis_state==AXIS_CLOSED_LOOP && st[i].got_enc)) all=false;
    if(all) break;
    for(int i=0;i<NUM_MOTORS;++i) tx_input_vel(NODE_IDS[i], 0.0f);
    delay(10);
  }
  for(int i=0;i<NUM_MOTORS;++i){
    if(st[i].axis_state!=AXIS_CLOSED_LOOP){
      Serial.print("[오류] node"); Serial.print(NODE_IDS[i]);
      Serial.print(" CLOSED_LOOP 실패 state="); Serial.print(st[i].axis_state);
      Serial.print(" err=0x"); Serial.println(st[i].axis_error,HEX);
      fault_stop("arm 실패"); return false;
    }
  }

  // 3) 두 모터 center 캡처 + soft gain + POS/PASSTHROUGH 전환 (각자 자기 center hold)
  for(int i=0;i<NUM_MOTORS;++i){
    g_center[i]=st[i].pos;
    tx_pos_gain(NODE_IDS[i], POS_GAIN_SOFT); delay(5);
    tx_ctrl_mode(NODE_IDS[i], CTRL_POSITION, INPUT_PASSTHROUGH); delay(5);
    tx_input_pos(NODE_IDS[i], g_center[i], 0.0f); delay(5);
    Serial.print("center["); Serial.print(NODE_IDS[i]); Serial.print("]=");
    Serial.print(g_center[i],4); Serial.println(" turn");
  }
  return true;
}

// 극성 flags → 모터별 부호로 적용(latch). amp=0 시점에 호출해야 안전(슬램 방지).
void apply_polarity(uint8_t flags){
  g_motor_sign[0] = (flags & 0x01) ? -1.0f : +1.0f;
  g_motor_sign[1] = (flags & 0x02) ? -1.0f : +1.0f;
  g_pol_flags = flags;
  Serial.print(">>> 극성 적용: m1="); Serial.print(g_motor_sign[0],0);
  Serial.print(" m2="); Serial.println(g_motor_sign[1],0);
}

// 토크 펌핑 arm: 검증된 VEL 모드로 먼저 arm(직접 토크진입은 실패함) → center 캡처
//              → CLOSED_LOOP 유지한 채 TORQUE/PASSTHROUGH 로 전환, torque=0.
bool arm_torque_pump(){
  // 1) VEL_RAMP 로 arm (arm_and_center 와 동일 — 직접 토크 arm 은 ODrive 가 거부)
  for(int i=0;i<NUM_MOTORS;++i){
    tx_clear_errors(NODE_IDS[i]); delay(5);
    tx_limits(NODE_IDS[i], VEL_LIMIT_BOARD, CURRENT_LIM[i]); delay(5);
    tx_ctrl_mode(NODE_IDS[i], CTRL_VELOCITY, INPUT_VEL_RAMP); delay(5);
    tx_input_vel(NODE_IDS[i], 0.0f); delay(5);
    st[i].got_enc=false;
  }
  for(int i=0;i<NUM_MOTORS;++i){ tx_axis_state(NODE_IDS[i], AXIS_CLOSED_LOOP); }
  uint32_t t0=millis();
  while(millis()-t0<1500){
    drainRx();
    bool all=true;
    for(int i=0;i<NUM_MOTORS;++i)
      if(!(st[i].axis_state==AXIS_CLOSED_LOOP && st[i].got_enc)) all=false;
    if(all) break;
    for(int i=0;i<NUM_MOTORS;++i) tx_input_vel(NODE_IDS[i], 0.0f);
    delay(10);
  }
  for(int i=0;i<NUM_MOTORS;++i){
    if(st[i].axis_state!=AXIS_CLOSED_LOOP){
      Serial.print("[오류] node"); Serial.print(NODE_IDS[i]); Serial.print(" 토크 arm 실패 err=0x");
      Serial.println(st[i].axis_error,HEX); fault_stop("토크 arm 실패"); return false;
    }
  }
  // 2) center 캡처 + 토크 모드로 전환 (armed 유지)
  for(int i=0;i<NUM_MOTORS;++i){
    g_center[i]=st[i].pos;
    tx_ctrl_mode(NODE_IDS[i], CTRL_TORQUE, INPUT_PASSTHROUGH); delay(5);
    tx_input_torque(NODE_IDS[i], 0.0f); delay(5);
    Serial.print("pump center["); Serial.print(NODE_IDS[i]); Serial.print("]="); Serial.println(g_center[i],4);
  }
  apply_polarity(g_rx_flags);   // 미러보정 latch (사인 모드와 동일 좌표)
  g_pump_swingvel=0;
  g_armed_pump=true;
  return true;
}

// lift-drop 재arm: 비여자(IDLE) 상태에서 다시 CLOSED_LOOP/POSITION 으로 (center 유지).
// VEL(0) 으로 진입(현재 위치 모르고도 arm) → 현재위치에서 POSITION. 빠른 catch 용(블로킹 짧게).
bool ld_rearm(){
  for(int i=0;i<NUM_MOTORS;++i){
    tx_clear_errors(NODE_IDS[i]); delay(3);
    tx_ctrl_mode(NODE_IDS[i], CTRL_VELOCITY, INPUT_VEL_RAMP); delay(3);
    tx_input_vel(NODE_IDS[i], 0.0f); delay(3);
    st[i].got_enc=false;
  }
  for(int i=0;i<NUM_MOTORS;++i) tx_axis_state(NODE_IDS[i], AXIS_CLOSED_LOOP);
  uint32_t t0=millis();
  while(millis()-t0<1000){
    drainRx();
    bool all=true;
    for(int i=0;i<NUM_MOTORS;++i) if(!(st[i].axis_state==AXIS_CLOSED_LOOP && st[i].got_enc)) all=false;
    if(all) break;
    for(int i=0;i<NUM_MOTORS;++i) tx_input_vel(NODE_IDS[i], 0.0f);
    delay(5);
  }
  for(int i=0;i<NUM_MOTORS;++i)
    if(st[i].axis_state!=AXIS_CLOSED_LOOP){ fault_stop("재arm 실패"); return false; }
  for(int i=0;i<NUM_MOTORS;++i){
    tx_pos_gain(NODE_IDS[i], LD_POS_GAIN);
    tx_ctrl_mode(NODE_IDS[i], CTRL_POSITION, INPUT_PASSTHROUGH);
    tx_input_pos(NODE_IDS[i], st[i].pos, 0.0f);   // 현재 위치(점프 방지)
  }
  float sa=0; for(int i=0;i<NUM_MOTORS;++i) sa += g_motor_sign[i]*(st[i].pos - g_center[i]); sa/=NUM_MOTORS;
  g_ld_cmd = sa; g_ld_armed = true;
  return true;
}

void start_running(float start_phase){
  if(g_waveform==WAVE_LIFTDROP){            // 들어올림+낙하 — POSITION arm
    if(!arm_and_center()) return;          // VEL→POS arm, g_center 캡처
    apply_polarity(g_rx_flags);
    for(int i=0;i<NUM_MOTORS;++i){
      tx_pos_gain(NODE_IDS[i], LD_POS_GAIN); delay(2);
      tx_ctrl_mode(NODE_IDS[i], CTRL_POSITION, INPUT_PASSTHROUGH); delay(2);  // 위치 스트리밍만(토크X)
    }
    g_armed_pump=true; g_ld_armed=true;
    g_ld_cmd=0; g_ld_btn=0; g_ld_prev_btn=0;   // 중심에서 대기 (버튼 누른 만큼만 이동)
    g_run_t0=millis(); g_mode=M_RUNNING;
    Serial.println(">>> RUNNING (들어올림+낙하) — 자동 60° 래칫");
    return;
  }
  if(g_waveform==WAVE_PUMP){                // 토크 펌핑
    if(!arm_torque_pump()) return;
    g_run_t0 = millis(); g_mode = M_RUNNING;
    Serial.println(">>> RUNNING (토크 펌핑) — 세기 0 부터");
    return;
  }
  if(!arm_and_center()) return;
  apply_polarity(g_rx_flags);     // 시작 시 극성 latch (수신 flags 반영)
  g_phase = start_phase;          // 앱 위상에 맞춰 시작
  g_freq  = g_freq_t;             // 목표 주파수에서 시작 (진폭은 env 로 fade-in)
  g_amp   = 0;                    // env 와 별개로 amp 도 0 에서
  g_run_t0 = millis();
  g_mode = M_RUNNING;
  Serial.println(">>> RUNNING (듀얼 모터 UDP LFO)");
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
          g_rx_flags = udpbuf[6];   // flags(극성) — u16 LE 의 하위바이트
          memcpy(&freq,&udpbuf[8],4);
          memcpy(&amp_deg,&udpbuf[12],4);
          memcpy(&phase,&udpbuf[16],4);
          // 안전 클램프
          if(freq<0) freq=0; if(freq>FREQ_MAX) freq=FREQ_MAX;
          g_waveform = (wave<=WAVE_LIFTDROP) ? wave : WAVE_SINE;
          if(g_waveform==WAVE_PUMP || g_waveform==WAVE_LIFTDROP){
            // 펌프: phase=게인, amp_deg=진폭. 리프트드롭: 90° 하드코딩 + 좌/우 버튼.
            float pg = phase; if(pg<0)pg=0; if(pg>1)pg=1;
            g_pump_gain = pg * PUMP_KGAIN_MAX;
            g_pump_amp_turn = deg_to_turn(amp_deg);
            // 좌/우 버튼 레벨 (우선 우(앞) > 좌(뒤))
            if(g_rx_flags & 0x10) g_ld_btn = +1;        // BTN_RIGHT
            else if(g_rx_flags & 0x08) g_ld_btn = -1;   // BTN_LEFT
            else g_ld_btn = 0;
          } else {
            // 오실레이터 모드: 목표 파형 peak 계수로 클램프 (출력 시점 재클램프)
            float pv = (g_waveform==WAVE_SWING)?WT_PEAKVEL_SWING:WT_PEAKVEL_SINE;
            float pa = (g_waveform==WAVE_SWING)?WT_PEAKACC_SWING:WT_PEAKACC_SINE;
            g_freq_t = freq;
            g_amp_t  = clamp_amp(deg_to_turn(amp_deg), freq, pv, pa);
          }
          g_run = run; g_seq = seq; g_pkt_cnt++;
          g_last_pkt_ms = millis();
          g_have_pkt = true;
          // 텔레메트리 회신 주소 = 이 패킷 송신자
          g_app_ip = udp.remoteIP(); g_app_port = udp.remotePort(); g_app_known = true;
          // IMU tare 요청 (flags bit2) rising-edge 감지
          bool tare_bit = (g_rx_flags & FLAG_REQ_TARE) != 0;
          if(tare_bit && !prev_tare_bit) imu_tare("앱 요청");
          prev_tare_bit = tare_bit;
          (void)phase;  // 단일 오실레이터: 외부 위상 lock 안 함 (Wi-Fi 지터로 튐 유발).
                        // 두 모터 위상 관계는 MOTOR_REL_PHASE 로 로컬에서 결정.
        }
      }
    }
    n = udp.parsePacket();
  }
}

/* ============================ IMU (센싱 전용) ============================ */
bool imu_enableReports(){
  bool ok=true;
  ok &= bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 10000);
  ok &= bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000);
  return ok;
}
bool imu_init(){
  if(!bno08x.begin_I2C(BNO_ADDR, &Wire)) return false;
  imu_enableReports();
  return true;
}
void imu_tare(const char* why){
  if(!got_q){ return; }
  q0w=qw; q0i=qi; q0j=qj; q0k=qk; rest_set=true;
  Serial.print(">>> IMU tare(0점) — "); Serial.println(why);
}
// rest 대비 상대회전 r=conj(q0)⊗q → tilt(크기) + 부호있는 pitch/roll[deg]
void imu_compute(){
  if(!rest_set || !got_q){ g_tilt=g_pitch=g_roll=0; return; }
  float aw=q0w, ax=-q0i, ay=-q0j, az=-q0k;
  float bw=qw,  bx=qi,   by=qj,   bz=qk;
  float rw=aw*bw-ax*bx-ay*by-az*bz;
  float rx=aw*bx+ax*bw+ay*bz-az*by;
  float ry=aw*by-ax*bz+ay*bw+az*bx;
  float rz=aw*bz+ax*by-ay*bx+az*bw;
  float w=fabsf(rw); if(w>1.0f)w=1.0f;
  g_tilt = 2.0f*acosf(w)*RAD2DEG;
  g_roll = atan2f(2.0f*(rw*rx+ry*rz), 1.0f-2.0f*(rx*rx+ry*ry))*RAD2DEG;
  float s=2.0f*(rw*ry-rz*rx); if(s>1)s=1; if(s<-1)s=-1;
  g_pitch= asinf(s)*RAD2DEG;
}
void imu_poll(){
  if(!g_imu_ok){
    if(millis()-g_imu_last_init>=1000){ g_imu_last_init=millis();
      if(imu_init()){ g_imu_ok=true; Serial.println(">>> BNO085 연결됨."); } }
    return;
  }
  if(bno08x.wasReset()){ imu_enableReports(); rest_set=false; }
  while(bno08x.getSensorEvent(&sv)){
    if(sv.sensorId==SH2_GAME_ROTATION_VECTOR){
      qw=sv.un.gameRotationVector.real; qi=sv.un.gameRotationVector.i;
      qj=sv.un.gameRotationVector.j;    qk=sv.un.gameRotationVector.k; got_q=true;
    } else if(sv.sensorId==SH2_GYROSCOPE_CALIBRATED){
      gx=sv.un.gyroscope.x; gy=sv.un.gyroscope.y; gz=sv.un.gyroscope.z;
    }
  }
  uint32_t now=millis();
  float gmag=sqrtf(gx*gx+gy*gy+gz*gz);
  if(gmag<STILL_GYRO){ if(still_since==0)still_since=now;
    if(!is_still && now-still_since>=STILL_MS){ is_still=true; if(!rest_set) imu_tare("자동(정지)"); } }
  else { still_since=0; is_still=false; }
  imu_compute();
  bool below=(gmag<APEX_GYRO);
  if(below && !prev_below_apex && g_tilt>APEX_MIN_TILT) g_apex_pulse=true;
  prev_below_apex=below;
}
// ESP32→앱 IMU 텔레메트리 (protocol.py TELEM_FMT 와 동일 38바이트)
void send_telem(){
  if(!g_app_known || WiFi.status()!=WL_CONNECTED) return;
  uint8_t b[TELEM_SIZE]; memset(b,0,TELEM_SIZE);
  uint16_t magic=TELEM_MAGIC; memcpy(&b[0],&magic,2);
  memcpy(&b[2],&g_telem_seq,2); g_telem_seq++;
  uint8_t status=0;
  if(g_imu_ok) status|=0x01; if(rest_set) status|=0x02;
  if(is_still) status|=0x04; if(g_apex_pulse) status|=0x08;
  b[4]=status; b[5]=0;
  memcpy(&b[6],&g_tilt,4); memcpy(&b[10],&g_pitch,4); memcpy(&b[14],&g_roll,4);
  memcpy(&b[18],&gx,4); memcpy(&b[22],&gy,4); memcpy(&b[26],&gz,4);
  float m1=st[0].pos, m2=st[1].pos;
  memcpy(&b[30],&m1,4); memcpy(&b[34],&m2,4);
  // 전류/전원: m1_iq, m2_iq, vbus(유효값), ibus(합=폴리퓨즈 예산)
  float iq1=st[0].iq, iq2=st[1].iq;
  float vbus = (st[0].vbus>1.0f)?st[0].vbus:st[1].vbus;
  float ibus = st[0].ibus + st[1].ibus;
  memcpy(&b[38],&iq1,4); memcpy(&b[42],&iq2,4); memcpy(&b[46],&vbus,4); memcpy(&b[50],&ibus,4);
  udp.beginPacket(g_app_ip, g_app_port); udp.write(b, TELEM_SIZE); udp.endPacket();
  g_apex_pulse=false;
}

void setup(){
  Serial.begin(115200);
  for(int i=0;i<30 && !Serial;++i) delay(100);
  delay(200);
  Serial.println();
  Serial.println("[esp32t2can] Phase 5: 듀얼 모터 UDP LFO 제어");
  Serial.print("  NODE_IDS = ");
  for(int i=0;i<NUM_MOTORS;++i){ Serial.print(NODE_IDS[i]); Serial.print(" "); }
  Serial.println();

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

  // IMU(BNO085) I2C — 없어도 모터는 정상(센싱 레이어는 선택)
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL); Wire.setClock(400000);
  if(imu_init()){ g_imu_ok=true; Serial.println(">>> BNO085 OK (센싱 전용, 정지 시 자동 0점)"); }
  else Serial.println("[경고] BNO085 못 찾음 — 모터는 정상, IMU 없이 진행");
  g_imu_last_init=millis();

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
  imu_poll();
  uint32_t now = millis();

  // ---- 전류/버스 모니터 RTR 요청 (40Hz — 피크 더 잘 잡게) ----
  static uint32_t last_mon=0;
  if(now-last_mon >= 25){ last_mon=now; req_monitor(); }
  // ---- IMU 텔레메트리 송신 (앱 표시용, 제어와 무관) ----
  static uint32_t last_telem=0;
  if(now-last_telem >= (uint32_t)(1000/TELEM_HZ)){ last_telem=now; send_telem(); }
  // ---- 시리얼 't' = IMU 수동 재-tare ----
  if(Serial.available()){ int c=Serial.read(); if(c=='t'||c=='T') imu_tare("시리얼"); }

  // ---- run 플래그 전이 ----
  if(g_have_pkt){
    if(g_run==0) g_restart_pending=false;   // 앱 STOP 은 극성-재시작보다 우선
    if(g_run==1 && g_mode==M_IDLE)      start_running(0.0f);
    else if(g_run==0 && g_mode==M_RUNNING){ g_mode=M_STOPPING; g_stop_t0=now; Serial.println(">>> STOP(app) fade-out"); }
    // 운전 중 극성 변경 → fade-out 후 자동 재시작 (즉시 반전 슬램 방지). 펌프모드 제외.
    else if(g_run==1 && g_mode==M_RUNNING && !g_armed_pump && (g_rx_flags&0x03)!=(g_pol_flags&0x03)){
      g_mode=M_STOPPING; g_stop_t0=now; g_restart_pending=true;
      Serial.println(">>> 극성 변경 → fade-out 후 재시작");
    }
    // 토크모드(펌프/리프트)↔오실레이터 전환은 제어모드 달라 재arm 필요 → 정지 후 재START.
    // (펌프↔리프트는 둘 다 토크라 재arm 불필요 — 출력법칙만 라이브 전환)
    else if(g_run==1 && g_mode==M_RUNNING &&
            ((g_waveform==WAVE_PUMP||g_waveform==WAVE_LIFTDROP)!=g_armed_pump)){
      fault_stop("모드 전환(토크↔파형) — START 다시 누르세요");
    }
  }
  // ---- 통신 두절 → IDLE ----
  if(g_mode==M_RUNNING && (now-g_last_pkt_ms)>COMM_TIMEOUT_MS){
    g_mode=M_STOPPING; g_stop_t0=now; g_restart_pending=false;
    Serial.println(">>> 통신 두절 → fade-out + IDLE");
  }
  // ---- BOOT = 로컬 비상정지 ----
  static int lb=HIGH; static uint32_t lbm=0;
  int b=digitalRead(PIN_BOOT);
  if(b==LOW && lb==HIGH && (now-lbm)>300){ lbm=now; if(g_mode!=M_IDLE) fault_stop("BOOT 비상정지"); }
  lb=b;
  // ---- ODrive fault (어느 한쪽이라도 자체 disarm → 양쪽 정지) ----
  // 단, lift-drop 프리폴(일부러 IDLE 비여자)은 fault 아님 → state 검사 제외, 에러만 본다.
  if(g_mode!=M_IDLE){
    float rt=(now-g_run_t0)/1000.0f;
    bool intentional_idle = (g_waveform==WAVE_LIFTDROP && !g_ld_armed);
    if(rt>0.4f){
      for(int i=0;i<NUM_MOTORS;++i){
        if(!st[i].got_hb) continue;
        bool state_bad = (st[i].axis_state!=AXIS_CLOSED_LOOP) && !intentional_idle;
        if(state_bad || st[i].axis_error!=0){
          Serial.print("[ODrive fault] node"); Serial.print(NODE_IDS[i]);
          Serial.print(" state="); Serial.print(st[i].axis_state);
          Serial.print(" err=0x"); Serial.println(st[i].axis_error,HEX);
          fault_stop("ODrive fault"); break;
        }
      }
    }
  }
  // ---- 펌프 안전 한계 (토크모드 폭주 방지) ----
  if(g_armed_pump && g_mode==M_RUNNING){
    float rt=(now-g_run_t0)/1000.0f;
    if(rt>0.3f){
      const char* why=nullptr;
      bool is_ld = (g_waveform==WAVE_LIFTDROP);
      float amp_lim = is_ld ? LD_AMP : g_pump_amp_turn;
      float hard    = is_ld ? 1.15f : PUMP_AMP_HARD;   // lift-drop: ±90°×1.15(±103°) 넘으면 즉시 정지
      if(!is_ld && g_imu_ok && rest_set && g_tilt>PUMP_IMU_MAXTILT) why="IMU tilt 한계";  // lift-drop은 IMU 안 봄
      for(int i=0;i<NUM_MOTORS && !why;++i){
        if(fabsf(st[i].vel)>PUMP_VEL_MAX) why="속도 폭주";
        else if(amp_lim>0 && fabsf(st[i].pos-g_center[i])>amp_lim*hard) why="각도 한계 초과(±90°)";
      }
      if(why) fault_stop(why);
    }
  }

  // ---- 200Hz 토크-기반(펌프/리프트드롭) 출력 ----
  static uint32_t last_txp=0;
  if(g_armed_pump && (g_mode==M_RUNNING||g_mode==M_STOPPING) && (now-last_txp)>=(1000/UPDATE_HZ)){
    last_txp=now;
    // 공통(강체 swing) 각/속도 = 미러보정 평균 (twist 무시)
    float sv=0, sa=0;
    for(int i=0;i<NUM_MOTORS;++i){ sv += g_motor_sign[i]*st[i].vel; sa += g_motor_sign[i]*(st[i].pos-g_center[i]); }
    sv/=NUM_MOTORS; sa/=NUM_MOTORS;
    g_pump_swingvel += (sv - g_pump_swingvel)*PUMP_VEL_LPF;

    if(g_waveform==WAVE_LIFTDROP){
      // === 수동: 위치제어 only(토크X=폭주불가). 버튼 누른 동안만 ±90° 로 이동, 떼면 그 자리 유지 ===
      if(g_mode==M_STOPPING){
        for(int i=0;i<NUM_MOTORS;++i){ tx_input_pos(NODE_IDS[i], g_center[i]+g_motor_sign[i]*g_ld_cmd, 0.0f); tx_axis_state(NODE_IDS[i],AXIS_IDLE); }
        g_mode=M_IDLE; g_armed_pump=false; Serial.println(">>> 리프트 정지 (IDLE)");
      } else if(g_ld_btn != 0){
        // 버튼 held → (필요시 재arm) 위치제어로 ±90° 로 ease. 폭주 불가(위치제어).
        if(!g_ld_armed){ if(!ld_rearm()){ g_ld_prev_btn=0; goto ld_done; } }   // 비여자였으면 재arm(catch)
        if(g_ld_btn != g_ld_prev_btn){
          g_ld_from=g_ld_cmd; g_ld_to=(float)g_ld_btn*LD_AMP; g_ld_drive_t0=now;
          g_ld_drive_s=fabsf(g_ld_to-g_ld_from)/LD_DRIVE_SPEED; if(g_ld_drive_s<0.3f) g_ld_drive_s=0.3f;
        }
        float p=(now-g_ld_drive_t0)/1000.0f/g_ld_drive_s; if(p>1.0f)p=1.0f;
        float s=p*p*(3.0f-2.0f*p), ds=6.0f*p*(1.0f-p);
        g_ld_cmd = g_ld_from + (g_ld_to-g_ld_from)*s;
        float vel_sw = (g_ld_to-g_ld_from)*ds/g_ld_drive_s;
        for(int i=0;i<NUM_MOTORS;++i)
          tx_input_pos(NODE_IDS[i], g_center[i] + g_motor_sign[i]*g_ld_cmd, g_motor_sign[i]*vel_sw);
      } else {
        // 버튼 뗌 → IDLE(비여자) = STOP과 동일. 한 번만 끄고 끝(이후 아무 명령 X = 진짜 off).
        if(g_ld_armed){ for(int i=0;i<NUM_MOTORS;++i) tx_axis_state(NODE_IDS[i], AXIS_IDLE); g_ld_armed=false; Serial.println(">>> 프리폴(IDLE 비여자)"); }
      }
      g_ld_prev_btn = g_ld_btn;
      ld_done:;
    } else {
      // === 토크 펌핑 (env fade + anti-damping) ===
      float env;
      if(g_mode==M_RUNNING){ float t=(now-g_run_t0)/1000.0f; env=(t<RAMP_S)?(t/RAMP_S):1.0f; }
      else { float td=(now-g_stop_t0)/1000.0f; env=1.0f-td/RAMP_S;
        if(env<=0.0f){ env=0.0f;
          for(int i=0;i<NUM_MOTORS;++i){ tx_input_torque(NODE_IDS[i],0.0f); tx_axis_state(NODE_IDS[i],AXIS_IDLE); }
          g_mode=M_IDLE; g_armed_pump=false; Serial.println(">>> 펌프 정지 (IDLE)");
        }
      }
      if(g_mode!=M_IDLE){
        float Tm = (fabsf(sa) < g_pump_amp_turn) ? (g_pump_gain*g_pump_swingvel) : (-PUMP_K_BRAKE*g_pump_swingvel);
        if(Tm> PUMP_T_MAX) Tm= PUMP_T_MAX; if(Tm<-PUMP_T_MAX) Tm=-PUMP_T_MAX;
        Tm *= env;
        for(int i=0;i<NUM_MOTORS;++i) tx_input_torque(NODE_IDS[i], g_motor_sign[i]*Tm);
      }
    }
  }

  // ---- 200Hz 오실레이터 출력 (두 모터) ----
  static uint32_t last_tx=0;
  if(!g_armed_pump && (g_mode==M_RUNNING||g_mode==M_STOPPING) && (now-last_tx)>=(1000/UPDATE_HZ)){
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
      if(env<=0.0f){
        env=0.0f;
        if(g_restart_pending){
          // 극성 변경 재시작: 모터는 armed 유지(center hold) + amp=0 이므로 부호만
          // 바꿔 곧바로 fade-in → 재arm 없이 매끈. (amp×env≈0 시점이라 슬램 없음)
          g_restart_pending=false;
          apply_polarity(g_rx_flags);
          g_phase=0.0f; g_run_t0=now; g_mode=M_RUNNING;
          Serial.println(">>> 새 극성으로 재시작(fade-in)");
        } else {
          for(int i=0;i<NUM_MOTORS;++i){ tx_input_pos(NODE_IDS[i], g_center[i], 0.0f); tx_axis_state(NODE_IDS[i], AXIS_IDLE); }
          g_mode=M_IDLE; Serial.println(">>> 양쪽 IDLE");
        }
      }
    }
    if(g_mode!=M_IDLE){
      // 블렌드된 peak 계수로 재클램프 (그네일수록 진폭 더 깎임 = 안전)
      float pv = lerpf(WT_PEAKVEL_SINE, WT_PEAKVEL_SWING, g_morph);
      float pa = lerpf(WT_PEAKACC_SINE, WT_PEAKACC_SWING, g_morph);
      float amp = clamp_amp(g_amp, g_freq, pv, pa);
      // 모터별: 자기 상대 위상에서 lookup → 자기 center 기준 + 부호 적용
      for(int i=0;i<NUM_MOTORS;++i){
        float ph = g_phase + MOTOR_REL_PHASE[i];
        float val  = lerpf(wt_lookup(WT_POS_SINE,ph), wt_lookup(WT_POS_SWING,ph), g_morph);
        float dval = lerpf(wt_lookup(WT_VEL_SINE,ph), wt_lookup(WT_VEL_SWING,ph), g_morph);
        float pos = g_center[i] + g_motor_sign[i]*env*amp*val;
        float vff = g_motor_sign[i]*env*amp*(TWO_PI_F*g_freq)*dval;
        tx_input_pos(NODE_IDS[i], pos, vff);
      }
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
    Serial.print(" wav="); Serial.print(g_waveform==WAVE_LIFTDROP?"LIFT":g_waveform==WAVE_PUMP?"PUMP":g_waveform==WAVE_SWING?"swing":"sine");
    if(g_armed_pump){
      if(g_waveform==WAVE_LIFTDROP){ Serial.print(g_ld_btn>0?" [우→":g_ld_btn<0?" [좌→":" [프리폴"); Serial.print(g_ld_btn?g_ld_to:0.0f,2); Serial.print("]"); }
      else { Serial.print(" pgain="); Serial.print(g_pump_gain,3); }
      Serial.print(" amp="); Serial.print(g_pump_amp_turn,3); Serial.print(" tilt="); Serial.print(g_tilt,1);
    }
    Serial.print(" morph="); Serial.print(g_morph,2);
    Serial.print(" run="); Serial.print(g_run);
    Serial.print(" pkt="); Serial.print(g_pkt_cnt);
    Serial.print(" age="); Serial.print(age); Serial.print("ms");
    for(int i=0;i<NUM_MOTORS;++i){
      if(st[i].got_hb){
        Serial.print(" | n"); Serial.print(NODE_IDS[i]);
        Serial.print(" st="); Serial.print(st[i].axis_state);
        Serial.print(" e=0x"); Serial.print(st[i].axis_error,HEX);
        Serial.print(" p="); Serial.print(st[i].pos,3);
        Serial.print(" iq="); Serial.print(st[i].iq,2);
      }
    }
    Serial.print(" || vbus="); Serial.print(st[0].vbus,1);
    Serial.print(" ibus="); Serial.print(st[0].ibus,2);
    Serial.print("/"); Serial.print(st[1].ibus,2);
    Serial.println();
  }
}
