/*
 * esp32t2can / enc_test.cpp  —  인코더 idle-read 진단 (arm 없이)
 * ============================================================================
 * 목적: "전류(arm) 없이 CAN 으로 위치를 읽을 수 있는가?" 를 직접 확인.
 *
 *   - ODrive 0.6.5 의 0x09 Get_Encoder_Estimates 는 컨트롤러 소스라 IDLE 에서 0.
 *   - 0x0A Get_Encoder_Count(shadow_count, count_in_cpr)는 인코더 드라이버 직접
 *     이라 IDLE(무여자)에도 라이브일 것으로 기대 → 이걸 RTR 로 요청해 확인.
 *
 * 이 펌웨어는 ODrive 에 axis_state/arm 명령을 절대 보내지 않는다. ODrive 는
 * IDLE 그대로(Iq=0). 손으로 모터를 돌리며 시리얼에 0x09 vs 0x0A 를 비교.
 *
 * 기대 결과(가설 맞다면):
 *   - 0x09 pos = 0.000 고정 (idle 이라 죽어 있음)
 *   - 0x0A shadow/cic = 손 따라 변함  ← 전류 0 으로 위치 파악 가능 증명
 *
 * 빌드/플래시:  platformio run -d esp32t2can -e enc-test -t upload --upload-port COM5
 * 배선: ODrive(들) → CAN-A 버스, 24V ON, IDLE. 종단 60Ω.
 * ============================================================================
 */
#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

/* ---- 핀맵 (main.cpp 와 동일) ---- */
static const uint8_t PIN_SPI_SCLK=12, PIN_SPI_MOSI=11, PIN_SPI_MISO=13;
static const uint8_t PIN_MCP_CS=10, PIN_MCP_RST=9;

/* ---- 프로브할 노드 (둘 다 시도, 응답하는 쪽만 보임) ---- */
static const uint8_t NUM_NODES = 2;
static const uint8_t NODE_IDS[NUM_NODES] = { 1, 2 };

/* ---- can_simple cmd id ---- */
static const uint8_t CMD_HEARTBEAT=0x01, CMD_ENCODER_ESTIMATES=0x09, CMD_ENCODER_COUNT=0x0A;

static const uint32_t REQ_MS = 100;     // 0x0A RTR 요청 주기
static const uint32_t PRINT_MS = 250;

MCP2515 mcp2515(PIN_MCP_CS, 10000000UL, &SPI);
bool g_mcp_ok=false;

struct St {
  bool got_hb=false; uint32_t axis_error=0; uint8_t axis_state=0; uint32_t cnt_hb=0;
  bool got09=false; float pos09=0, vel09=0; uint32_t cnt09=0;
  bool got0A=false; int32_t shadow=0, cic=0; uint32_t cnt0A=0;
};
St st[NUM_NODES];

static inline uint8_t id_node(uint32_t id){ return (id>>5)&0x3F; }
static inline uint8_t id_cmd(uint32_t id){ return id&0x1F; }
static inline int node_index(uint8_t node){
  for(int i=0;i<NUM_NODES;++i) if(NODE_IDS[i]==node) return i;
  return -1;
}

/* 0x0A 를 RTR(remote request)로 요청 → ODrive 가 데이터 프레임으로 응답.
 * encoder_count_rate_ms=0(기본) 이어도 RTR 이면 on-demand 응답 → config 변경 불필요. */
void request_count(uint8_t node){
  struct can_frame f;
  f.can_id = (((uint32_t)node<<5) | CMD_ENCODER_COUNT) | CAN_RTR_FLAG;
  f.can_dlc = 8;
  for(int i=0;i<8;++i) f.data[i]=0;
  mcp2515.sendMessage(&f);
}

void drainRx(){
  struct can_frame f;
  while(mcp2515.readMessage(&f)==MCP2515::ERROR_OK){
    uint32_t id=f.can_id&CAN_SFF_MASK;
    int i=node_index(id_node(id));
    if(i<0) continue;
    uint8_t cmd=id_cmd(id);
    if(cmd==CMD_HEARTBEAT && f.can_dlc>=5){
      st[i].cnt_hb++; memcpy(&st[i].axis_error,&f.data[0],4); st[i].axis_state=f.data[4]; st[i].got_hb=true;
    } else if(cmd==CMD_ENCODER_ESTIMATES && f.can_dlc>=8){
      st[i].cnt09++; memcpy(&st[i].pos09,&f.data[0],4); memcpy(&st[i].vel09,&f.data[4],4); st[i].got09=true;
    } else if(cmd==CMD_ENCODER_COUNT && f.can_dlc>=8){
      st[i].cnt0A++; memcpy(&st[i].shadow,&f.data[0],4); memcpy(&st[i].cic,&f.data[4],4); st[i].got0A=true;
    }
  }
}

void setup(){
  Serial.begin(115200);
  for(int i=0;i<30 && !Serial;++i) delay(100);
  delay(200);
  Serial.println();
  Serial.println("[enc-test] arm 없이 CAN 0x0A idle-read 진단");
  Serial.println("  ODrive IDLE 유지(arm 명령 안 보냄). 손으로 모터를 천천히 돌려보세요.");
  Serial.println("  기대: 0x09 pos=0(죽음) / 0x0A shadow,cic=손 따라 변함(라이브) → 전류0 위치파악 OK");

  pinMode(PIN_MCP_RST, OUTPUT);
  digitalWrite(PIN_MCP_RST,HIGH); delay(100);
  digitalWrite(PIN_MCP_RST,LOW);  delay(100);
  digitalWrite(PIN_MCP_RST,HIGH); delay(100);
  SPI.begin(PIN_SPI_SCLK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_MCP_CS);
  int e1=mcp2515.reset(), e2=mcp2515.setBitrate(CAN_500KBPS,MCP_16MHZ), e3=mcp2515.setNormalMode();
  g_mcp_ok = (e1==MCP2515::ERROR_OK && e2==MCP2515::ERROR_OK && e3==MCP2515::ERROR_OK);
  Serial.print("MCP2515 "); Serial.println(g_mcp_ok?"OK":"실패");
}

void loop(){
  drainRx();
  uint32_t now=millis();

  static uint32_t lreq=0;
  if(now-lreq>=REQ_MS){
    lreq=now;
    for(int i=0;i<NUM_NODES;++i) request_count(NODE_IDS[i]);
  }

  static uint32_t lp=0;
  if(now-lp>=PRINT_MS){
    lp=now;
    for(int i=0;i<NUM_NODES;++i){
      if(!st[i].got_hb && !st[i].got09 && !st[i].got0A) continue;  // 응답 없는 노드 skip
      Serial.print("node"); Serial.print(NODE_IDS[i]);
      Serial.print(" st="); Serial.print(st[i].axis_state);
      Serial.print(" err=0x"); Serial.print(st[i].axis_error,HEX);
      Serial.print(" | 0x09 pos="); Serial.print(st[i].pos09,4);
      Serial.print(" (n"); Serial.print(st[i].cnt09); Serial.print(")");
      Serial.print(" | 0x0A shadow="); Serial.print(st[i].shadow);
      Serial.print(" cic="); Serial.print(st[i].cic);
      Serial.print(" (n"); Serial.print(st[i].cnt0A); Serial.print(")");
      // cic/cpr 로 환산한 mono-turn 위치 (cpr=16384)
      Serial.print(" turn~="); Serial.print(st[i].cic/16384.0f,4);
      Serial.println();
    }
  }
}
