/*
 * imu_test.cpp  —  BNO085 데이터만 읽어 시리얼 출력하는 진단 모드
 * ============================================================================
 * 제어(CAN/모터)와 완전 분리된 독립 펌웨어. 메인 그네 펌웨어(main.cpp)는 안 건드림.
 * IMU 배선/주소/기울기/정점감지/tare 를 실배포 전에 단독 검증하는 용도.
 * (기존 probe_*.py / read_encoders.py 의 ESP32 IMU 판.)
 *
 * 빌드/플래시:
 *   platformio run -d esp32t2can -e imu-test -t upload --upload-port COM5
 * 모니터:
 *   esp32t2can/tools/read_serial.py COM5 <초>   (penv 파이썬)
 *
 * 하드웨어:
 *   - Adafruit BNO085 (9-DOF, SH-2 온보드 융합)
 *   - T-2CAN QWIIC(STEMMA) = I2C: SDA=GPIO1, SCL=GPIO2 (LilyGo 공식 핀맵 확인됨)
 *   - 기본 I2C 주소 0x4A
 *
 * 무엇을 읽나 (그네 응용 기준):
 *   - Game Rotation Vector (자이로+가속, 지자기 안 씀 → 모터 자석 간섭 회피).
 *     중력기준 tilt 만 필요하므로 magnetometer 불필요.
 *   - Calibrated Gyroscope (정지 판정 + 정점 zero-cross 감지).
 *
 * 동작:
 *   1) 부팅 → I2C/BNO085 init. 못 찾으면 그 사실만 출력하고 재시도(블로킹 안 함).
 *   2) 정지(자이로 작음) 가 STILL_MS 지속되면 자동 tare → 현재 방향이 0° 기준.
 *      "설치할 때마다 위치가 달라짐" 흡수 (모터 g_center 캡처와 같은 패턴).
 *   3) tilt(rest 대비 °), gyro[xyz], 정지여부 출력. 정점(apex)에서 마커 출력.
 *   4) 's'/'t' 시리얼 또는 BOOT 버튼 → 수동 재-tare.
 *
 * ⚠️ 소프트웨어 tare: 라이브러리 tare API 의존 대신, rest 쿼터니언 q0 를 저장해
 *    상대회전 conj(q0)⊗q 로 계산 (이식성/명확성).
 */
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

/* ---- 핀 / 주소 ---- */
static const uint8_t PIN_I2C_SDA = 1;   // T-2CAN QWIIC
static const uint8_t PIN_I2C_SCL = 2;
static const uint8_t PIN_BOOT    = 0;   // 수동 재-tare
static const uint8_t BNO_ADDR    = 0x4A;

/* ---- 튜닝 상수 ---- */
static const uint16_t PRINT_HZ      = 20;     // 시리얼 출력율
static const uint32_t REINIT_MS     = 1000;   // BNO 못 찾을 때 재시도 간격
static const float    STILL_GYRO    = 0.05f;  // rad/s — 이하면 "정지"
static const uint32_t STILL_MS      = 800;    // 이만큼 정지 지속 → 정지 확정/자동tare
static const float    APEX_GYRO     = 0.10f;  // rad/s — gyro_mag 가 이 아래로 내려오며
static const float    APEX_MIN_TILT = 3.0f;   //  tilt 가 이 이상이면 정점(apex) 으로 판정

static const float    RAD2DEG = 57.2957795f;

Adafruit_BNO08x bno08x(-1);     // reset 핀 미사용
sh2_SensorValue_t sv;

bool  g_imu_ok = false;
uint32_t g_last_init = 0;

// 최신 쿼터니언(game rot vec) + 자이로
float qw=1, qi=0, qj=0, qk=0;
float gx=0, gy=0, gz=0;
bool  got_q=false, got_g=false;

// rest(0점) 기준 쿼터니언
float q0w=1, q0i=0, q0j=0, q0k=0;
bool  rest_set=false;

// 정지/정점 상태
uint32_t still_since=0;
bool  is_still=false;
bool  prev_below_apex=true;

/* ---- BNO085 리포트 활성화 ---- */
bool enableReports(){
  bool ok = true;
  // 100Hz (10ms) — 출력은 더 낮게 throttle
  ok &= bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 10000);
  ok &= bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000);
  return ok;
}

bool initBNO(){
  if(!bno08x.begin_I2C(BNO_ADDR, &Wire)){
    return false;
  }
  if(!enableReports()){
    Serial.println("[경고] BNO085 리포트 활성화 실패");
  }
  return true;
}

/* ---- 현재 방향을 0점(rest)으로 캡처 ---- */
void tare(const char* why){
  if(!got_q){ Serial.println("[tare] 아직 쿼터니언 수신 전 — 무시"); return; }
  q0w=qw; q0i=qi; q0j=qj; q0k=qk;
  rest_set=true;
  Serial.print(">>> TARE (0점 설정) — "); Serial.println(why);
}

/* ---- rest 대비 상대 tilt 각 [deg] : angle of conj(q0)⊗q ---- */
float tiltFromRest(){
  if(!rest_set || !got_q) return 0.0f;
  // conj(q0) = (q0w, -q0i, -q0j, -q0k)
  float aw=q0w, ax=-q0i, ay=-q0j, az=-q0k;
  float bw=qw,  bx=qi,   by=qj,   bz=qk;
  float rw = aw*bw - ax*bx - ay*by - az*bz;   // 상대회전 실수부만 필요
  float w = fabsf(rw); if(w>1.0f) w=1.0f;
  return 2.0f*acosf(w)*RAD2DEG;
}

void setup(){
  Serial.begin(115200);
  for(int i=0;i<30 && !Serial;++i) delay(100);
  delay(200);
  Serial.println();
  Serial.println("[imu_test] BNO085 진단 모드 (제어/모터 없음)");
  Serial.print("I2C SDA=GPIO"); Serial.print(PIN_I2C_SDA);
  Serial.print(" SCL=GPIO"); Serial.print(PIN_I2C_SCL);
  Serial.print(" addr=0x"); Serial.println(BNO_ADDR, HEX);

  pinMode(PIN_BOOT, INPUT_PULLUP);
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);

  if(initBNO()){
    g_imu_ok=true;
    Serial.println(">>> BNO085 OK. 정지 상태로 두면 자동 0점(tare) 잡힘.");
    Serial.println("    수동 재-tare: 시리얼 't' 또는 BOOT 버튼.");
  } else {
    Serial.println("[오류] BNO085 못 찾음 (배선/주소 확인). 1초마다 재시도.");
  }
  g_last_init = millis();
}

void loop(){
  uint32_t now = millis();

  // ---- 미연결 시 재시도 (블로킹 안 함) ----
  if(!g_imu_ok){
    if(now - g_last_init >= REINIT_MS){
      g_last_init = now;
      if(initBNO()){ g_imu_ok=true; Serial.println(">>> BNO085 연결됨."); }
      else Serial.println("... BNO085 대기중 (배선 확인)");
    }
    return;
  }

  // ---- 리셋 감지 시 리포트 재활성화 ----
  if(bno08x.wasReset()){
    Serial.println("[info] BNO085 리셋 감지 → 리포트 재활성화");
    enableReports();
    rest_set=false;   // 리셋되면 0점 다시 잡아야 함
  }

  // ---- 센서 이벤트 흡수 ----
  while(bno08x.getSensorEvent(&sv)){
    switch(sv.sensorId){
      case SH2_GAME_ROTATION_VECTOR:
        qw=sv.un.gameRotationVector.real; qi=sv.un.gameRotationVector.i;
        qj=sv.un.gameRotationVector.j;    qk=sv.un.gameRotationVector.k;
        got_q=true; break;
      case SH2_GYROSCOPE_CALIBRATED:
        gx=sv.un.gyroscope.x; gy=sv.un.gyroscope.y; gz=sv.un.gyroscope.z;
        got_g=true; break;
      default: break;
    }
  }

  // ---- 정지 판정 ----
  float gmag = sqrtf(gx*gx + gy*gy + gz*gz);
  if(gmag < STILL_GYRO){
    if(still_since==0) still_since=now;
    if(!is_still && (now - still_since) >= STILL_MS){
      is_still=true;
      if(!rest_set) tare("자동(정지 확정)");   // 부팅 후 처음 정지 → 자동 0점
    }
  } else {
    still_since=0; is_still=false;
  }

  // ---- 정점(apex) 감지: 움직이다가 gyro_mag 가 APEX_GYRO 아래로 떨어질 때 ----
  float tilt = tiltFromRest();
  bool below = (gmag < APEX_GYRO);
  if(below && !prev_below_apex && tilt > APEX_MIN_TILT){
    Serial.print("    ▲ APEX (정점) tilt="); Serial.print(tilt,1); Serial.println("°");
  }
  prev_below_apex = below;

  // ---- 수동 재-tare (시리얼 / BOOT) ----
  if(Serial.available()){
    int c=Serial.read();
    if(c=='t'||c=='T'||c=='s'||c=='S') tare("시리얼 명령");
  }
  static int lb=HIGH; static uint32_t lbm=0;
  int b=digitalRead(PIN_BOOT);
  if(b==LOW && lb==HIGH && (now-lbm)>300){ lbm=now; tare("BOOT 버튼"); }
  lb=b;

  // ---- 출력 (throttle) ----
  static uint32_t lp=0;
  if(now-lp >= (uint32_t)(1000/PRINT_HZ)){
    lp=now;
    Serial.print("q=[");
    Serial.print(qw,3); Serial.print(","); Serial.print(qi,3); Serial.print(",");
    Serial.print(qj,3); Serial.print(","); Serial.print(qk,3); Serial.print("] ");
    Serial.print("tilt="); Serial.print(rest_set?tilt:0.0f,1); Serial.print("° ");
    Serial.print("gyro=["); Serial.print(gx,2); Serial.print(",");
    Serial.print(gy,2); Serial.print(","); Serial.print(gz,2); Serial.print("] ");
    Serial.print("|g|="); Serial.print(gmag,2);
    Serial.print(is_still?" STILL":"      ");
    Serial.print(rest_set?" rest=set":" rest=---");
    Serial.println();
  }
}
