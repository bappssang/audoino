#include <Servo.h>

// Arduino pin assignment
#define PIN_LED   9   // LED active-low
#define PIN_TRIG  12  // sonar sensor TRIGGER
#define PIN_ECHO  13  // sonar sensor ECHO
#define PIN_SERVO 10  // servo motor

// configurable parameters for sonar
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25      // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 180.0   // minimum distance to be measured (unit: mm)
#define _DIST_MAX 360.0   // maximum distance to be measured (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL) // coefficient to convert duration to distance

#define _EMA_ALPHA 0.5    // EMA weight of new sample (range: 0 to 1)

// Target Distance
#define _TARGET_LOW  250.0
#define _TARGET_HIGH 290.0

// duty duration for myservo.writeMicroseconds()
// NEEDS TUNING (servo by servo)
#define _DUTY_MIN 1000 // servo full clockwise position (0 degree)
#define _DUTY_NEU 1500 // servo neutral position (90 degree)
#define _DUTY_MAX 2000 // servo full counterclockwise position (180 degree)

// global variables
float dist_ema, dist_prev = _DIST_MAX; // unit: mm
unsigned long last_sampling_time;      // unit: ms

Servo myservo;

void setup() {
  // initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);    // sonar TRIGGER
  pinMode(PIN_ECHO, INPUT);     // sonar ECHO
  digitalWrite(PIN_TRIG, LOW);  // turn-off Sonar 

  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_NEU);

  // initialize USS related variables
  dist_prev = _DIST_MIN; // raw distance output from USS (unit: mm)

  // initialize serial port
  Serial.begin(57600);
}

void loop() {
  float dist_raw;
  
  // wait until next sampling time. 
  if (millis() < (last_sampling_time + INTERVAL))
    return;

  dist_raw = USS_measure(PIN_TRIG, PIN_ECHO); // read distance

  // 거리 측정 값에 따른 처리
  if ((dist_raw == 0.0) || (dist_raw > _DIST_MAX)) {
    dist_raw = dist_prev;           // 범위 초과 시 이전 값 유지
    digitalWrite(PIN_LED, HIGH);    // LED OFF
  } else if (dist_raw < _DIST_MIN) {
    dist_raw = dist_prev;           // 범위 이하 시 이전 값 유지
    digitalWrite(PIN_LED, HIGH);    // LED OFF
  } else {    // 유효 범위 내 거리일 때
    dist_prev = dist_raw;
    digitalWrite(PIN_LED, LOW);     // LED ON
  }

  // 필터 적용 (EMA)
  dist_ema = _EMA_ALPHA * dist_raw + (1 - _EMA_ALPHA) * dist_prev;

  // 서보 모터 제어 (거리에 따른 각도 계산)
  int servo_position = map(dist_ema, _DIST_MIN, _DIST_MAX, _DUTY_MIN, _DUTY_MAX);
  myservo.writeMicroseconds(servo_position);

  // 거리 값과 서보 모터 위치 출력
  Serial.print("Distance (raw): ");
  Serial.print(dist_raw);
  Serial.print(" mm, Filtered Distance: ");
  Serial.print(dist_ema);
  Serial.print(" mm, Servo Position: ");
  Serial.println(servo_position);

  // 샘플링 시간 업데이트
  last_sampling_time += INTERVAL;
}

// 초음파 센서를 통해 거리 측정 (단위: mm)
float USS_measure(int TRIG, int ECHO) {
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  
  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // 거리 계산 (단위: mm)
}
