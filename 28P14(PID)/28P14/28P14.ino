#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9  // LED 9번핀에 연결
#define PIN_SERVO 10 // 서보10핀에 연결
#define PIN_IR A0  // 적외선 센서 signal -> A0핀

// Framework setting
#define _DIST_TARGET 255    // 목표 위치가 25.5cm임을 선언
#define _DIST_MIN 100
#define _DIST_MAX 410

// Servo range
#define _DUTY_MIN 1100     
#define _DUTY_NEU 1450        
#define _DUTY_MAX 1900    

// Servo speed control
#define _SERVO_ANGLE 25       // servo 각도 설정
#define _SERVO_SPEED 1000      // servo 속도 설정

// Event periods
#define _INTERVAL_DIST 50
#define _INTERVAL_SERVO 50
#define _INTERVAL_SERIAL 100

// PID parameters
#define _KP 1.8 // 1.8 이하  올리면 진동폭이 커졌음
#define _KD 33.0  // 33.0
#define _KI 0.01  // 0.01
// Servo instance
Servo myservo; 

//////////////////////
// global variables //
//////////////////////
float dist_min, dist_max;
float a, b;
float alpha;



// Distance sensor
float dist_target; // location to send the ball
float dist_ema, dist_cali, dist_raw;

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial;

// Servo speed control
int duty_chg_per_interval; 
int duty_target, duty_curr;

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm, KP, KD;

//filter
#define DELAY_MICROS  1500 // 필터에 넣을 샘플값을 측정하는 딜레이(고정값!)
#define EMA_ALPHA 0.35     // EMA 필터 값을 결정하는 ALPHA 값. 작성자가 생각하는 최적값임.
float ema_dist=0;            // EMA 필터에 사용할 변수
float filtered_dist;       // 최종 측정된 거리값을 넣을 변수. loop()안에 filtered_dist = filtered_ir_distance(); 형태로 사용하면 됨.
float samples_num = 3;     // 스파이크 제거를 위한 부분필터에 샘플을 몇개 측정할 것인지. 3개로 충분함! 가능하면 수정하지 말 것.

//Ramp UP
int duty_chg_max;
int duty_chg_adjust;
#define _RAMPUP_TIME 360

void setup() {
// initialize GPIO pins for LED and attach servo 
 pinMode(PIN_LED,OUTPUT);
 digitalWrite(PIN_LED, 1);
 myservo.attach(PIN_SERVO); 


// initialize global variables
  dist_min = _DIST_MIN;
  dist_max = _DIST_MAX;
  dist_target = _DIST_TARGET;
  a = 68.5;
  b = 308;
  error_prev = 0;
  KP = _KP;
  KD = _KD;
  iterm = 0;
  duty_target = duty_curr = dist_min;
  myservo.writeMicroseconds(duty_curr);

// move servo to neutral position
 myservo.writeMicroseconds(_DUTY_NEU);
 
// initialize serial port
  Serial.begin(57600); // 시리얼 포트 초기화

// convert angle speed into duty change per interval.
  duty_chg_max = (float)(_DUTY_MAX - _DUTY_MIN) * _SERVO_SPEED / 180 * _INTERVAL_SERVO / 1000;
  duty_chg_adjust = (float) duty_chg_max * _INTERVAL_SERVO / _RAMPUP_TIME;
  duty_chg_per_interval = 0; // initial speed is set to 0.
}
  
void loop() {

  if(millis() < last_sampling_time_servo + _INTERVAL_SERVO) return;
  // adjust duty_curr toward duty_target
  if(duty_target > duty_curr) {
    if(duty_chg_per_interval < duty_chg_max) {
      duty_chg_per_interval += duty_chg_adjust;
      if(duty_chg_per_interval > duty_chg_max) duty_chg_per_interval = duty_chg_max;
    }
    duty_curr += duty_chg_per_interval;
    if(duty_curr > duty_target) duty_curr = duty_target;
  }
  else if(duty_target < duty_curr) {
    if(duty_chg_per_interval > -duty_chg_max) {
      duty_chg_per_interval -= duty_chg_adjust;
      if(duty_chg_per_interval < -duty_chg_max) duty_chg_per_interval = -duty_chg_max;
    }
    duty_curr += duty_chg_per_interval;
    if(duty_curr < duty_target) duty_curr = duty_target;
  }
  else {
    duty_chg_per_interval = 0;
  }

  myservo.writeMicroseconds(duty_curr);
  
/////////////////////
// Event generator // 이벤트 실행 간격 구현 
/////////////////////
  if (millis() >= last_sampling_time_dist + _INTERVAL_DIST) event_dist = true;
  if (millis() >= last_sampling_time_servo + _INTERVAL_SERVO) event_servo = true;
  if (millis() >= last_sampling_time_serial + _INTERVAL_SERIAL) event_serial = true;

////////////////////
// Event handlers //
////////////////////

  if(event_dist) {
    event_dist = false;
  // get a distance reading from the distance sensor
    
    dist_raw = filtered_ir_distance();
    

  
  // PID control logic
    error_curr = dist_target - dist_raw;
    if(error_curr<0){
      KD = _KD+8;
      KP = _KP+0.5;
    }
    else{
      KD = _KD;
      KP = _KP;
    }
    pterm = KP * error_curr;
    dterm = KD * (error_curr - error_prev);
    iterm += _KI * error_curr;
    control = pterm+dterm+iterm;

  // duty_target = f(duty_neutral, control)
    duty_target = _DUTY_NEU + control;

  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if (duty_target > _DUTY_MAX) duty_target = _DUTY_MAX;
    if (duty_target < _DUTY_MIN) duty_target = _DUTY_MIN; 

  // update error_prev
    error_prev = error_curr;
    
    last_sampling_time_dist = millis(); // 마지막 dist event 처리 시각 기록
  }

  /*
  if(event_servo) {
    event_servo = false;
    // adjust duty_curr toward duty_target by duty_chg_per_interval
    myservo.writeMicroseconds(duty_target);
    // update servo position
    duty_curr = duty_target;
    last_sampling_time_servo = millis(); // 마지막 servo event 처리 시각 기록
  }*/
  
  if(event_serial) {
    event_serial = false;
    Serial.print("IR:"); 
    Serial.print(dist_raw); 
    Serial.print(",T:"); 
    Serial.print(dist_target); 
    Serial.print(",P:"); 
    Serial.print(map(pterm,-1000,1000,510,610)); 
    Serial.print(",D:"); 
    Serial.print(map(dterm,-1000,1000,510,610)); 
    Serial.print(",I:"); 
    Serial.print(map(iterm,-1000,1000,510,610)); 
    Serial.print(",DTT:"); 
    Serial.print(map(duty_target,1000,2000,410,510)); 
    Serial.print(",DTC:"); 
    Serial.print(map(duty_curr,1000,2000,410,510)); 
    Serial.println(",-G:245,+G:265,m:0,M:800");
    last_sampling_time_serial = millis(); // 마지막 serial event 처리 시각 기록
  }
}


float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

float under_noise_filter(void){ // 아래로 떨어지는 형태의 스파이크를 제거해주는 필터
  int currReading;
  int largestReading = 0;
  
  for (int i = 0; i < samples_num; i++) {
    currReading = 100 + 300.0 / (b - a) * (ir_distance() - a);
    if (currReading > largestReading) { largestReading = currReading; }
    // Delay a short time before taking another reading
    delayMicroseconds(DELAY_MICROS);
  }
  return largestReading;
}

float filtered_ir_distance(void){ // 아래로 떨어지는 형태의 스파이크를 제거 후, 위로 치솟는 스파이크를 제거하고 EMA필터를 적용함.
  // under_noise_filter를 통과한 값을 upper_nosie_filter에 넣어 최종 값이 나옴.
  int currReading;
  int lowestReading = 1024;
  for (int i = 0; i < samples_num; i++) {
    currReading = under_noise_filter();
    if (currReading < lowestReading) { lowestReading = currReading; }
  }
  // eam 필터 추가
  ema_dist = EMA_ALPHA*lowestReading + (1-EMA_ALPHA)*ema_dist;
  return ema_dist;
}
