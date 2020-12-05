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

// Distance sensor
#define _DIST_ALPHA 0.5        // ema 필터의 측정 보정치

// Servo range
#define _DUTY_MIN 1100     
#define _DUTY_NEU 1560        
#define _DUTY_MAX 1900    

// Servo speed control
#define _SERVO_ANGLE 25       // servo 각도 설정
#define _SERVO_SPEED 30      // servo 속도 설정

// Event periods
#define _INTERVAL_DIST 50
#define _INTERVAL_SERVO 50
#define _INTERVAL_SERIAL 100

// PID parameters
#define _KP 1.0
#define _KD 20.0

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
#define FN 30
float dist_list[FN];
float dist_filtering, sum=0;
int i=0, cnt=0;

void setup() {
// initialize GPIO pins for LED and attach servo 
 pinMode(PIN_LED,OUTPUT);
 digitalWrite(PIN_LED, 1);
 myservo.attach(PIN_SERVO); 


// initialize global variables
  dist_min = _DIST_MIN;
  dist_max = _DIST_MAX;
  alpha = _DIST_ALPHA;
  dist_target = _DIST_TARGET;
  a = 68.5;
  b = 308;
  error_prev = 0;
  KP = _KP;
  KD = _KD;

// move servo to neutral position
 myservo.writeMicroseconds(_DUTY_NEU);
 
// initialize serial port
  Serial.begin(57600); // 시리얼 포트 초기화

// convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * ((float)_SERVO_SPEED / 180) * ((float)_INTERVAL_SERVO / 1000);
}
  
void loop() {
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
    
    ir_distance_filter();
    

  
  // PID control logic
    error_curr = dist_target - dist_ema;
    if(error_curr<0){
      KD = _KD+3;
      KP = _KP+0.5;
    }
    else{
      KD = _KD;
      KP = _KP;
    }
    pterm = KP * error_curr;
    dterm = KD * (error_curr - error_prev);
    control = pterm+dterm;

  // duty_target = f(duty_neutral, control)
    duty_target = _DUTY_NEU + control;

  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if (duty_target > _DUTY_MAX) duty_target = _DUTY_MAX;
    if (duty_target < _DUTY_MIN) duty_target = _DUTY_MIN; 

  // update error_prev
    error_prev = error_curr;
    
    last_sampling_time_dist = millis(); // 마지막 dist event 처리 시각 기록
  }
  
  if(event_servo) {
    event_servo = false;
    // adjust duty_curr toward duty_target by duty_chg_per_interval
    myservo.writeMicroseconds(duty_target);
    // update servo position
    duty_curr = duty_target;
    last_sampling_time_servo = millis(); // 마지막 servo event 처리 시각 기록
  }
  
  if(event_serial) {
    event_serial = false;
    Serial.print("dist_ir:");
    Serial.print(dist_ema);
    Serial.print(",pterm:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",dterm:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");
    last_sampling_time_serial = millis(); // 마지막 serial event 처리 시각 기록
  }
}


float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}



float ir_distance_filter() {

  for(i=0;i<FN;i++){
    dist_list[i] = 100 + 300.0 / (b - a) * (ir_distance() - a);
    sum+=dist_list[i];
  }
  float change;
  for(int idx=0;idx<FN;idx++){
    for(int jdx=i+1;jdx<FN;jdx++){
      if(dist_list[idx] > dist_list[jdx]){
        change = dist_list[idx];
        dist_list[idx] = dist_list[jdx];
        dist_list[jdx] = change;
      }
    }
  }
  //컷팅
  for (int idx = 0; idx < 10; idx++) {
    sum -= dist_list[idx];
  }
  for (int idx = 1; idx <= 10; idx++) {
    sum -= dist_list[FN-idx];
  }

  dist_filtering = sum/(FN-20);
  sum=0;
  i=0;
  dist_ema = alpha*dist_filtering + (1-alpha)*dist_ema;
}
