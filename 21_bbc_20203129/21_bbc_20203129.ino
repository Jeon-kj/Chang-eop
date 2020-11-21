// Arduino pin assignment
#include <Servo.h>
#define PIN_SERVO 10
#define PIN_IR A0
#define _DIST_ALPHA 0.5 // EMA weight of new sample (range: 0 to 1). Setting this value to 1 effectively disables EMA filter.

Servo myservo;

// global variables
float dist_raw, dist_ema, dist_cali, alpha; // unit: mm
int a, b; // unit: mm


void setup() {
// initialize GPIO pins
  myservo.attach(PIN_SERVO); 
  
// initialize serial port
  Serial.begin(57600);

  a = 68.5;
  b = 308;
  alpha = _DIST_ALPHA;
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
  dist_raw = ir_distance();
  dist_cali = 100 + 300.0 / (b - a) * (dist_raw - a);
  dist_ema = alpha*dist_cali+(1-alpha)*dist_ema;

  if(dist_ema < 255) myservo.writeMicroseconds(1780);
  else if(dist_ema > 255) myservo.writeMicroseconds(1240);
}
