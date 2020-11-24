
void setup() {
  // put your setup code here, to run once:
  pinMode(7,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  // 1000000us = 1sec delayMicroseconds(delay);
  // ms : 1000ms = 1sec
  // set_duty(100);
  set_period(10000); // 100 to 10000(unit:us)
}

void set_duty(int duty){ // 0 to 100 (unit: %)
  digitalWrite(7, 1);
  delayMicroseconds(duty);
  digitalWrite(7, 0);
  delayMicroseconds(100-duty);
}

void set_period(int period){ // 100 to 10000 (unit: us)
  double i;
  double x = 500000.0000/period;
  double cnt = period/x;
  if(period==10000){
    x = 1000000.0000/period;
    cnt = period/x;
  }
  for(i=0; i<period; i+=cnt){
    digitalWrite(7, 1);
    delayMicroseconds(i);
    digitalWrite(7, 0);
    delayMicroseconds(period-i);
  }
  for(i=period; i>0; i-=cnt){
    digitalWrite(7, 1);
    delayMicroseconds(i);
    digitalWrite(7, 0);
    delayMicroseconds(period-i);
  }
}
