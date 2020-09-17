#define PIN_LED 7 //20203129 전경진
unsigned int count, toggle;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200); //Initialize serial port
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  count = toggle = 0;
  digitalWrite(PIN_LED, toggle); // turn on LED.
}

void loop() {
  ++count;
  toggle = toggle_state(toggle); // toggle LED value.
  digitalWrite(PIN_LED, toggle); // update LED status.
  delay(100); // wait for 1,000 milliseconds
}

int toggle_state(int toggle) {
  switch(toggle){
    case 0:
      if(count<10){
        toggle=0;
       }
      else if(count>=10 && count <=20 || count>=20){
        toggle=1;
      }
      break;
    case 1:
      toggle=0;
      if(count>=20){
        while(1){}
      }
      break;
  }
  return toggle;
}
