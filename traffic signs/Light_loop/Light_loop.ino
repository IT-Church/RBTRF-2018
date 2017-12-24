#include<timer-api.h>
const int R = 2;
const int Y = 3;
const int G = 4;
char Signal=0x09;
void timer_handle_interrupts(int timer) {
  Serial.println(Signal, HEX);
}

void setup() {
Serial.begin(115200);
pinMode(R,OUTPUT);
pinMode(Y,OUTPUT);
pinMode(G,OUTPUT);
timer_init_ISR_10Hz(TIMER_DEFAULT);
}

void loop() {
  Signal = 0x08;
  digitalWrite(R,1);
  delay(500);
  digitalWrite(Y,1);
  Signal= 0x01;
  delay(500);
  Signal= 0x02;
  digitalWrite(R,0);
  digitalWrite(Y,0);
  digitalWrite(G,1);
  delay(500);
  Signal= 0x03;
  for(int i = 0; i < 3; i++){
    digitalWrite(G,0);
    delay(500);
    digitalWrite(G,1);
    delay(500);
  }
  Signal= 0x06;
  digitalWrite(G,0);
  digitalWrite(Y,1); 
  delay(500);
  digitalWrite(Y,0);
}
