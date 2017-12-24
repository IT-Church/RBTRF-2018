const int R = 2;
const int Y = 3;
const int G = 4;
long long qtime = 0;
char Signal=0x09;
void setup() {
Serial.begin(115200);
pinMode(R,OUTPUT);
pinMode(Y,OUTPUT);
pinMode(G,OUTPUT);

}

void loop() {
  qtime = millis();
  while(millis()-qtime < 10000){
    digitalWrite(Y,0);
    digitalWrite(R,1);
    Serial.write(0x00);
    delay(50);
  }
  qtime = millis();
  while(millis()-qtime < 2000){
    digitalWrite(Y,1);
    Serial.write(0x01);
    delay(50);
  }
  qtime = millis();
  while(millis()-qtime < 2000){
    digitalWrite(Y,0);
    digitalWrite(R,0);
    digitalWrite(G,1);
    Serial.write(0x02);
    delay(50);
  }
    qtime = millis();
  while(millis()-qtime < 500){
    digitalWrite(G,0);
    Serial.write(0x03);
    delay(50);
  }
    qtime = millis();
    while(millis()-qtime < 500){
    digitalWrite(G,1);
    Serial.write(0x03);
    delay(50);
  }
    qtime = millis();
    while(millis()-qtime < 500){
    digitalWrite(G,0);
    Serial.write(0x03);
    delay(50);
  }
    qtime = millis();
    while(millis()-qtime < 500){
    digitalWrite(G,1);
    Serial.write(0x03);
    delay(50);
  }
    qtime = millis();
    while(millis()-qtime < 500){
    digitalWrite(G,0);
    Serial.write(0x03);
    delay(50);
  }
    qtime = millis();
    while(millis()-qtime < 500){
    digitalWrite(G,1);
    Serial.write(0x03);
    delay(50);
  }
    qtime = millis();
    while(millis()-qtime < 2000){
    digitalWrite(Y,1);
    digitalWrite(G,0);
    Serial.write(0x0414);
    delay(50);
  }
}
