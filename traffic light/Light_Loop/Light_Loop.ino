const int R = 2;
const int Y = 3;
const int G = 4;
void setup() {
Serial.begin(115200);
pinMode(R,OUTPUT);
pinMode(Y,OUTPUT);
pinMode(G,OUTPUT);
}

void loop() {
  Serial.print(0x00);
  digitalWrite(G,1);
}
