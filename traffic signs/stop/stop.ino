void setup() {
 Serial.begin(115200);

}

void loop() {
    Serial.write(0x06);
    delay(50);

}
