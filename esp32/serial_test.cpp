#define RX_PIN 16   // Adjust to your wiring
#define TX_PIN 17   // Adjust to your wiring

HardwareSerial MBotSerial(1);

void setup() {
  Serial.begin(115200);             
  MBotSerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);

  Serial.println("ESP32 Ready!");
}

void loop() {
  if (MBotSerial.available()) {
    int value = MBotSerial.parseInt(); // read number from mBot
    if (value > 0) {
      Serial.print("Got from mBot: ");
      Serial.println(value);

      int nextValue = value + 1;
      delay(500); // avoid flooding
      MBotSerial.println(nextValue);

      Serial.print("Sent to mBot: ");
      Serial.println(nextValue);
    }
  }
}
