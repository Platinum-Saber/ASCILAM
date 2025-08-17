// mBot Ping-Pong Test
int counter = 1;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("mBot Ready!");

  // Start with ping 1
  Serial.println(counter);
}

void loop() {
  if (Serial.available()) {
    int value = Serial.parseInt(); // read number from ESP32
    if (value > 0) {
      counter = value + 1;  // increment counter
      delay(500);           // small delay to avoid flooding
      Serial.println(counter);
    }
  }
}
