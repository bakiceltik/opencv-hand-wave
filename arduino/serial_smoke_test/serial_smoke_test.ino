#include <Arduino.h>

const int LED_PIN = LED_BUILTIN;
const unsigned long BLINK_INTERVAL_MS = 250;

unsigned long lastBlinkAt = 0;
bool ledOn = false;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);
  delay(1000);

  Serial.println("READY");
  Serial.println("TYPE: W");
}

void loop() {
  if (millis() - lastBlinkAt >= BLINK_INTERVAL_MS) {
    lastBlinkAt = millis();
    ledOn = !ledOn;
    digitalWrite(LED_PIN, ledOn ? HIGH : LOW);
  }

  while (Serial.available() > 0) {
    char ch = static_cast<char>(Serial.read());
    Serial.print("RX:");
    Serial.println(ch);

    if (ch == 'W' || ch == 'w') {
      Serial.println("WAVE_OK");
      digitalWrite(LED_PIN, HIGH);
      delay(1000);
      digitalWrite(LED_PIN, LOW);
    }
  }
}
