const int LED_PIN = LED_BUILTIN;
const int OUTPUT_PIN = 7;

String incomingLine;
unsigned long ledUntil = 0;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(OUTPUT_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(OUTPUT_PIN, LOW);

  Serial.begin(115200);
}

void loop() {
  while (Serial.available() > 0) {
    char ch = static_cast<char>(Serial.read());
    if (ch == '\n' || ch == '\r') {
      if (incomingLine == "WAVE") {
        digitalWrite(LED_PIN, HIGH);
        digitalWrite(OUTPUT_PIN, HIGH);
        ledUntil = millis() + 1000;
      }
      incomingLine = "";
    } else {
      incomingLine += ch;
    }
  }

  if (ledUntil != 0 && millis() > ledUntil) {
    digitalWrite(LED_PIN, LOW);
    digitalWrite(OUTPUT_PIN, LOW);
    ledUntil = 0;
  }
}
