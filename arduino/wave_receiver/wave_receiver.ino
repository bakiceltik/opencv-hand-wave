#include <Servo.h>

const int SERIAL_BAUDRATE = 9600;

const int LED_PIN = LED_BUILTIN;
const int OUTPUT_PIN = 7;

const int GOVDE_SERVO_PIN = 3;
const int DENGE_SERVO_PIN = 5;
const int SOL_OMUZ_PIN = 6;
const int SAG_OMUZ_PIN = 9;
const int SOL_KOL_PIN = 10;
const int SAG_KOL_PIN = 11;

const int JOY_X_PIN = A0;
const int JOY_SW_PIN = 2;

const int GOVDE_MIN_ANGLE = 20;
const int GOVDE_MAX_ANGLE = 160;
const int GOVDE_PUNCH_OFFSET = 40;

Servo govdeServo;
Servo sagOmuz;
Servo sagKol;
Servo solOmuz;
Servo solKol;
Servo dengeServo;

String incomingLine;
unsigned long ledUntil = 0;
bool discardUntilNewline = false;

int dengeAci = 140;

int govdeHazir = 90;
int currentGovdeAngle = 90;

int sagOmuzHazir = 50;
int sagKolHazir = 140;
int sagOmuzYumruk = 145;
int sagKolYumruk = 50;

int solOmuzHazir = 120;
int solKolHazir = 80;
int solOmuzYumruk = 30;
int solKolYumruk = 170;

int gelGelOmuz = 145;
int gelGelKolOrta = 140;
int gelGelKolYukari = 130;
int gelGelKolAsagi = 150;

int solEsik = 300;
int sagEsik = 700;

bool joystickMerkezde = true;
bool oncekiJoystickButon = HIGH;

void pulseWaveOutputs() {
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(OUTPUT_PIN, HIGH);
  ledUntil = millis() + 1000;
}

void updateFeedbackOutputs() {
  if (ledUntil != 0 && millis() > ledUntil) {
    digitalWrite(LED_PIN, LOW);
    digitalWrite(OUTPUT_PIN, LOW);
    ledUntil = 0;
  }
}

void setGovdeAngle(int angle) {
  currentGovdeAngle = constrain(angle, GOVDE_MIN_ANGLE, GOVDE_MAX_ANGLE);
  govdeServo.write(currentGovdeAngle);
}

void sagHazirPozisyon() {
  sagOmuz.write(sagOmuzHazir);
  sagKol.write(sagKolHazir);
}

void solHazirPozisyon() {
  solOmuz.write(solOmuzHazir);
  solKol.write(solKolHazir);
}

void sagYumrukAt() {
  int govdeDinlenme = currentGovdeAngle;
  int govdeVurusAci = constrain(govdeDinlenme + GOVDE_PUNCH_OFFSET, GOVDE_MIN_ANGLE, GOVDE_MAX_ANGLE);

  sagOmuz.attach(SAG_OMUZ_PIN);
  sagKol.attach(SAG_KOL_PIN);

  setGovdeAngle(govdeDinlenme);
  sagOmuz.write(sagOmuzHazir);
  sagKol.write(sagKolHazir);
  delay(150);

  setGovdeAngle(govdeVurusAci);
  sagOmuz.write(sagOmuzYumruk);
  sagKol.write(sagKolYumruk);
  delay(450);

  for (int i = 0; i <= 30; i++) {
    int govdeAci = map(i, 0, 30, govdeVurusAci, govdeDinlenme);
    int omuzAci = map(i, 0, 30, sagOmuzYumruk, sagOmuzHazir);
    int kolAci = map(i, 0, 30, sagKolYumruk, sagKolHazir);

    setGovdeAngle(govdeAci);
    sagOmuz.write(omuzAci);
    sagKol.write(kolAci);
    delay(20);
  }

  delay(200);

  sagOmuz.detach();
  sagKol.detach();
}

void solYumrukAt() {
  int govdeDinlenme = currentGovdeAngle;
  int govdeVurusAci = constrain(govdeDinlenme - GOVDE_PUNCH_OFFSET, GOVDE_MIN_ANGLE, GOVDE_MAX_ANGLE);

  solOmuz.attach(SOL_OMUZ_PIN);
  solKol.attach(SOL_KOL_PIN);

  setGovdeAngle(govdeDinlenme);
  solOmuz.write(solOmuzHazir);
  solKol.write(solKolHazir);
  delay(150);

  setGovdeAngle(govdeVurusAci);
  solOmuz.write(solOmuzYumruk);
  solKol.write(solKolYumruk);
  delay(450);

  for (int i = 0; i <= 30; i++) {
    int govdeAci = map(i, 0, 30, govdeVurusAci, govdeDinlenme);
    int omuzAci = map(i, 0, 30, solOmuzYumruk, solOmuzHazir);
    int kolAci = map(i, 0, 30, solKolYumruk, solKolHazir);

    setGovdeAngle(govdeAci);
    solOmuz.write(omuzAci);
    solKol.write(kolAci);
    delay(20);
  }

  delay(200);

  solOmuz.detach();
  solKol.detach();
}

void gelGelIsareti() {
  int govdeDinlenme = currentGovdeAngle;
  int govdeIsaretAci = constrain(govdeDinlenme + GOVDE_PUNCH_OFFSET, GOVDE_MIN_ANGLE, GOVDE_MAX_ANGLE);

  sagOmuz.attach(SAG_OMUZ_PIN);
  sagKol.attach(SAG_KOL_PIN);

  setGovdeAngle(govdeIsaretAci);
  sagOmuz.write(gelGelOmuz);
  sagKol.write(gelGelKolOrta);
  delay(250);

  for (int tekrar = 0; tekrar < 3; tekrar++) {
    sagKol.write(gelGelKolYukari);
    delay(180);
    sagKol.write(gelGelKolAsagi);
    delay(180);
  }

  sagKol.write(gelGelKolOrta);
  delay(200);

  for (int i = 0; i <= 30; i++) {
    int govdeAci = map(i, 0, 30, govdeIsaretAci, govdeDinlenme);
    int omuzAci = map(i, 0, 30, gelGelOmuz, sagOmuzHazir);
    int kolAci = map(i, 0, 30, gelGelKolOrta, sagKolHazir);

    setGovdeAngle(govdeAci);
    sagOmuz.write(omuzAci);
    sagKol.write(kolAci);
    delay(20);
  }

  delay(200);

  sagOmuz.detach();
  sagKol.detach();
}

void elSalla() {
  sagOmuz.attach(SAG_OMUZ_PIN);
  sagKol.attach(SAG_KOL_PIN);
  solOmuz.attach(SOL_OMUZ_PIN);
  solKol.attach(SOL_KOL_PIN);

  int sagOmuzYukari = 180;
  int solOmuzYukari = 0;
  int sagKolDuz = 60;
  int solKolDuz = 160;
  int sagKolYukari = 50;
  int sagKolAsagi = 70;
  int solKolYukari = 170;
  int solKolAsagi = 150;

  sagOmuz.write(sagOmuzYukari);
  solOmuz.write(solOmuzYukari);
  sagKol.write(sagKolDuz);
  solKol.write(solKolDuz);
  delay(500);

  for (int tekrar = 0; tekrar < 4; tekrar++) {
    sagKol.write(sagKolYukari);
    solKol.write(solKolYukari);
    delay(180);

    sagKol.write(sagKolAsagi);
    solKol.write(solKolAsagi);
    delay(180);
  }

  sagKol.write(sagKolDuz);
  solKol.write(solKolDuz);
  delay(250);

  for (int i = 0; i <= 30; i++) {
    int sagOmuzAci = map(i, 0, 30, sagOmuzYukari, sagOmuzHazir);
    int sagKolAci = map(i, 0, 30, sagKolDuz, sagKolHazir);
    int solOmuzAci = map(i, 0, 30, solOmuzYukari, solOmuzHazir);
    int solKolAci = map(i, 0, 30, solKolDuz, solKolHazir);

    sagOmuz.write(sagOmuzAci);
    sagKol.write(sagKolAci);
    solOmuz.write(solOmuzAci);
    solKol.write(solKolAci);
    delay(20);
  }

  delay(200);

  sagOmuz.detach();
  sagKol.detach();
  solOmuz.detach();
  solKol.detach();
}

void triggerWaveAction() {
  pulseWaveOutputs();
  elSalla();
}

void handleLine(const String &line) {
  Serial.print("RX:");
  Serial.println(line);

  if (line.equalsIgnoreCase("WAVE") || line.equalsIgnoreCase("W")) {
    triggerWaveAction();
    return;
  }

  if (line.equalsIgnoreCase("COME")) {
    gelGelIsareti();
    return;
  }

  if (line.equalsIgnoreCase("LEFT_PUNCH")) {
    solYumrukAt();
    return;
  }

  if (line.equalsIgnoreCase("RIGHT_PUNCH")) {
    sagYumrukAt();
    return;
  }

  if (line.startsWith("ANG:")) {
    int angle = line.substring(4).toInt();
    setGovdeAngle(angle);
  }
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    char ch = static_cast<char>(Serial.read());

    if (discardUntilNewline) {
      if (ch == '\n' || ch == '\r') {
        discardUntilNewline = false;
      }
      continue;
    }

    if (incomingLine.length() == 0 && (ch == 'W' || ch == 'w')) {
      Serial.println("RX:W");
      triggerWaveAction();
      discardUntilNewline = true;
      continue;
    }

    if (ch == '\n' || ch == '\r') {
      if (incomingLine.length() > 0) {
        handleLine(incomingLine);
        incomingLine = "";
      }
    } else if (incomingLine.length() < 31) {
      incomingLine += ch;
    }
  }
}

void setup() {
  Serial.begin(SERIAL_BAUDRATE);

  pinMode(LED_PIN, OUTPUT);
  pinMode(OUTPUT_PIN, OUTPUT);
  pinMode(JOY_SW_PIN, INPUT_PULLUP);

  digitalWrite(LED_PIN, LOW);
  digitalWrite(OUTPUT_PIN, LOW);

  dengeServo.attach(DENGE_SERVO_PIN);
  dengeServo.write(dengeAci);
  delay(300);

  govdeServo.attach(GOVDE_SERVO_PIN);
  setGovdeAngle(govdeHazir);
  delay(300);

  sagOmuz.attach(SAG_OMUZ_PIN);
  sagKol.attach(SAG_KOL_PIN);
  sagHazirPozisyon();
  delay(500);
  sagOmuz.detach();
  sagKol.detach();

  solOmuz.attach(SOL_OMUZ_PIN);
  solKol.attach(SOL_KOL_PIN);
  solHazirPozisyon();
  delay(500);
  solOmuz.detach();
  solKol.detach();

  Serial.println("READY");
}

void loop() {
  readSerialCommands();

  int joyX = analogRead(JOY_X_PIN);
  bool joystickButon = digitalRead(JOY_SW_PIN);

  dengeServo.write(dengeAci);
  updateFeedbackOutputs();

  if (oncekiJoystickButon == HIGH && joystickButon == LOW) {
    triggerWaveAction();
  }

  oncekiJoystickButon = joystickButon;

  if (joyX > 400 && joyX < 600) {
    joystickMerkezde = true;
  }

  if (joystickMerkezde && joyX < solEsik) {
    joystickMerkezde = false;
    solYumrukAt();
  }

  if (joystickMerkezde && joyX > sagEsik) {
    joystickMerkezde = false;
    sagYumrukAt();
  }

  delay(20);
}
