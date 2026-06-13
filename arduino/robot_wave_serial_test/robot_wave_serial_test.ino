#include <Servo.h>

const int SERIAL_BAUDRATE = 9600;

const int SAG_OMUZ_PIN = 9;
const int SAG_KOL_PIN = 11;
const int SOL_OMUZ_PIN = 6;
const int SOL_KOL_PIN = 10;
const int JOY_SW_PIN = 2;
const int LED_PIN = LED_BUILTIN;

Servo sagOmuz;
Servo sagKol;
Servo solOmuz;
Servo solKol;

bool oncekiJoystickButon = HIGH;

int sagOmuzHazir = 50;
int sagKolHazir = 140;
int solOmuzHazir = 120;
int solKolHazir = 80;

void hazirPozisyon() {
  sagOmuz.write(sagOmuzHazir);
  sagKol.write(sagKolHazir);
  solOmuz.write(solOmuzHazir);
  solKol.write(solKolHazir);
}

void elSalla() {
  Serial.println("ACTION:WAVE");
  digitalWrite(LED_PIN, HIGH);

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

  digitalWrite(LED_PIN, LOW);
  Serial.println("ACTION:DONE");
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(JOY_SW_PIN, INPUT_PULLUP);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(SERIAL_BAUDRATE);
  delay(1000);
  Serial.println("READY");
  Serial.println("SEND: W");

  sagOmuz.attach(SAG_OMUZ_PIN);
  sagKol.attach(SAG_KOL_PIN);
  solOmuz.attach(SOL_OMUZ_PIN);
  solKol.attach(SOL_KOL_PIN);
  hazirPozisyon();
  delay(500);
  sagOmuz.detach();
  sagKol.detach();
  solOmuz.detach();
  solKol.detach();
}

void loop() {
  bool joystickButon = digitalRead(JOY_SW_PIN);

  if (oncekiJoystickButon == HIGH && joystickButon == LOW) {
    Serial.println("BTN");
    elSalla();
  }
  oncekiJoystickButon = joystickButon;

  while (Serial.available() > 0) {
    char ch = static_cast<char>(Serial.read());
    Serial.print("RX:");
    Serial.println(ch);
    if (ch == 'W' || ch == 'w') {
      elSalla();
    }
  }

  delay(20);
}
