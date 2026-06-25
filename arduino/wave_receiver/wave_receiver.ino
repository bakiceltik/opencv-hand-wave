#include <Servo.h>
#include <SoftwareSerial.h>

const int SERIAL_BAUDRATE = 9600;

// Wired link to the ESP32-CAM's hardware UART0 (GPIO1/TX -> ESP_RX_PIN, GPIO3/RX -> ESP_TX_PIN),
// so commands (WAVE, ANG:.., ...) arrive over this wire instead of USB from a PC.
// Baud must match Serial.begin() in camera_web_server.ino.
const int ESP_RX_PIN = 4;
const int ESP_TX_PIN = A2;
const int ESP_SERIAL_BAUDRATE = 9600;

const int LED_PIN = LED_BUILTIN;
const int OUTPUT_PIN = 7;

const int GOVDE_SERVO_PIN = 3;
const int DENGE_SERVO_PIN = 5;
const int SOL_OMUZ_PIN = 6;
const int ULTRASONIC_ECHO_PIN = 8;
const int SAG_OMUZ_PIN = 9;
const int SOL_KOL_PIN = 10;
const int SAG_KOL_PIN = 11;
const int ULTRASONIC_TRIG_PIN = 12;

const int JOY_X_PIN = A0;
const int JOY_SW_PIN = 2;

const int GOVDE_MIN_ANGLE = 20;
const int GOVDE_MAX_ANGLE = 160;
const int GOVDE_PUNCH_OFFSET = 40;
const int MESAFE_GARD_CM = 40;
const int MESAFE_YUMRUK_CM = 10;
const int MESAFE_GARD_HYSTERESIS_CM = 5;

const unsigned long ULTRASONIC_TIMEOUT_US = 25000;
const unsigned long ULTRASONIC_SAMPLE_INTERVAL_MS = 120;
const unsigned long STATUS_PRINT_INTERVAL_MS = 300;

Servo govdeServo;
Servo sagOmuz;
Servo sagKol;
Servo solOmuz;
Servo solKol;
Servo dengeServo;

SoftwareSerial espSerial(ESP_RX_PIN, ESP_TX_PIN);

struct LineReader {
  String buffer;
  bool discardUntilNewline = false;
};

LineReader usbReader;
LineReader espReader;

unsigned long ledUntil = 0;

int dengeAci = 140;

int govdeHazir = 90;
int currentGovdeAngle = 90;

int sagOmuzHazir = 50;
int sagKolHazir = 50;
int sagOmuzYumruk = 145;
int sagKolYumruk = 50;

int solOmuzHazir = 120;
int solKolHazir = 170;
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
unsigned long sonUltrasonicOkumaMs = 0;
unsigned long sonDurumYazdirmaMs = 0;
bool yumrukHazir = true;
bool sonrakiYumrukSol = true;

enum MesafeDurumu {
  MESAFE_BEKLE,
  MESAFE_GARD,
  MESAFE_YAKIN
};

MesafeDurumu mesafeDurumu = MESAFE_BEKLE;

const char *mesafeDurumuAdi(MesafeDurumu durum) {
  switch (durum) {
    case MESAFE_BEKLE:
      return "BEKLE";
    case MESAFE_GARD:
      return "GARD";
    case MESAFE_YAKIN:
      return "YAKIN";
    default:
      return "BILINMIYOR";
  }
}

void writeDistanceStatus(long mesafeCm, MesafeDurumu durum) {
  Serial.print("DIST_CM:");
  Serial.print(mesafeCm);
  Serial.print(" STATE:");
  Serial.println(mesafeDurumuAdi(durum));
}

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

void tumKollariAyir() {
  sagOmuz.detach();
  sagKol.detach();
  solOmuz.detach();
  solKol.detach();
}

void hazirPozisyonaDon() {
  setGovdeAngle(govdeHazir);

  sagOmuz.attach(SAG_OMUZ_PIN);
  sagKol.attach(SAG_KOL_PIN);
  solOmuz.attach(SOL_OMUZ_PIN);
  solKol.attach(SOL_KOL_PIN);

  sagHazirPozisyon();
  solHazirPozisyon();
  delay(400);
}

void gardPozisyonuAl() {
  const int sagKolGard = 130;
  const int solKolGard = 80;

  setGovdeAngle(govdeHazir);

  sagOmuz.attach(SAG_OMUZ_PIN);
  sagKol.attach(SAG_KOL_PIN);
  solOmuz.attach(SOL_OMUZ_PIN);
  solKol.attach(SOL_KOL_PIN);

  sagOmuz.write(sagOmuzHazir);
  sagKol.write(sagKolGard);
  solOmuz.write(solOmuzHazir);
  solKol.write(solKolGard);
  delay(250);

  tumKollariAyir();
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
  while (Serial.available()) Serial.read();
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
  while (Serial.available()) Serial.read();
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
  while (Serial.available()) Serial.read();
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
  while (Serial.available()) Serial.read();
}

void triggerWaveAction() {
  pulseWaveOutputs();
  elSalla();
}

long mesafeOlcCm() {
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  unsigned long sureUs = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT_US);
  if (sureUs == 0) {
    return -1;
  }

  return static_cast<long>(sureUs / 58);
}

void handleDistanceSensor() {
  if (millis() - sonUltrasonicOkumaMs < ULTRASONIC_SAMPLE_INTERVAL_MS) {
    return;
  }

  sonUltrasonicOkumaMs = millis();

  long mesafeCm = mesafeOlcCm();
  if (mesafeCm <= 0) {
    return;
  }

  MesafeDurumu hedefDurum = MESAFE_YAKIN;
  if (mesafeCm > MESAFE_GARD_CM) {
    hedefDurum = MESAFE_BEKLE;
  } else if (mesafeCm >= MESAFE_YUMRUK_CM) {
    hedefDurum = MESAFE_GARD;
  }

  if (millis() - sonDurumYazdirmaMs >= STATUS_PRINT_INTERVAL_MS) {
    sonDurumYazdirmaMs = millis();
    writeDistanceStatus(mesafeCm, hedefDurum);
  }

  bool bekleEsigiAsildi = (mesafeDurumu == MESAFE_BEKLE)
      ? (mesafeCm > MESAFE_GARD_CM - MESAFE_GARD_HYSTERESIS_CM)
      : (mesafeCm > MESAFE_GARD_CM + MESAFE_GARD_HYSTERESIS_CM);

  if (bekleEsigiAsildi) {
    yumrukHazir = true;
    if (mesafeDurumu != MESAFE_BEKLE) {
      hazirPozisyonaDon();
      mesafeDurumu = MESAFE_BEKLE;
      Serial.println("DIST:BEKLE");
    }
    return;
  }

  if (mesafeCm >= MESAFE_YUMRUK_CM) {
    yumrukHazir = true;
    if (mesafeDurumu != MESAFE_GARD) {
      gardPozisyonuAl();
      mesafeDurumu = MESAFE_GARD;
      Serial.println("DIST:GARD");
    }
    return;
  }

  if (yumrukHazir) {
    yumrukHazir = false;
    mesafeDurumu = MESAFE_YAKIN;
    if (sonrakiYumrukSol) {
      Serial.println("DIST:YUMRUK:SOL");
      solYumrukAt();
    } else {
      Serial.println("DIST:YUMRUK:SAG");
      sagYumrukAt();
    }
    sonrakiYumrukSol = !sonrakiYumrukSol;
    gardPozisyonuAl();
  }
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

// allowBareWShortcut is only for the USB link, where a human types a single 'W' + Enter
// in the Serial Monitor. The ESP32-CAM link only ever sends full, newline-terminated
// lines, and its own WiFi/camera boot logs ("WiFi connecting...") can start with 'W' --
// so that link must always go through the full-line match in handleLine() instead.
void readCommandsFrom(Stream &stream, LineReader &reader, bool allowBareWShortcut) {
  while (stream.available() > 0) {
    char ch = static_cast<char>(stream.read());

    if (reader.discardUntilNewline) {
      if (ch == '\n' || ch == '\r') {
        reader.discardUntilNewline = false;
      }
      continue;
    }

    if (allowBareWShortcut && reader.buffer.length() == 0 && (ch == 'W' || ch == 'w')) {
      Serial.println("RX:W");
      triggerWaveAction();
      reader.discardUntilNewline = true;
      continue;
    }

    if (ch == '\n' || ch == '\r') {
      if (reader.buffer.length() > 0) {
        handleLine(reader.buffer);
        reader.buffer = "";
      }
    } else if (reader.buffer.length() < 31) {
      reader.buffer += ch;
    }
  }
}

void readSerialCommands() {
  readCommandsFrom(Serial, usbReader, true);
  readCommandsFrom(espSerial, espReader, false);
}

void setup() {
  Serial.begin(SERIAL_BAUDRATE);
  espSerial.begin(ESP_SERIAL_BAUDRATE);

  pinMode(LED_PIN, OUTPUT);
  pinMode(OUTPUT_PIN, OUTPUT);
  pinMode(JOY_SW_PIN, INPUT_PULLUP);
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);

  digitalWrite(LED_PIN, LOW);
  digitalWrite(OUTPUT_PIN, LOW);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

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
    gelGelIsareti();
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

  handleDistanceSensor();

  delay(20);
}
