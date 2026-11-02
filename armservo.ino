#include <ESP8266WiFi.h>
#include <Servo.h>

// Inisialisasi 4 Servo
Servo servoBase;     // Servo 1: Base (GPIO 5 / D1) - DIKUNCI DI 180°
Servo servoShoulder; // Servo 2: Shoulder (GPIO 4 / D2)
Servo servoElbow;    // Servo 3: Elbow (GPIO 14 / D5)
Servo servoGripper;  // Servo 4: Gripper (GPIO 12 / D6)

// Pin GPIO Servo
const int PIN_SERVO1 = 5;
const int PIN_SERVO2 = 4;
const int PIN_SERVO3 = 14;
const int PIN_SERVO4 = 12;

// ===== Sensor Infrared =====
const int  PIN_IR = 16;              // GPIO 16 (D0) -> OUT sensor IR
const bool IR_ACTIVE_LOW = true;     // true: LOW = ada benda (umum di modul FC-51)
const int  IR_CONFIRM_MS = 10;       // Sensor harus stabil mendeteksi selama ini (anti-noise)
const int  STOP_SETTLE_MS = 100;     // Tunggu benda diam setelah conveyor berhenti
const unsigned long COOLDOWN_MS = 0; // Abaikan sensor sebentar setelah conveyor jalan lagi

// ===== Motor Conveyor via DRV8833 =====
const int PIN_IN1 = 13; // GPIO 13 (D7) -> DRV8833 IN1 (PWM kecepatan)
const int PIN_IN2 = 15; // GPIO 15 (D8) -> DRV8833 IN2 (selalu LOW = putar satu arah)
const int CONVEYOR_SPEED = 195;     // 0 - 1023 (range PWM ESP8266)
const int CONVEYOR_PWM_FREQ = 1000; // Hz
const int CONVEYOR_PWM_MAX = 1023;  // Range PWM ESP8266 (0 - 1023)
const bool CONVEYOR_ON = true;      // Set false untuk menonaktifkan conveyor sepenuhnya (tes)

// Jalankan motor conveyor (speed 0 - 1023)
void conveyorRun(int speed) {
  speed = constrain(speed, 0, CONVEYOR_PWM_MAX);
  digitalWrite(PIN_IN2, LOW);
  analogWrite(PIN_IN1, speed);
  Serial.print("[CONVEYOR] RUN, speed = ");
  Serial.println(speed);
}

// Hentikan motor conveyor
void conveyorStop() {
  analogWrite(PIN_IN1, 0);
  digitalWrite(PIN_IN2, LOW);
  Serial.println("[CONVEYOR] STOP");
}

// Posisi Stand By Awal
const float FIXED_BASE  = 180.0;
float currentShoulder   = 150.0;
float currentElbow      = 120.0;
float currentGripper    = 40.0;

// Parameter kecepatan default
const float DEFAULT_STEP = 1.5; // Derajat per step
const int   DEFAULT_DLY  = 10;  // ms per step

// Parameter khusus gerakan berat (shoulder saat lengan terentang)
const float HEAVY_STEP = 0.8;
const int   HEAVY_DLY  = 15;

// Waktu servo menyusul posisi fisik setelah gerakan selesai
const int SETTLE_MS = 400;

// Waktu terakhir siklus arm selesai
unsigned long lastCycleEnd = 0;

// Fungsi Interpolasi Mulus 3 Servo, kecepatan bisa diatur per gerakan
void move3AxisSmooth(float targetShoulder, float targetElbow, float targetGripper,
                     float stepSpeed = DEFAULT_STEP, int stepDelay = DEFAULT_DLY) {
  bool inPosition = false;

  while (!inPosition) {
    inPosition = true;

    // Shoulder
    if (abs(targetShoulder - currentShoulder) > stepSpeed) {
      currentShoulder += (targetShoulder > currentShoulder) ? stepSpeed : -stepSpeed;
      inPosition = false;
    } else {
      currentShoulder = targetShoulder;
    }

    // Elbow
    if (abs(targetElbow - currentElbow) > stepSpeed) {
      currentElbow += (targetElbow > currentElbow) ? stepSpeed : -stepSpeed;
      inPosition = false;
    } else {
      currentElbow = targetElbow;
    }

    // Gripper
    if (abs(targetGripper - currentGripper) > stepSpeed) {
      currentGripper += (targetGripper > currentGripper) ? stepSpeed : -stepSpeed;
      inPosition = false;
    } else {
      currentGripper = targetGripper;
    }

    // Eksekusi ke Servo Fisik
    servoShoulder.write((int)currentShoulder);
    servoElbow.write(180 - (int)currentElbow); // Inversi orientasi servo Elbow
    servoGripper.write((int)currentGripper);

    delay(stepDelay);
    yield(); // Beri waktu ke watchdog ESP8266
  }

  // Beri waktu servo fisik menyusul posisi perintah
  delay(SETTLE_MS);
}

// Baca sensor IR mentah: true = ada benda
bool irRaw() {
  int v = digitalRead(PIN_IR);
  return IR_ACTIVE_LOW ? (v == LOW) : (v == HIGH);
}

// Deteksi benda dengan konfirmasi (anti-noise)
bool objectDetected() {
  if (!irRaw()) return false;
  unsigned long start = millis();
  while (millis() - start < IR_CONFIRM_MS) {
    if (!irRaw()) return false; // Hanya noise sesaat
    yield();
  }
  return true;
}

// Urutan kerja arm (mulai dan berakhir di posisi Stand By)
void runArmSequence() {
  // 2. Move (Shoulder: 100, Elbow: 55, Gripper: 40)
  Serial.println("[POSE] 2. Move (100, 55, 40)");
  move3AxisSmooth(100.0, 55.0, 40.0);
  delay(1000);

  // 3. Grip (Shoulder: 100, Elbow: 55, Gripper: 120)
  Serial.println("[POSE] 3. Grip (100, 55, 120)");
  move3AxisSmooth(100.0, 55.0, 120.0);
  delay(1000);

  // 4a. Move to Place - TAHAP 1: Siku bergerak dulu (Elbow: 55 -> 175)
  Serial.println("[POSE] 4a. Siku Dulu (110, 175, 120)");
  move3AxisSmooth(110.0, 175.0, 120.0);
  delay(1000); // Jeda agar elbow stabil sebelum shoulder bergerak

  // 4b. Move to Place - TAHAP 2: Shoulder menyusul (diperlambat)
  Serial.println("[POSE] 4b. Shoulder Menyusul (55, 175, 110)");
  move3AxisSmooth(55.0, 175.0, 110.0, HEAVY_STEP, HEAVY_DLY);
  delay(1000);

  // 5. Place (Shoulder: 55, Elbow: 175, Gripper: 40)
  Serial.println("[POSE] 5. Place (55, 175, 40)");
  move3AxisSmooth(55.0, 175.0, 40.0);
  delay(1000);

  // 6. Stand By - kembali ke posisi awal (diperlambat)
  Serial.println("[POSE] 6. Stand By (150, 120, 40)");
  move3AxisSmooth(150.0, 120.0, 40.0, HEAVY_STEP, HEAVY_DLY);
  delay(500);
}

void setup() {
  Serial.begin(115200);

  // Nonaktifkan Wi-Fi
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();

  // Konfigurasi Sensor IR
  pinMode(PIN_IR, INPUT);

  // Konfigurasi Motor Conveyor (DRV8833)
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  analogWriteFreq(CONVEYOR_PWM_FREQ);

  // Attach Servo Pins
  servoBase.attach(PIN_SERVO1, 500, 2500);
  servoShoulder.attach(PIN_SERVO2, 500, 2500);
  servoElbow.attach(PIN_SERVO3, 500, 2500);
  servoGripper.attach(PIN_SERVO4, 500, 2500);

  // Posisi awal langsung ke Stand By
  servoBase.write((int)FIXED_BASE);
  servoShoulder.write((int)currentShoulder);
  servoElbow.write(180 - (int)currentElbow);
  servoGripper.write((int)currentGripper);

  Serial.println("--- Otomasi Robot Arm + Sensor IR + Conveyor ---");
  delay(3000); // Tahan posisi Stand By awal

  // Conveyor mulai berputar, menunggu benda
  if (CONVEYOR_ON) conveyorRun(CONVEYOR_SPEED);
  lastCycleEnd = millis() - COOLDOWN_MS; // Sensor langsung aktif
}

void loop() {
  // Abaikan sensor selama masa cooldown setelah siklus selesai
  if (millis() - lastCycleEnd < COOLDOWN_MS) {
    yield();
    return;
  }

  if (objectDetected()) {
    Serial.println("[IR] Benda terdeteksi");

    // 1. Conveyor berhenti
    conveyorStop();
    delay(STOP_SETTLE_MS); // Tunggu benda benar-benar diam

    // 2. Jalankan fungsi arm
    runArmSequence();

    // 3. Arm sudah kembali ke Stand By -> conveyor jalan lagi
    if (CONVEYOR_ON) conveyorRun(CONVEYOR_SPEED);
    lastCycleEnd = millis();
    Serial.println("[IR] Siklus selesai, menunggu benda berikutnya");
  }

  yield();
}