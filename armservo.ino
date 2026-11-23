/*
 * ==============================================================================
 *  ESP8266 ROBOT ARM + CONVEYOR + SENSOR IR - KONTROL MODBUS TCP DARI OPENPLC
 * ==============================================================================
 *  Pinout ESP8266 (NodeMCU):
 *    - Servo Base     : GPIO 5  (D1)
 *    - Servo Shoulder : GPIO 4  (D2)
 *    - Servo Elbow    : GPIO 14 (D5)
 *    - Servo Gripper  : GPIO 12 (D6)
 *    - Sensor IR OUT  : GPIO 16 (D0)  -> FC-51 (LOW = Benda Terdeteksi)
 *    - Conveyor IN1   : GPIO 13 (D7)  -> DRV8833 IN1 (PWM Kecepatan)
 *    - Conveyor IN2   : GPIO 15 (D8)  -> DRV8833 IN2 (GND / LOW = Putar 1 arah)
 *    - LED Indikator  : LED_BUILTIN (D4, Active LOW = Nyala saat ada benda)
 *
 *  Pemetaan Modbus TCP (Port 502) - SESUAI CONFIG OPENPLC:
 *  -----------------------------------------------------------------------------
 *  1. Holding Register (FC 06 / FC 16 / FC 03):
 *     - %QW0 (Offset 0) -> Sudut Servo Base     (0 - 180 deg, default: 65)
 *     - %QW1 (Offset 1) -> Sudut Servo Shoulder (0 - 180 deg, default: 150)
 *     - %QW2 (Offset 2) -> Sudut Servo Elbow    (0 - 180 deg, default: 90)
 *     - %QW3 (Offset 3) -> Sudut Servo Gripper  (0 - 180 deg, default: 40)
 *     - %QW4 (Offset 4) -> Mirror Sensor IR     (0 = Kosong, 1 = Ada Benda)
 *
 *  2. Coils (FC 01 / FC 05 / FC 15):
 *     - %QX0.0 (Offset 4) -> Kontrol Conveyor   (PLC -> ESP: 1 = RUN, 0 = STOP)
 *     - %IX0.0 (Offset 6) -> Status Sensor IR   (ESP -> PLC: 1 = Ada Benda)
 * ==============================================================================
 */

#include <ESP8266WiFi.h>
#include <ModbusIP_ESP8266.h>
#include <Servo.h>

#if __has_include("secrets.h")
#include "secrets.h"
#endif

// ===== Konfigurasi Pin GPIO Servo =====
const int PIN_SERVO_BASE     = 5;  // D1
const int PIN_SERVO_SHOULDER = 4;  // D2
const int PIN_SERVO_ELBOW    = 14; // D5
const int PIN_SERVO_GRIPPER  = 12; // D6

// Set true jika orientasi servo Elbow dipasang terbalik secara fisik
const bool INVERT_ELBOW = false;

// ===== Konfigurasi Smooth Movement Servo (Non-Blocking) =====
// Semakin KECIL nilai ms = semakin CEPAT gerakan servo (misal: 8 - 12 ms untuk arm, 4 - 8 ms untuk gripper)
// Semakin BESAR nilai ms = semakin LAMBAT & HALUS
const unsigned long STEP_DELAY_BASE     = 12; // ms per 1 derajat
const unsigned long STEP_DELAY_SHOULDER = 15; // ms per 1 derajat (sedikit lebih pelan karena menahan beban)
const unsigned long STEP_DELAY_ELBOW    = 14; // ms per 1 derajat
const unsigned long STEP_DELAY_GRIPPER  = 8;  // ms per 1 derajat

// ===== Konfigurasi Pin Sensor IR =====
const int  PIN_IR          = 16;          // D0 -> OUT sensor IR FC-51
const bool IR_ACTIVE_LOW   = true;        // Sensor FC-51: LOW saat ada benda
const int  IR_DEBOUNCE_MS  = 20;          // Filter anti-noise non-blocking (ms)
const int  PIN_STATUS_LED  = LED_BUILTIN; // D4 -> LED onboard ESP8266 (Active LOW)

// ===== Konfigurasi Conveyor (Driver DRV8833) =====
const int PIN_CONV_IN1        = 13;   // D7 -> DRV8833 IN1 (PWM)
const int PIN_CONV_IN2        = 15;   // D8 -> DRV8833 IN2 (LOW)
const int CONV_START_SPEED    = 500;  // Kecepatan awal saat conveyor mulai jalan (0 - 1023)
const int CONV_RUN_SPEED      = 350;  // Kecepatan normal setelah kick-start (0 - 1023)
const unsigned long CONV_KICK_TIME = 300; // Durasi kecepatan awal sebelum turun (ms)
const int CONV_PWM_FREQ       = 1000; // Frekuensi PWM (Hz)

// ===== Alamat Register Modbus (Cocok Persis dengan OpenPLC Anda) =====
const uint16_t REG_BASE        = 0; // %QW0 -> Offset 0
const uint16_t REG_SHOULDER    = 1; // %QW1 -> Offset 1
const uint16_t REG_ELBOW       = 2; // %QW2 -> Offset 2
const uint16_t REG_GRIPPER     = 3; // %QW3 -> Offset 3
const uint16_t REG_IR_STATUS   = 4; // %QW4 / %IW4 mirror -> Offset 4

const uint16_t COIL_CONVEYOR   = 4; // %QX0.0 -> Offset 4 (FC 05 Write Coil)
const uint16_t COIL_IR         = 5; // %IX0.0 -> Offset 5 (FC 01 Read Coils)

// ===== Objek Servo & Modbus =====
Servo servoBase;
Servo servoShoulder;
Servo servoElbow;
Servo servoGripper;

ModbusIP mb;

// ===== Variabel Posisi & Smooth Servo =====
int curBase     = 65;
int curShoulder = 150;
int curElbow    = 90;
int curGripper  = 40;

int targetBase     = 65;
int targetShoulder = 150;
int targetElbow    = 90;
int targetGripper  = 40;

int lastLoggedBase     = -1;
int lastLoggedShoulder = -1;
int lastLoggedElbow    = -1;
int lastLoggedGripper  = -1;

unsigned long lastStepBase     = 0;
unsigned long lastStepShoulder = 0;
unsigned long lastStepElbow    = 0;
unsigned long lastStepGripper  = 0;

// ===== Variabel Status Conveyor =====
bool conveyorRunning          = false;
int currentConvSpeed          = 0;
unsigned long convStartTime   = 0;

// ===== Variabel Status IR (Debounce Non-Blocking) =====
bool irDetected        = false;
bool lastIrRaw         = false;
unsigned long lastIrChangeTime = 0;

// ===== Timer Heartbeat =====
unsigned long lastHeartbeat = 0;

// Helper Mengatur Kecepatan PWM Conveyor
void conveyorSetSpeed(int speed) {
  speed = constrain(speed, 0, 1023);
  currentConvSpeed = speed;
  digitalWrite(PIN_CONV_IN2, LOW);
  analogWrite(PIN_CONV_IN1, speed);
}

// Fungsi Menjalankan Conveyor
void conveyorRun(int speed) {
  conveyorSetSpeed(speed);
  conveyorRunning = true;
  convStartTime = millis();
  Serial.printf("[CONVEYOR] RUN (Start Speed: %d)\n", speed);
}

// Fungsi Menghentikan Conveyor
void conveyorStop() {
  conveyorSetSpeed(0);
  conveyorRunning = false;
  Serial.println("[CONVEYOR] STOP");
}

// Baca Sensor IR mentah (FC-51: LOW saat mendeteksi pantulan benda)
bool readIrRaw() {
  int v = digitalRead(PIN_IR);
  return IR_ACTIVE_LOW ? (v == LOW) : (v == HIGH);
}

// Update status IR ke register Modbus
void updateIrModbus(bool state) {
  mb.Coil(COIL_IR, state);
  mb.Coil(2, state);
  mb.Ists(0, state);
  mb.Ists(2, state);
  mb.Ists(6, state);

  mb.Hreg(REG_IR_STATUS, state ? 1 : 0);

  digitalWrite(PIN_STATUS_LED, state ? LOW : HIGH);
}

// Helper: Menggerakkan satu servo selangkah demi selangkah secara non-blocking
void stepServoSmooth(Servo &srv, int &currentPos, int targetPos,
                     unsigned long &lastStepTime, unsigned long stepDelayMs,
                     bool invertOutput = false) {
  if (currentPos == targetPos) return;

  unsigned long now = millis();
  if ((now - lastStepTime) >= stepDelayMs) {
    lastStepTime = now;
    if (currentPos < targetPos) {
      currentPos++;
    } else {
      currentPos--;
    }

    int outAngle = invertOutput ? (180 - currentPos) : currentPos;
    srv.write(constrain(outAngle, 0, 180));
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println();
  Serial.println("==========================================================");
  Serial.println(" ESP8266 Server Modbus TCP - Robot Arm + Conveyor + IR   ");
  Serial.println("==========================================================");

  // Inisialisasi Pin IR & LED Indikator
  pinMode(PIN_IR, INPUT);
  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, HIGH);

  // Inisialisasi Pin Conveyor
  pinMode(PIN_CONV_IN1, OUTPUT);
  pinMode(PIN_CONV_IN2, OUTPUT);
  digitalWrite(PIN_CONV_IN2, LOW);
  analogWriteRange(1023);
  analogWriteFreq(CONV_PWM_FREQ);
  analogWrite(PIN_CONV_IN1, 0);

  // Attach Servo
  servoBase.attach(PIN_SERVO_BASE, 500, 2500);
  servoShoulder.attach(PIN_SERVO_SHOULDER, 500, 2500);
  servoElbow.attach(PIN_SERVO_ELBOW, 500, 2500);
  servoGripper.attach(PIN_SERVO_GRIPPER, 500, 2500);

  // Posisi default awal saat ESP baru nyala
  curBase     = 65;
  curShoulder = 150;
  curElbow    = 90;
  curGripper  = 40;

  targetBase     = curBase;
  targetShoulder = curShoulder;
  targetElbow    = curElbow;
  targetGripper  = curGripper;

  servoBase.write(curBase);
  servoShoulder.write(curShoulder);
  servoElbow.write(INVERT_ELBOW ? (180 - curElbow) : curElbow);
  servoGripper.write(curGripper);

  // Hubungkan ke WiFi
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);

  Serial.print("[WIFI] Menghubungkan ke ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  unsigned long startWifi = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startWifi < 20000) {
    delay(300);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WIFI] Terhubung!");
    Serial.print("  -> IP ESP: ");
    Serial.println(WiFi.localIP());
    Serial.println("  -> CATAT IP DI ATAS DAN MASUKKAN KE OPENPLC REMOTE DEVICE!");
  } else {
    Serial.println("\n[WIFI] GAGAL TERHUBUNG (Timeout 20 detik)!");
    Serial.println("  -> Pastikan SSID & Password di secrets.h sudah benar.");
    Serial.println("  -> Pastikan ESP dan Laptop berada di jaringan WiFi yang SAMA.");
  }

  // Jalankan Modbus TCP Server (Port 502)
  mb.server();

  // Daftarkan Holding Register (Offset 0..5)
  mb.addHreg(REG_BASE,      targetBase);     // Offset 0 (%QW0)
  mb.addHreg(REG_SHOULDER,  targetShoulder); // Offset 1 (%QW1)
  mb.addHreg(REG_ELBOW,     targetElbow);    // Offset 2 (%QW2)
  mb.addHreg(REG_GRIPPER,   targetGripper);  // Offset 3 (%QW3)
  mb.addHreg(REG_IR_STATUS, 0);              // Offset 4 (%QW4 / %IW4)
  mb.addHreg(5,             0);              // Offset 5

  // Daftarkan Coils 0 s/d 7 secara berurutan
  for (int c = 0; c < 8; c++) {
    mb.addCoil(c, false);
  }

  // Daftarkan Discrete Inputs 0 s/d 7 secara berurutan
  for (int i = 0; i < 8; i++) {
    mb.addIsts(i, false);
  }

  // Baca kondisi awal fisik IR sebelum loop berjalan
  irDetected = readIrRaw();
  lastIrRaw  = irDetected;
  updateIrModbus(irDetected);

  Serial.println("[MODBUS] Siap melayani OpenPLC di Port 502:");
  Serial.println("  - %QW0: Base     (Offset 0)");
  Serial.println("  - %QW1: Shoulder (Offset 1)");
  Serial.println("  - %QW2: Elbow    (Offset 2)");
  Serial.println("  - %QW3: Gripper  (Offset 3)");
  Serial.println("  - %QX0.0: Conveyor (Coil Offset 4)");
  Serial.println("  - %IX0.0: Sensor IR (Coil Offset 6)");
  Serial.println("==========================================================");
}

void loop() {
  // 1. Layani komunikasi Modbus TCP dari OpenPLC
  mb.task();

  // 2. Pembacaan Sensor IR dengan Debounce Non-Blocking
  bool rawIr = readIrRaw();
  if (rawIr != lastIrRaw) {
    lastIrRaw = rawIr;
    lastIrChangeTime = millis();
  }

  if ((millis() - lastIrChangeTime) >= IR_DEBOUNCE_MS) {
    if (rawIr != irDetected) {
      irDetected = rawIr;
      updateIrModbus(irDetected);
      Serial.printf("[IR] Sensor berubah: %s\n", irDetected ? "BENDA TERDETEKSI (1)" : "KOSONG (0)");
    }
  }

  // 3. Kontrol Motor Conveyor dari OpenPLC
  bool cmdConv = mb.Coil(COIL_CONVEYOR) || mb.Coil(0);

  if (cmdConv) {
    if (!conveyorRunning) {
      conveyorRun(CONV_START_SPEED);
    } else if (currentConvSpeed != CONV_RUN_SPEED && (millis() - convStartTime >= CONV_KICK_TIME)) {
      conveyorSetSpeed(CONV_RUN_SPEED);
      Serial.printf("[CONVEYOR] Speed disesuaikan ke %d (setelah %lu ms)\n", CONV_RUN_SPEED, CONV_KICK_TIME);
    }
  } else if (conveyorRunning) {
    conveyorStop();
  }

  // 4. Baca target sudut dari Register OpenPLC (%QW0..%QW3)
  targetBase     = constrain((int)mb.Hreg(REG_BASE),     0, 180);
  targetShoulder = constrain((int)mb.Hreg(REG_SHOULDER), 0, 180);
  targetElbow    = constrain((int)mb.Hreg(REG_ELBOW),    0, 180);
  targetGripper  = constrain((int)mb.Hreg(REG_GRIPPER),  0, 180);

  // Log ke Serial hanya saat target baru berubah dari OpenPLC
  if (targetBase != lastLoggedBase) {
    lastLoggedBase = targetBase;
    Serial.printf("[TARGET] Base (%QW0) -> %d deg (Smooth)\n", targetBase);
  }
  if (targetShoulder != lastLoggedShoulder) {
    lastLoggedShoulder = targetShoulder;
    Serial.printf("[TARGET] Shoulder (%QW1) -> %d deg (Smooth)\n", targetShoulder);
  }
  if (targetElbow != lastLoggedElbow) {
    lastLoggedElbow = targetElbow;
    Serial.printf("[TARGET] Elbow (%QW2) -> %d deg (Smooth)\n", targetElbow);
  }
  if (targetGripper != lastLoggedGripper) {
    lastLoggedGripper = targetGripper;
    Serial.printf("[TARGET] Gripper (%QW3) -> %d deg (Smooth)\n", targetGripper);
  }

  // Eksekusi langkah gerakan smooth tiap servo secara independen
  stepServoSmooth(servoBase,     curBase,     targetBase,     lastStepBase,     STEP_DELAY_BASE,     false);
  stepServoSmooth(servoShoulder, curShoulder, targetShoulder, lastStepShoulder, STEP_DELAY_SHOULDER, false);
  stepServoSmooth(servoElbow,    curElbow,    targetElbow,    lastStepElbow,    STEP_DELAY_ELBOW,    INVERT_ELBOW);
  stepServoSmooth(servoGripper,  curGripper,  targetGripper,  lastStepGripper,  STEP_DELAY_GRIPPER,  false);

  // 5. Cetak status Heartbeat tiap 3 detik
  if (millis() - lastHeartbeat >= 3000) {
    lastHeartbeat = millis();
    Serial.printf("[STATUS] IP: %s | IR: %d | Conv: %s (%d) | Pos(B/S/E/G): %d/%d/%d/%d\n",
                  WiFi.localIP().toString().c_str(),
                  irDetected ? 1 : 0,
                  conveyorRunning ? "RUN" : "STOP",
                  currentConvSpeed,
                  curBase, curShoulder, curElbow, curGripper);
  }

  yield();
}