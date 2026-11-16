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

// ===== Konfigurasi Pin Sensor IR =====
const int  PIN_IR          = 16;          // D0 -> OUT sensor IR FC-51
const bool IR_ACTIVE_LOW   = true;        // Sensor FC-51: LOW saat ada benda
const int  IR_DEBOUNCE_MS  = 20;          // Filter anti-noise non-blocking (ms)
const int  PIN_STATUS_LED  = LED_BUILTIN; // D4 -> LED onboard ESP8266 (Active LOW)

// ===== Konfigurasi Conveyor (Driver DRV8833) =====
const int PIN_CONV_IN1     = 13;   // D7 -> DRV8833 IN1 (PWM)
const int PIN_CONV_IN2     = 15;   // D8 -> DRV8833 IN2 (LOW)
const int CONVEYOR_SPEED   = 1000;  // Kecepatan conveyor ditentukan di sini (0 - 1023)
const int CONV_PWM_FREQ    = 1000; // Frekuensi PWM (Hz)

// ===== Alamat Register Modbus (Cocok Persis dengan OpenPLC Anda) =====
// Holding Registers (FC 06 / %QW dari OpenPLC)
const uint16_t REG_BASE        = 0; // %QW0 -> Offset 0
const uint16_t REG_SHOULDER    = 1; // %QW1 -> Offset 1
const uint16_t REG_ELBOW       = 2; // %QW2 -> Offset 2
const uint16_t REG_GRIPPER     = 3; // %QW3 -> Offset 3
const uint16_t REG_IR_STATUS   = 4; // %QW4 / %IW4 mirror -> Offset 4

// Coils (%QX dan %IX) Sesuai Gambar IO Tag Mapping OpenPLC
const uint16_t COIL_CONVEYOR   = 4; // %QX0.0 -> Offset 4 (FC 05 Write Coil)
const uint16_t COIL_IR         = 5; // %IX0.0 -> Offset 5 (FC 01 Read Coils)

// ===== Objek Servo & Modbus =====
Servo servoBase;
Servo servoShoulder;
Servo servoElbow;
Servo servoGripper;

ModbusIP mb;

// ===== Variabel Status Servo =====
int lastBase     = -1;
int lastShoulder = -1;
int lastElbow    = -1;
int lastGripper  = -1;

// ===== Variabel Status Conveyor =====
bool conveyorRunning = false;

// ===== Variabel Status IR (Debounce Non-Blocking) =====
bool irDetected        = false;
bool lastIrRaw         = false;
unsigned long lastIrChangeTime = 0;

// ===== Timer Heartbeat =====
unsigned long lastHeartbeat = 0;

// Fungsi Menjalankan Conveyor
void conveyorRun(int speed) {
  speed = constrain(speed, 0, 1023);
  digitalWrite(PIN_CONV_IN2, LOW);
  analogWrite(PIN_CONV_IN1, speed);
  conveyorRunning = true;
  Serial.printf("[CONVEYOR] RUN (Speed: %d)\n", speed);
}

// Fungsi Menghentikan Conveyor
void conveyorStop() {
  analogWrite(PIN_CONV_IN1, 0);
  digitalWrite(PIN_CONV_IN2, LOW);
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
  // Sesuai OpenPLC: Offset 6 (FC 01 Read Coils)
  mb.Coil(COIL_IR, state);
  // Cadangan jika OpenPLC masih membaca offset 2 atau 0
  mb.Coil(2, state);
  mb.Ists(0, state);
  mb.Ists(2, state);
  mb.Ists(6, state);

  // Mirror ke Holding Register (FC 03, offset 4)
  mb.Hreg(REG_IR_STATUS, state ? 1 : 0);

  // LED Onboard ESP8266 (Active LOW)
  digitalWrite(PIN_STATUS_LED, state ? LOW : HIGH);
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
  digitalWrite(PIN_STATUS_LED, HIGH); // LED Built-in mati di awal (Active LOW)

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
  servoBase.write(65);
  servoShoulder.write(150);
  servoElbow.write(INVERT_ELBOW ? (180 - 90) : 90);
  servoGripper.write(40);

  // Hubungkan ke WiFi
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.setSleepMode(WIFI_NONE_SLEEP); // Wajib agar Modbus tidak lag/timeout

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
  mb.addHreg(REG_BASE,      65);  // Offset 0 (%QW0)
  mb.addHreg(REG_SHOULDER,  150); // Offset 1 (%QW1)
  mb.addHreg(REG_ELBOW,     90);  // Offset 2 (%QW2)
  mb.addHreg(REG_GRIPPER,   40);  // Offset 3 (%QW3)
  mb.addHreg(REG_IR_STATUS, 0);   // Offset 4 (%QW4 / %IW4)
  mb.addHreg(5,             0);   // Offset 5

  lastBase     = 65;
  lastShoulder = 150;
  lastElbow    = 90;
  lastGripper  = 40;

  // Daftarkan Coils 0 s/d 7 secara berurutan agar request read tidak pernah error
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
  // 1. Layani komunikasi Modbus TCP dari OpenPLC (Wajib dipanggil sering)
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

  // 3. Kontrol Motor Conveyor dari OpenPLC via Coil (%QX0.0 di Offset 4, atau Offset 0)
  bool cmdConv = mb.Coil(COIL_CONVEYOR) || mb.Coil(0);

  if (cmdConv && !conveyorRunning) {
    conveyorRun(CONVEYOR_SPEED);
  } else if (!cmdConv && conveyorRunning) {
    conveyorStop();
  }

  // 4. Kontrol Servo Arm dari Register OpenPLC (%QW0..%QW3)
  int valBase     = constrain((int)mb.Hreg(REG_BASE),     0, 180);
  int valShoulder = constrain((int)mb.Hreg(REG_SHOULDER), 0, 180);
  int valElbow    = constrain((int)mb.Hreg(REG_ELBOW),    0, 180);
  int valGripper  = constrain((int)mb.Hreg(REG_GRIPPER),  0, 180);

  // Jika ada perubahan dari OpenPLC -> langsung gerakkan servo seketika!
  if (valBase != lastBase) {
    lastBase = valBase;
    servoBase.write(valBase);
    Serial.printf("[MOVE] Base (%QW0) -> %d deg\n", valBase);
  }

  if (valShoulder != lastShoulder) {
    lastShoulder = valShoulder;
    servoShoulder.write(valShoulder);
    Serial.printf("[MOVE] Shoulder (%QW1) -> %d deg\n", valShoulder);
  }

  if (valElbow != lastElbow) {
    lastElbow = valElbow;
    int targetElbow = INVERT_ELBOW ? (180 - valElbow) : valElbow;
    servoElbow.write(targetElbow);
    Serial.printf("[MOVE] Elbow (%QW2) -> %d deg (Fisik: %d deg)\n", valElbow, targetElbow);
  }

  if (valGripper != lastGripper) {
    lastGripper = valGripper;
    servoGripper.write(valGripper);
    Serial.printf("[MOVE] Gripper (%QW3) -> %d deg\n", valGripper);
  }

  // 5. Cetak status Heartbeat tiap 3 detik
  if (millis() - lastHeartbeat >= 3000) {
    lastHeartbeat = millis();
    Serial.printf("[STATUS] IP: %s | IR: %d | Conv: %s (%d) | Coil4(Conv): %d | Coil6(IR): %d\n",
                  WiFi.localIP().toString().c_str(),
                  irDetected ? 1 : 0,
                  conveyorRunning ? "RUN" : "STOP",
                  CONVEYOR_SPEED,
                  mb.Coil(COIL_CONVEYOR) ? 1 : 0,
                  mb.Coil(COIL_IR) ? 1 : 0);
  }

  yield();
}
