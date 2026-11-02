/*
 * ============================================================
 *  ESP8266 ROBOT ARM - KONTROL LANGSUNG DARI OPENPLC
 * ============================================================
 *  Alamat Holding Register di OpenPLC:
 *    %QW0 (Offset 0) -> Servo Base     (Pin D1 / GPIO 5)
 *    %QW1 (Offset 1) -> Servo Shoulder (Pin D2 / GPIO 4)
 *    %QW2 (Offset 2) -> Servo Elbow    (Pin D5 / GPIO 14)
 *    %QW3 (Offset 3) -> Servo Gripper  (Pin D6 / GPIO 12)
 *
 *  Cara Kerja:
 *    Begitu OpenPLC mengeksekusi blok MOVE dengan output ke %QW
 *    (misal MOVE 180 ke %QW2), servo langsung bergerak ke sudut itu!
 * ============================================================
 */

#include <ESP8266WiFi.h>
#include <ModbusIP_ESP8266.h>
#include <Servo.h>

#if __has_include("secrets.h")
#include "secrets.h"
#endif

#ifndef WIFI_SSID
#define WIFI_SSID "apacoba"
#define WIFI_PASS "12345678"
#endif

// ===== Pin GPIO Servo =====
const int PIN_SERVO_BASE     = 5;  // D1
const int PIN_SERVO_SHOULDER = 4;  // D2
const int PIN_SERVO_ELBOW    = 14; // D5
const int PIN_SERVO_GRIPPER  = 12; // D6

// ===== Alamat Register Modbus =====
const uint16_t REG_BASE     = 0; // %QW0
const uint16_t REG_SHOULDER = 1; // %QW1
const uint16_t REG_ELBOW    = 2; // %QW2
const uint16_t REG_GRIPPER  = 3; // %QW3

// Objek Servo & Modbus
Servo servoBase;
Servo servoShoulder;
Servo servoElbow;
Servo servoGripper;

ModbusIP mb;

// Simpan nilai sudut sebelumnya untuk deteksi perubahan
int lastBase     = -1;
int lastShoulder = -1;
int lastElbow    = -1;
int lastGripper  = -1;

unsigned long lastHeartbeat = 0;

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println();
  Serial.println("==========================================");
  Serial.println("  ESP8266 Server Modbus TCP untuk OpenPLC ");
  Serial.println("==========================================");

  // Attach Servo
  servoBase.attach(PIN_SERVO_BASE, 500, 2500);
  servoShoulder.attach(PIN_SERVO_SHOULDER, 500, 2500);
  servoElbow.attach(PIN_SERVO_ELBOW, 500, 2500);
  servoGripper.attach(PIN_SERVO_GRIPPER, 500, 2500);

  // Posisi default awal saat ESP baru nyala
  servoBase.write(65);
  servoShoulder.write(150);
  servoElbow.write(90);
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

  // Daftarkan Holding Register dengan nilai awal
  mb.addHreg(REG_BASE,     65);
  mb.addHreg(REG_SHOULDER, 150);
  mb.addHreg(REG_ELBOW,    90);
  mb.addHreg(REG_GRIPPER,  40);

  lastBase     = 65;
  lastShoulder = 150;
  lastElbow    = 90;
  lastGripper  = 40;

  Serial.println("[MODBUS] Siap menerima nilai MOVE dari OpenPLC!");
  Serial.println("==========================================");
}

void loop() {
  // Layani koneksi Modbus dari OpenPLC
  mb.task();

  // Ambil nilai register terkini yang dikirim OpenPLC
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
    servoElbow.write(valElbow);
    Serial.printf("[MOVE] Elbow (%QW2) -> %d deg\n", valElbow);
  }

  if (valGripper != lastGripper) {
    lastGripper = valGripper;
    servoGripper.write(valGripper);
    Serial.printf("[MOVE] Gripper (%QW3) -> %d deg\n", valGripper);
  }

  // Cetak status heartbeat tiap 3 detik
  if (millis() - lastHeartbeat >= 3000) {
    lastHeartbeat = millis();
    Serial.printf("[STATUS] IP: %s | Base: %d | Shoulder: %d | Elbow: %d | Gripper: %d\n",
                  WiFi.localIP().toString().c_str(),
                  valBase, valShoulder, valElbow, valGripper);
  }

  yield();
}
