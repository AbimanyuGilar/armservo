/*
 * ============================================================
 *  TES IR + MODBUS TCP (ESP8266 / NodeMCU)
 * ============================================================
 *  Tujuan: membuktikan jalur ESP8266 -> OpenPLC untuk sensor IR
 *  saja, tanpa servo/arm.
 *
 *  Peta alamat (HARUS sama dengan OpenPLC):
 *    %IX0.0  IR          <- ESP, FC 01 (Read Coils), offset 2
 *    %QX0.0  CONVEYOR    -> ESP, FC 05 (Write Coil),  offset 0
 *    %QW0    mirror IR   <- ESP, FC 03 (Read Holding), offset 0 (opsional)
 *
 *  Catatan: OpenPLC di project ini membaca IR dengan "Digital Input
 *  (Coil Status)" = FC 01 offset 2. Jadi IR WAJIB ditulis ke COIL 2,
 *  bukan coil 0 dan bukan discrete input (mb.Ists).
 *
 *  Serial Monitor: 115200 baud
 * ============================================================
 */

#include <ESP8266WiFi.h>
#include <ModbusIP_ESP8266.h>
#include "secrets.h"   // berisi WIFI_SSID & WIFI_PASS (tidak di-commit)

// ===== Pin =====
const uint8_t PIN_IR        = 16;          // D0  <- OUT sensor IR
const uint8_t PIN_IN1       = 13;          // D7  -> DRV8833 IN1 (PWM)
const uint8_t PIN_IN2       = 15;          // D8  -> DRV8833 IN2 (LOW)
const uint8_t STATUS_LED    = LED_BUILTIN; // LED onboard (D4), active LOW

const bool IR_ACTIVE_LOW    = true;        // FC-51: LOW = ada benda
const int  IR_DEBOUNCE_MS   = 20;

// ===== Alamat Modbus =====
const uint16_t MB_COIL_CONVEYOR = 0;  // %QX0.0 dari OpenPLC
const uint16_t MB_COIL_IR       = 2;  // %IX0.0 ke OpenPLC (FC01 offset 2)
const uint16_t MB_HREG_IR       = 0;  // %QW0 mirror, buat cek manual

// PENTING: config di runtime sekarang masih membaca IR di offset 0.
// Ubah grup IR di OpenPLC jadi offset 2 (coil 2) supaya coil 0 tetap
// khusus conveyor. Kalau belum diubah, sementara set MB_COIL_IR = 0.

// ===== Conveyor =====
const int CONVEYOR_SPEED = 195;       // 0..1023
bool conveyorRunning = false;

// ===== State IR =====
bool irState     = false;   // hasil debounce
bool irRawLast   = false;   // pembacaan mentah terakhir
unsigned long irRawChangeAt = 0;
unsigned long lastHeartbeat = 0;

ModbusIP mb;

bool irRaw() {
  int v = digitalRead(PIN_IR);
  return IR_ACTIVE_LOW ? (v == LOW) : (v == HIGH);
}

void conveyorRun(int speed) {
  speed = constrain(speed, 0, 1023);
  digitalWrite(PIN_IN2, LOW);
  analogWrite(PIN_IN1, speed);
  conveyorRunning = true;
  Serial.println("[CONVEYOR] RUN");
}

void conveyorStop() {
  analogWrite(PIN_IN1, 0);
  digitalWrite(PIN_IN2, LOW);
  conveyorRunning = false;
  Serial.println("[CONVEYOR] STOP");
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(PIN_IR, INPUT);   // GPIO16 tidak punya pull-up internal
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);

  digitalWrite(STATUS_LED, HIGH); // LED mati (active low)
  digitalWrite(PIN_IN2, LOW);
  analogWriteRange(1023);
  analogWriteFreq(1000);
  analogWrite(PIN_IN1, 0);

  Serial.println();
  Serial.println("=== TES IR + MODBUS TCP ===");
  Serial.print("Menghubungkan ke WiFi: ");
  Serial.println(WIFI_SSID);

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setSleepMode(WIFI_NONE_SLEEP); // penting: modem sleep bikin Modbus timeout

  // OPSIONAL: IP statis. Hilangkan komentar kalau IP DHCP berubah-ubah
  // dan OpenPLC sudah terlanjur diarahkan ke 192.168.1.17.
  // IPAddress ip(192, 168, 1, 17);
  // IPAddress gw(192, 168, 1, 1);
  // IPAddress sn(255, 255, 255, 0);
  // WiFi.config(ip, gw, sn);

  WiFi.begin(WIFI_SSID, WIFI_PASS);

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 30000) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("!!! WiFi GAGAL TERHUBUNG");
    Serial.print("    status = ");
    Serial.println(WiFi.status());
    Serial.println("    Cek SSID/password di secrets.h (ESP8266 hanya 2.4 GHz)");
    // Tetap lanjut supaya IP AP/TIDAK ada; sensor tetap bisa dites via Serial
  } else {
    Serial.print("WiFi OK. IP ESP = ");
    Serial.println(WiFi.localIP());
    Serial.print("MAC = ");
    Serial.println(WiFi.macAddress());
    Serial.print("RSSI = ");
    Serial.println(WiFi.RSSI());
  }

  mb.server();
  mb.addCoil(MB_COIL_CONVEYOR, false);
  mb.addCoil(MB_COIL_IR, false);
  mb.addHreg(MB_HREG_IR, 0);

  Serial.print("Modbus TCP server aktif di port 502.");
  Serial.println(" IR -> coil 2 (FC01)");
  Serial.println("Masukkan IP ESP di atas ke OpenPLC: Remote Device 'esp'");
  Serial.println("-----------------------------------------------");
}

void loop() {
  mb.task(); // jangan pernah diblokir lama

  // --- Debounce IR (non-blocking) ---
  bool raw = irRaw();
  if (raw != irRawLast) {
    irRawLast = raw;
    irRawChangeAt = millis();
  }
  if (raw != irState && millis() - irRawChangeAt >= IR_DEBOUNCE_MS) {
    irState = raw;
    Serial.print("[IR] ");
    Serial.println(irState ? "ADA BENDA (1)" : "kosong (0)");
  }

  // --- Kirim ke OpenPLC ---
  mb.Coil(MB_COIL_IR, irState);          // %IX0.0, FC01 offset 2
  mb.Hreg(MB_HREG_IR, irState ? 1 : 0);  // %QW0, cek manual

  // --- LED onboard mengikuti sensor (bukti ESP benar-benar membaca pin) ---
  digitalWrite(STATUS_LED, irState ? LOW : HIGH);

  // --- Terima perintah conveyor dari OpenPLC ---
  bool cmd = mb.Coil(MB_COIL_CONVEYOR);
  if (cmd && !conveyorRunning)      conveyorRun(CONVEYOR_SPEED);
  else if (!cmd && conveyorRunning) conveyorStop();

  // --- Heartbeat tiap 5 detik ---
  if (millis() - lastHeartbeat >= 5000) {
    lastHeartbeat = millis();
    Serial.print("[STATUS] pin=");
    Serial.print(digitalRead(PIN_IR) == HIGH ? "HIGH" : "LOW");
    Serial.print(" | IR=");
    Serial.print(irState ? 1 : 0);
    Serial.print(" | coil2=");
    Serial.print(mb.Coil(MB_COIL_IR) ? 1 : 0);
    Serial.print(" | conveyorCmd=");
    Serial.print(cmd ? 1 : 0);
    Serial.print(" | IP=");
    Serial.println(WiFi.localIP());
  }

  yield();
}
