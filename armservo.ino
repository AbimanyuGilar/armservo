#include <ESP8266WiFi.h>
#include <Servo.h>
#include <ModbusIP_ESP8266.h>

// ===== Konfigurasi Wi-Fi =====
const char* ssid = "NAMA_WIFI_ANDA";
const char* password = "PASSWORD_WIFI_ANDA";

// Inisialisasi Modbus IP Object
ModbusIP mb;

// Inisialisasi 4 Servo
Servo servoBase;     
Servo servoShoulder; 
Servo servoElbow;    
Servo servoGripper;  

// Pin GPIO Servo
const int PIN_SERVO1 = 5;
const int PIN_SERVO2 = 4;
const int PIN_SERVO3 = 14;
const int PIN_SERVO4 = 12;

// ===== Sensor Infrared =====
const int  PIN_IR = 16;              
const bool IR_ACTIVE_LOW = true;     
const int  IR_CONFIRM_MS = 10;       

// ===== Motor Conveyor via DRV8833 =====
const int PIN_IN1 = 13; 
const int PIN_IN2 = 15; 
const int CONVEYOR_SPEED = 195;     
const int CONVEYOR_PWM_FREQ = 1000; 
const int CONVEYOR_PWM_MAX = 1023;  

// Posisi Stand By Awal
const float FIXED_BASE  = 180.0;
float currentShoulder   = 150.0;
float currentElbow      = 120.0;
float currentGripper    = 40.0;

// Parameter kecepatan
const float DEFAULT_STEP = 1.5; 
const int   DEFAULT_DLY  = 10;  
const float HEAVY_STEP = 0.8;
const int   HEAVY_DLY  = 15;
const int SETTLE_MS = 400;

// Variabel status untuk Modbus
bool lastConveyorState = false;
bool isArmRunning = false;

// Alamat Modbus Coils / Register
const int MB_CONVEYOR_CMD = 0; // %QX0.0 (Dari OpenPLC)
const int MB_ARM_CMD      = 1; // %QX0.1 (Dari OpenPLC)
const int MB_IR_STATUS    = 0; // %IX0.0 (Ke OpenPLC)
const int MB_ARM_BUSY     = 1; // %IX0.1 (Ke OpenPLC)

// Jalankan motor conveyor
void conveyorRun(int speed) {
  speed = constrain(speed, 0, CONVEYOR_PWM_MAX);
  digitalWrite(PIN_IN2, LOW);
  analogWrite(PIN_IN1, speed);
  Serial.println("[CONVEYOR] RUN");
}

// Hentikan motor conveyor
void conveyorStop() {
  analogWrite(PIN_IN1, 0);
  digitalWrite(PIN_IN2, LOW);
  Serial.println("[CONVEYOR] STOP");
}

// Fungsi Interpolasi Mulus 3 Servo
void move3AxisSmooth(float targetShoulder, float targetElbow, float targetGripper,
                     float stepSpeed = DEFAULT_STEP, int stepDelay = DEFAULT_DLY) {
  bool inPosition = false;

  while (!inPosition) {
    inPosition = true;

    if (abs(targetShoulder - currentShoulder) > stepSpeed) {
      currentShoulder += (targetShoulder > currentShoulder) ? stepSpeed : -stepSpeed;
      inPosition = false;
    } else {
      currentShoulder = targetShoulder;
    }

    if (abs(targetElbow - currentElbow) > stepSpeed) {
      currentElbow += (targetElbow > currentElbow) ? stepSpeed : -stepSpeed;
      inPosition = false;
    } else {
      currentElbow = targetElbow;
    }

    if (abs(targetGripper - currentGripper) > stepSpeed) {
      currentGripper += (targetGripper > currentGripper) ? stepSpeed : -stepSpeed;
      inPosition = false;
    } else {
      currentGripper = targetGripper;
    }

    servoShoulder.write((int)currentShoulder);
    servoElbow.write((int)currentElbow); 
    servoGripper.write((int)currentGripper);

    // Update server modbus agar tetap merespons saat blocking loop servo
    mb.task();
    delay(stepDelay);
    yield();
  }
  delay(SETTLE_MS);
}

// Baca sensor IR mentah
bool irRaw() {
  int v = digitalRead(PIN_IR);
  return IR_ACTIVE_LOW ? (v == LOW) : (v == HIGH);
}

// Deteksi benda dengan anti-noise
bool objectDetected() {
  if (!irRaw()) return false;
  unsigned long start = millis();
  while (millis() - start < IR_CONFIRM_MS) {
    if (!irRaw()) return false;
    mb.task();
    yield();
  }
  return true;
}

// Urutan kerja arm
void runArmSequence() {
  isArmRunning = true;
  mb.Coil(MB_ARM_BUSY, true);

  Serial.println("[ARM] Memulai sekuens pick and place...");

  // 2. Move (Shoulder: 100, Elbow: 55, Gripper: 40)
  move3AxisSmooth(100.0, 55.0, 40.0);
  delay(1000);

  // 3. Grip 
  move3AxisSmooth(100.0, 55.0, 120.0);
  delay(1000);

  // 4a. Siku Dulu (110, 175, 120)
  move3AxisSmooth(110.0, 175.0, 120.0);
  delay(1000); 

  // 4b. Shoulder Menyusul 
  move3AxisSmooth(55.0, 175.0, 110.0, HEAVY_STEP, HEAVY_DLY);
  delay(1000);

  // 5. Place 
  move3AxisSmooth(55.0, 175.0, 40.0);
  delay(1000);

  // 6. Stand By 
  move3AxisSmooth(150.0, 120.0, 40.0, HEAVY_STEP, HEAVY_DLY);
  delay(500);

  isArmRunning = false;
  mb.Coil(MB_ARM_BUSY, false);
  mb.Coil(MB_ARM_CMD, false); // Reset trigger command dari PLC
  Serial.println("[ARM] Sekuens selesai.");
}

void setup() {
  Serial.begin(115200);

  // Hubungkan ke Wi-Fi untuk Modbus TCP
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("WiFi terhubung! IP Address: ");
  Serial.println(WiFi.localIP());

  // Inisialisasi Modbus IP
  mb.server();

  // Tambahkan Coils untuk I/O Mapping
  mb.addCoil(MB_CONVEYOR_CMD, false); // %QX0.0
  mb.addCoil(MB_ARM_CMD, false);      // %QX0.1
  mb.addCoil(MB_IR_STATUS, false);    // %IX0.0
  mb.addCoil(MB_ARM_BUSY, false);     // %IX0.1

  // Konfigurasi Sensor IR
  pinMode(PIN_IR, INPUT);

  // Konfigurasi Motor Conveyor
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  analogWriteFreq(CONVEYOR_PWM_FREQ);

  // Attach Servo Pins
  servoBase.attach(PIN_SERVO1, 500, 2500);
  servoShoulder.attach(PIN_SERVO2, 500, 2500);
  servoElbow.attach(PIN_SERVO3, 500, 2500);
  servoGripper.attach(PIN_SERVO4, 500, 2500);

  // Posisi awal Stand By
  servoBase.write((int)FIXED_BASE);
  servoShoulder.write((int)currentShoulder);
  servoElbow.write((int)currentElbow);
  servoGripper.write((int)currentGripper);

  Serial.println("--- ESP8266 Modbus TCP Slave Siap ---");
  delay(2000);
}

void loop() {
  // 1. Proses komunikasi Modbus
  mb.task();

  // 2. Baca sensor IR dan update ke Modbus server
  bool irDetected = objectDetected();
  mb.Coil(MB_IR_STATUS, irDetected);

  // 3. Kontrol Conveyor berdasarkan perintah dari OpenPLC (%QX0.0)
  bool conveyorCmd = mb.Coil(MB_CONVEYOR_CMD);
  if (conveyorCmd && !isArmRunning) {
    conveyorRun(CONVEYOR_SPEED);
  } else {
    conveyorStop();
  }

  // 4. Jalankan arm jika ada perintah dari OpenPLC (%QX0.1)
  if (mb.Coil(MB_ARM_CMD) && !isArmRunning) {
    runArmSequence();
  }

  yield();
}