/*
 * ============================================================
 *  ESP8266 ROBOT ARM - KONTROL LANGSUNG DARI OPENPLC
 * ============================================================
 *  Alamat Holding Register di OpenPLC:
 *    %QW0 (Offset 0) -> Servo Base     (Pin D1 / GPIO 5)
 *    %QW1 (Offset 1) -> Servo Shoulder (Pin D2 / GPIO 4)
 *    %QW2 (Offset 2) -> Servo Elbow    (Pin D5 / GPIO 14)
 *    %QW3 (Offset 3) -> Servo Gripper  (Pin D6 / GPIO 12)
 *    %IW0 (Offset 0) <- Sensor IR      (Pin D0 / GPIO 16)
 *    %IW1 (Offset 1) <- Web: Base      (setpoint, bukan servo langsung)
 *    %IW2 (Offset 2) <- Web: Shoulder
 *    %IW3 (Offset 3) <- Web: Elbow
 *    %IW4 (Offset 4) <- Web: Gripper
 *
 *  Sensor IR dibaca sebagai Input Register (FC 04), jadi di OpenPLC
 *  muncul sebagai %IW0 dengan nilai 1 = ada benda, 0 = kosong.
 *
 *  Web UI (http://IP-ESP/) hanya menulis setpoint ke %IW1..%IW4.
 *  Servo TETAP digerakkan oleh logika OpenPLC lewat %QW0..%QW3.
 *
 *  Cara Kerja:
 *    Begitu OpenPLC mengeksekusi blok MOVE dengan output ke %QW
 *    (misal MOVE 180 ke %QW2), servo langsung bergerak ke sudut itu!
 * ============================================================
 */

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
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

// ===== Pin Sensor =====
const int PIN_SENSOR_D0 = 16; // D0 / GPIO16 (tidak punya pull-up internal)

// ===== Alamat Register Modbus =====
const uint16_t REG_BASE     = 0; // %QW0
const uint16_t REG_SHOULDER = 1; // %QW1
const uint16_t REG_ELBOW    = 2; // %QW2
const uint16_t REG_GRIPPER  = 3; // %QW3
const uint16_t REG_SENSOR   = 0; // %IW0 (Input Register, FC 04)

// ===== Input Register dari Web UI (ESP -> dibaca OpenPLC, FC 04) =====
// Web TIDAK menggerakkan servo langsung. Nilai ini hanya "setpoint" yang
// dibaca OpenPLC; logika PLC milikmu yang memutuskan kapan menggerakkan arm.
const uint16_t REG_WEB_BASE     = 1; // %IW1
const uint16_t REG_WEB_SHOULDER = 2; // %IW2
const uint16_t REG_WEB_ELBOW    = 3; // %IW3
const uint16_t REG_WEB_GRIPPER  = 4; // %IW4

// Objek Servo & Modbus
Servo servoBase;
Servo servoShoulder;
Servo servoElbow;
Servo servoGripper;

ModbusIP mb;
ESP8266WebServer webServer(80);

// ============================================================
//  HALAMAN WEB (disajikan ESP8266, port 80)
//  Slider hanya menulis %IW1..%IW4. Servo tidak digerakkan dari sini.
// ============================================================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="id">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Arm Servo - Web Setpoint</title>
<style>
  :root{--bg:#0f1220;--card:#1a1f35;--line:#2c3450;--txt:#e8ecf8;--mut:#8c96b5;--acc:#4da3ff}
  *{box-sizing:border-box}
  body{margin:0;font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif;background:radial-gradient(1200px 600px at 50% -10%,#1b2450,#0f1220 60%);color:var(--txt);min-height:100vh;padding:24px}
  .wrap{max-width:860px;margin:0 auto}
  h1{font-size:22px;margin:0 0 6px}
  .sub{color:var(--mut);font-size:13px;line-height:1.5;margin:0 0 18px}
  .status{display:flex;flex-wrap:wrap;gap:10px;margin-bottom:18px}
  .chip{background:#11162a;border:1px solid var(--line);border-radius:999px;padding:7px 12px;font-size:12px;color:var(--mut)}
  .chip b{color:var(--txt)}
  .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(340px,1fr));gap:14px}
  .card{background:linear-gradient(180deg,#1a2038,#161b2e);border:1px solid var(--line);border-radius:16px;padding:16px}
  .row{display:flex;justify-content:space-between;align-items:baseline;margin-bottom:8px}
  .name{font-size:14px;font-weight:600}
  .addr{font-size:11px;color:#6f7aa0}
  .val{font-variant-numeric:tabular-nums;font-size:24px;font-weight:700;color:var(--acc)}
  input[type=range]{width:100%;accent-color:#4da3ff;height:28px}
  .btns{display:flex;gap:10px;margin-top:18px}
  button{flex:1;background:var(--acc);color:#06122a;border:0;border-radius:12px;padding:13px;font-weight:700;font-size:14px;cursor:pointer}
  button.ghost{background:transparent;color:var(--txt);border:1px solid var(--line)}
  .foot{color:#6f7aa0;font-size:11px;margin-top:16px;text-align:center}
</style>
</head>
<body>
<div class="wrap">
  <h1>Robot Arm &mdash; Web Setpoint</h1>
  <p class="sub">Nilai di bawah dikirim ke OpenPLC sebagai <b>%IW1&ndash;%IW4</b>. Servo tetap digerakkan oleh logika OpenPLC, bukan langsung oleh web.</p>
  <div class="status">
    <span class="chip">Sensor %IW0: <b id="sensor">-</b></span>
    <span class="chip">Servo aktual (%QW): <b id="servo">-</b></span>
    <span class="chip">Koneksi: <b id="conn">-</b></span>
  </div>
  <div class="grid" id="grid"></div>
  <div class="btns">
    <button onclick="sendAll()">Kirim Semua ke OpenPLC</button>
    <button class="ghost" onclick="syncFromWeb()">Muat Nilai dari OpenPLC</button>
  </div>
  <div class="foot">Web hanya menulis input register. Perintah gerak tetap dari program OpenPLC.</div>
</div>
<script>
const defs=[
  {key:"base",name:"Base",addr:"%IW1",pin:"D1"},
  {key:"shoulder",name:"Shoulder",addr:"%IW2",pin:"D2"},
  {key:"elbow",name:"Elbow",addr:"%IW3",pin:"D5"},
  {key:"gripper",name:"Gripper",addr:"%IW4",pin:"D6"}
];
const grid=document.getElementById("grid");
const inputs={};
for(const d of defs){
  const card=document.createElement("div");
  card.className="card";
  card.innerHTML=`<div class="row"><div><div class="name">${d.name}</div><div class="addr">${d.addr} &middot; ${d.pin}</div></div><div class="val" id="v_${d.key}">90</div></div>
    <input type="range" min="0" max="180" step="1" value="90" id="i_${d.key}">`;
  grid.appendChild(card);
  const inp=card.querySelector("input");
  inputs[d.key]=inp;
  inp.addEventListener("input",()=>{document.getElementById("v_"+d.key).textContent=inp.value;});
  inp.addEventListener("change",()=>sendOne(d.key));
}
function setConn(ok){const e=document.getElementById("conn");e.textContent=ok?"OK":"GAGAL";e.style.color=ok?"#39d98a":"#ff6b6b";}
function sendOne(key){
  const v=inputs[key].value;
  fetch(`/api/set?${key}=${v}`).then(()=>setConn(true)).catch(()=>setConn(false));
}
function sendAll(){
  const p=defs.map(d=>`${d.key}=${inputs[d.key].value}`).join("&");
  fetch(`/api/set?${p}`).then(()=>setConn(true)).catch(()=>setConn(false));
}
function updateState(s){
  document.getElementById("sensor").textContent=s.sensor?"ADA BENDA (1)":"kosong (0)";
  document.getElementById("servo").textContent=`${s.servo.base}/${s.servo.shoulder}/${s.servo.elbow}/${s.servo.gripper}`;
  setConn(true);
}
function syncFromWeb(){
  fetch("/api/state").then(r=>r.json()).then(s=>{
    for(const d of defs){
      if(typeof s.web[d.key]==="number"){
        inputs[d.key].value=s.web[d.key];
        document.getElementById("v_"+d.key).textContent=s.web[d.key];
      }
    }
    updateState(s);
  }).catch(()=>setConn(false));
}
function poll(){
  fetch("/api/state").then(r=>r.json()).then(updateState).catch(()=>setConn(false));
}
syncFromWeb();
setInterval(poll,1500);
</script>
</body>
</html>
)rawliteral";

// Simpan nilai sudut sebelumnya untuk deteksi perubahan
int lastBase     = -1;
int lastShoulder = -1;
int lastElbow    = -1;
int lastGripper  = -1;

// ===== State Sensor D0 =====
const bool SENSOR_ACTIVE_LOW  = true;  // FC-51: LOW = ada benda
const int  SENSOR_DEBOUNCE_MS = 20;

bool sensorState     = false; // hasil debounce
bool sensorRawLast   = false; // pembacaan mentah terakhir
unsigned long sensorRawChangeAt = 0;

unsigned long lastHeartbeat = 0;

bool sensorRaw() {
  int v = digitalRead(PIN_SENSOR_D0);
  return SENSOR_ACTIVE_LOW ? (v == LOW) : (v == HIGH);
}

// ===== Handler Web UI =====
void handleRoot() {
  webServer.send_P(200, "text/html", INDEX_HTML);
}

// Simpan setpoint web ke Input Register. Hanya menerima nilai 0..180.
void handleSet() {
  if (webServer.hasArg("base")) {
    mb.Ireg(REG_WEB_BASE, constrain(webServer.arg("base").toInt(), 0, 180));
  }
  if (webServer.hasArg("shoulder")) {
    mb.Ireg(REG_WEB_SHOULDER, constrain(webServer.arg("shoulder").toInt(), 0, 180));
  }
  if (webServer.hasArg("elbow")) {
    mb.Ireg(REG_WEB_ELBOW, constrain(webServer.arg("elbow").toInt(), 0, 180));
  }
  if (webServer.hasArg("gripper")) {
    mb.Ireg(REG_WEB_GRIPPER, constrain(webServer.arg("gripper").toInt(), 0, 180));
  }
  webServer.send(200, "application/json", "{\"ok\":true}");
}

// Status untuk halaman web: setpoint web, posisi servo aktual, dan sensor.
void handleState() {
  char json[240];
  snprintf(json, sizeof(json),
           "{\"web\":{\"base\":%d,\"shoulder\":%d,\"elbow\":%d,\"gripper\":%d},"
           "\"servo\":{\"base\":%d,\"shoulder\":%d,\"elbow\":%d,\"gripper\":%d},"
           "\"sensor\":%d}",
           (int)mb.Ireg(REG_WEB_BASE),
           (int)mb.Ireg(REG_WEB_SHOULDER),
           (int)mb.Ireg(REG_WEB_ELBOW),
           (int)mb.Ireg(REG_WEB_GRIPPER),
           (int)mb.Hreg(REG_BASE),
           (int)mb.Hreg(REG_SHOULDER),
           (int)mb.Hreg(REG_ELBOW),
           (int)mb.Hreg(REG_GRIPPER),
           sensorState ? 1 : 0);
  webServer.send(200, "application/json", json);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println();
  Serial.println("==========================================");
  Serial.println("  ESP8266 Server Modbus TCP untuk OpenPLC ");
  Serial.println("==========================================");

  // Sensor IR di D0 sebagai input digital
  pinMode(PIN_SENSOR_D0, INPUT);

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

  // Input Register %IW0 untuk sensor D0
  mb.addIreg(REG_SENSOR, 0);

  // Input Register setpoint dari Web UI (%IW1..%IW4)
  mb.addIreg(REG_WEB_BASE,     65);
  mb.addIreg(REG_WEB_SHOULDER, 150);
  mb.addIreg(REG_WEB_ELBOW,    90);
  mb.addIreg(REG_WEB_GRIPPER,  40);

  // Web UI di port 80 (hanya menulis input register, bukan servo)
  webServer.on("/", handleRoot);
  webServer.on("/api/set", handleSet);
  webServer.on("/api/state", handleState);
  webServer.begin();

  lastBase     = 65;
  lastShoulder = 150;
  lastElbow    = 90;
  lastGripper  = 40;

  Serial.println("[MODBUS] Siap menerima nilai MOVE dari OpenPLC!");
  Serial.println("[MODBUS] Sensor D0 -> %IW0 (Input Register, FC 04)");
  Serial.println("[MODBUS] Setpoint Web -> %IW1..%IW4 (Input Register, FC 04)");
  Serial.print("[WEB] Buka di browser: http://");
  Serial.print(WiFi.localIP());
  Serial.println("/");
  Serial.println("==========================================");
}

void loop() {
  // Layani koneksi Modbus dari OpenPLC
  mb.task();

  // Layani halaman web (port 80)
  webServer.handleClient();

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

  // ===== Baca sensor di D0 -> %IW0 (Input Register, FC 04) =====
  bool raw = sensorRaw();
  if (raw != sensorRawLast) {
    sensorRawLast = raw;
    sensorRawChangeAt = millis();
  }
  if (raw != sensorState && millis() - sensorRawChangeAt >= SENSOR_DEBOUNCE_MS) {
    sensorState = raw;
    Serial.printf("[SENSOR] D0 (%IW0) -> %d (%s)\n",
                  sensorState ? 1 : 0,
                  sensorState ? "ADA BENDA" : "kosong");
  }
  mb.Ireg(REG_SENSOR, sensorState ? 1 : 0);

  // Cetak status heartbeat tiap 3 detik
  if (millis() - lastHeartbeat >= 3000) {
    lastHeartbeat = millis();
    Serial.printf("[STATUS] IP: %s | Base: %d | Shoulder: %d | Elbow: %d | Gripper: %d | Sensor: %d\n",
                  WiFi.localIP().toString().c_str(),
                  valBase, valShoulder, valElbow, valGripper, sensorState ? 1 : 0);
  }

  yield();
}
