# Tes IR + Modbus TCP

Folder ini khusus untuk membuktikan sensor IR terbaca di OpenPLC **tanpa**
servo/arm/conveyor ikut bergerak.

## Kenapa sebelumnya gagal

1. `armservo.ino` yang ada di folder induk **mematikan WiFi**:
   `WiFi.mode(WIFI_OFF); WiFi.forceSleepBegin();` dan **tidak ada Modbus sama sekali**.
   Jadi ESP memang tidak pernah listen di port 502.
2. Config yang benar-benar di-upload ke runtime (`modbus_master.json` di container)
   memakai **IR = FC 01 offset 0**, dan **semua** register servo juga offset 0:
   `%QW0..%QW3` semuanya menulis ke holding register 0. Ini akan bikin 4 servo
   selalu dapat nilai yang sama. Alamat IR di coil 0 juga bentrok dengan
   `CONVEYOR` (%QX0.0) yang juga coil 0.
3. Alamat host di `devices/remote/esp.json` (file di disk) masih `10.165.137.211`
   (jaringan lain), sedangkan config yang jalan sudah `192.168.1.17:502`.
4. Saat dicek, tidak ada satu pun host di LAN yang membuka port 502 dan tidak ada
   MAC Espressif di tabel ARP → ESP tidak terhubung ke WiFi.

## Peta alamat yang dipakai firmware ini

| Sinyal | Arah | OpenPLC | Modbus | Catatan |
|---|---|---|---|---|
| IR | ESP → PLC | `%IX0.0` | FC 01 (Read Coils), **offset 2** | coil 2 |
| Conveyor | PLC → ESP | `%QX0.0` | FC 05 (Write Coil), offset 0 | coil 0 |
| Mirror IR | ESP → PLC | `%QW0` | FC 03, offset 0 | opsional, buat cek manual |
| Base/Shoulder/Elbow/Gripper | PLC → ESP | `%QW0..%QW3` | FC 06, offset 0/1/2/3 | untuk nanti |

Di OpenPLC Editor, grup remote device `esp` sebaiknya diisi:

| Grup | FC | Offset |
|---|---|---|
| BASE | 6 | 0 |
| SHOULDER | 6 | 1 |
| ELBOW | 6 | 2 |
| GRIPPER | 6 | 3 |
| CONVEYOR | 5 | 0 |
| IR | 1 | **2** |

Kalau grup IR masih offset 0 dan kamu belum mau mengubah OpenPLC, ubah
`MB_COIL_IR` di sketch menjadi `0` — tapi conveyor tidak bisa dipakai bersamaan
karena coil 0 dipakai dua-duanya.

## Isi folder

| File | Fungsi |
|---|---|
| `ir_modbus_test.ino` | firmware tes: IR → coil 2, conveyor ← coil 0 |
| `secrets.h` | SSID/password WiFi (sudah terisi, masuk `.gitignore`) |
| `secrets.example.h` | contoh kalau mau isi manual |
| `test_modbus.py` | cek Modbus langsung dari laptop (stdlib, tanpa pip) |

## Langkah

1. Upload `ir_modbus_test.ino` ke ESP8266.
2. Buka Serial Monitor **115200**. Catat baris `WiFi OK. IP ESP = ...`.
   - Kalau muncul `WiFi GAGAL`, cek `secrets.h`.
3. Uji dulu tanpa OpenPLC:

   ```bash
   python3 test_modbus.py <IP_ESP>
   ```

   Harus keluar `coil 2 (IR) = ...`. Tutup/geser benda di depan sensor →
   nilai berubah 0/1, LED onboard ESP juga ikut menyala.

4. Di OpenPLC Editor → Remote Device `esp`:
   - host = IP ESP di atas,
   - grup IR: **functionCode 1, offset 2**.
5. Compile & Run ulang (config lama di runtime masih offset 0).
6. `%IX0.0` sekarang mengikuti sensor.

## Kalau masih false

- `python3 test_modbus.py <IP> --watch` → kalau di sini **sudah berubah** tapi
  OpenPLC tetap false, masalahnya di sisi OpenPLC (host/FC/offset, atau project
  belum di-compile/upload ulang).
- Kalau `python3` kena `Connection refused/timeout` → ESP belum dapat IP,
  IP salah, atau beda jaringan. Pastikan laptop & ESP di WiFi yang sama.
- Kalau nilai tidak pernah berubah walau sensor detect: cek `[STATUS] pin=...`
  di Serial. Kalau `pin` tetap HIGH/LOW terus padahal sensor detect, pindahkan
  OUT sensor ke **D5 (GPIO14)** atau **D6 (GPIO12)** dan sesuaikan `PIN_IR`
  (GPIO16/D0 tidak punya pull-up internal, dan sinyal 5V sebaiknya diberi
  pembagi/level shifter — lebih aman beri daya sensor dari 3.3V).
