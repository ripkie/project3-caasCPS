## 📱 Deskripsi Device

Sistem ini adalah prototipe **wearable fall detection** berbasis Edge AI yang dirancang untuk mendeteksi kejadian jatuh (*fall*) secara real-time langsung di perangkat (*on-device inference*), tanpa memerlukan koneksi internet atau pemrosesan di cloud.

Perangkat terdiri dari **ESP32-S3** sebagai mikrokontroler utama yang menjalankan model TinyML, dan **MPU-9250/6500** sebagai sensor gerak yang membaca data akselerasi tubuh pada 3 sumbu (X, Y, Z) dengan frekuensi 20Hz. Data akselerasi dikumpulkan dalam sliding window selama 2 detik, kemudian diproses oleh model Neural Network untuk mengklasifikasikan aktivitas pengguna.

Sistem ini cocok digunakan sebagai dasar pengembangan perangkat keselamatan untuk lansia, pasien rehabilitasi, atau pekerja di lingkungan berisiko tinggi.

---

## ✨ Fitur

### 🔍 Klasifikasi Aktivitas Real-Time
Sistem mampu mengenali 3 jenis aktivitas secara real-time:
- **Idle** — pengguna diam, duduk, atau berdiri tanpa bergerak
- **Walking** — pengguna berjalan normal
- **Fall** — pengguna jatuh atau melakukan gerakan jatuh

### ⚡ Inferensi 100% Offline
Seluruh proses pengambilan keputusan (*inference*) berjalan di dalam ESP32-S3 tanpa mengirim data ke server atau cloud. Ini menjamin:
- Privasi data pengguna
- Latensi rendah (tidak tergantung koneksi internet)
- Dapat bekerja di area tanpa sinyal

### 🚨 Fall Detection dengan Cooldown
Ketika fall terdeteksi dengan confidence di atas threshold (`0.85` atau 85%), sistem akan:
1. Menampilkan peringatan `FALL DETECTED!` di Serial Monitor
2. Memasuki mode **cooldown selama 5 detik** untuk mencegah alarm berulang
3. Menampilkan countdown cooldown setiap detik
4. Kembali aktif secara otomatis setelah cooldown selesai

### 📊 Confidence Score per Kelas
Setiap siklus inferensi menampilkan skor kepercayaan untuk semua kelas secara bersamaan, sehingga memudahkan debugging dan evaluasi performa model:
