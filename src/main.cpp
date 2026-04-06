#include <Arduino.h>
#include <Wire.h>
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "model-parameters/model_metadata.h"

// =========================
// KONFIGURASI PIN
// =========================
#define SDA_PIN 8
#define SCL_PIN 9
#define MPU_ADDR 0x68

#define LED_PIN 2    // LED built-in ESP32-S3 (ganti sesuai board)
#define BUZZER_PIN 4 // Pin buzzer aktif (ganti sesuai wiring)

// =========================
// KONFIGURASI DETEKSI
// =========================
#define SAMPLE_INTERVAL_MS 20 // 50 Hz sampling
#define FALL_COOLDOWN_MS 3000 // Cooldown 5 detik setelah fall terdeteksi
#define FALL_THRESHOLD 0.85f  // Naikkan ke 0.88-0.90 jika masih false positive

// Durasi notifikasi buzzer & LED (ms)
#define BUZZER_BEEP_MS 200
#define BUZZER_PAUSE_MS 100
#define BUZZER_REPEAT 3 // Jumlah beep saat fall detected

// =========================
// BUFFER EDGE IMPULSE
// =========================
static float input_buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = {0};
static size_t input_ix = 0;

// =========================
// STATE VARIABLES
// =========================
static unsigned long lastFallTime = 0;
static unsigned long lastSampleTime = 0;
static bool inCooldown = false;

// =========================
// FUNGSI I2C MPU6050
// =========================
static void writeRegister(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

static uint8_t readRegister(uint8_t reg)
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

static bool readAccelRaw(float &ax, float &ay, float &az)
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)6);

  if (Wire.available() < 6)
    return false;

  int16_t rawX = (Wire.read() << 8) | Wire.read();
  int16_t rawY = (Wire.read() << 8) | Wire.read();
  int16_t rawZ = (Wire.read() << 8) | Wire.read();

  // Konversi ke g (±2g range → 16384 LSB/g)
  ax = rawX / 16384.0f;
  ay = rawY / 16384.0f;
  az = rawZ / 16384.0f;
  return true;
}

// INIT MPU6050
static bool initMPU()
{
  uint8_t whoami = readRegister(0x75);
  Serial.printf("[MPU] WHO_AM_I = 0x%02X\n", whoami);

  // WHO_AM_I valid untuk MPU6050/6500 series
  if (whoami != 0x68 && whoami != 0x70 &&
      whoami != 0x71 && whoami != 0x72 && whoami != 0x73)
  {
    Serial.println("[MPU] ERROR: Sensor tidak terdeteksi! Cek wiring SDA/SCL.");
    return false;
  }

  writeRegister(0x6B, 0x00); // Wake up
  delay(100);
  writeRegister(0x1C, 0x00); // Accel range ±2g
  delay(50);

  Serial.println("[MPU] Initialized OK.");
  return true;
}

// NOTIFIKASI FALL
static void triggerFallAlert()
{
  Serial.println("🚨 [ALERT] FALL DETECTED!");

  for (int i = 0; i < BUZZER_REPEAT; i++)
  {
    digitalWrite(LED_PIN, HIGH);
    digitalWrite(BUZZER_PIN, HIGH);
    delay(BUZZER_BEEP_MS);

    digitalWrite(LED_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
    delay(BUZZER_PAUSE_MS);
  }
}

// SIGNAL CALLBACK (Edge Impulse)
static int get_signal_data(size_t offset, size_t length, float *out_ptr)
{
  memcpy(out_ptr, input_buffer + offset, length * sizeof(float));
  return EIDSP_OK;
}

// RUN INFERENCE
static void runInference()
{
  signal_t signal;
  signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
  signal.get_data = &get_signal_data;

  ei_impulse_result_t result = {0};
  EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);

  if (err != EI_IMPULSE_OK)
  {
    Serial.printf("[EI] Classifier error: %d\n", err);
    return;
  }

  // --- Cetak semua label ---
  Serial.println("\n=== HASIL INFERENSI ===");
  float fallConfidence = 0.0f;
  const char *bestLabel = "Unknown";
  float bestValue = 0.0f;

  for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++)
  {
    const char *label = result.classification[i].label;
    float value = result.classification[i].value;
    Serial.printf("  %-12s : %.4f (%.1f%%)\n", label, value, value * 100.0f);

    if (value > bestValue)
    {
      bestValue = value;
      bestLabel = label;
    }

    // Deteksi label fall (case-insensitive workaround)
    if (strstr(label, "fall") || strstr(label, "Fall") || strstr(label, "jatuh"))
    {
      fallConfidence = value;
    }
  }

  Serial.printf("  → Prediksi: %s (%.1f%%)\n", bestLabel, bestValue * 100.0f);
  Serial.printf("  → Fall confidence: %.1f%%\n", fallConfidence * 100.0f);

  // --- Trigger jika fall confidence tinggi ---
  if (fallConfidence >= FALL_THRESHOLD)
  {
    triggerFallAlert();
    lastFallTime = millis();
    inCooldown = true;
  }

  Serial.println("=======================\n");
}

// SETUP
void setup()
{
  Serial.begin(115200);
  delay(2000);

  // Init pin output
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  // Init I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  Serial.println("============================================");
  Serial.println("  Wearable Fall Detection - ESP32-S3");
  Serial.println("============================================");
  Serial.printf("  Model input  : %d samples\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
  Serial.printf("  Sampling     : %d ms (%.0f Hz)\n", SAMPLE_INTERVAL_MS, 1000.0f / SAMPLE_INTERVAL_MS);
  Serial.printf("  Fall threshold: %.2f\n", FALL_THRESHOLD);
  Serial.printf("  Cooldown     : %d ms\n", FALL_COOLDOWN_MS);
  Serial.println("============================================\n");

  if (!initMPU())
  {
    // Blink cepat tanda error, tidak lanjut
    while (true)
    {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(200);
    }
  }

  // Indikator siap: LED nyala 1 detik
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);

  Serial.println("[SYS] Sistem siap. Mulai monitoring...\n");
}

// LOOP
void loop()
{
  unsigned long now = millis();

  // --- Cooldown: tunggu dulu, tidak inference ---
  if (inCooldown)
  {
    unsigned long elapsed = now - lastFallTime;
    if (elapsed >= FALL_COOLDOWN_MS)
    {
      inCooldown = false;
      digitalWrite(LED_PIN, LOW);
      Serial.println("[SYS] Cooldown selesai → kembali monitoring...\n");
    }
    else
    {
      // LED berkedip pelan selama cooldown
      digitalWrite(LED_PIN, (elapsed / 500) % 2);
      delay(SAMPLE_INTERVAL_MS);
      return;
    }
  }

  // --- Sampling pada interval tetap ---
  if (now - lastSampleTime < SAMPLE_INTERVAL_MS)
    return;
  lastSampleTime = now;

  // --- Baca akselerometer ---
  float ax, ay, az;
  if (!readAccelRaw(ax, ay, az))
  {
    Serial.println("[MPU] Gagal baca data!");
    return;
  }

  // --- Isi buffer ---
  input_buffer[input_ix++] = ax;
  input_buffer[input_ix++] = ay;
  input_buffer[input_ix++] = az;

  // --- Jalankan inferensi jika buffer penuh ---
  if (input_ix >= EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE)
  {
    runInference();
    input_ix = 0;
  }
}