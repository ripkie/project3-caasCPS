#include <Arduino.h>
#include <Wire.h>
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "model-parameters/model_metadata.h"

// =========================
// KONFIGURASI
// =========================
#define SDA_PIN 8
#define SCL_PIN 9
#define MPU_ADDR 0x68

#define SAMPLE_INTERVAL_MS  20      // 50 Hz
#define FALL_COOLDOWN_MS    5000    // 5 detik (bisa diubah)
#define FALL_THRESHOLD      0.85    // Naikkan jadi 0.88 atau 0.90 kalau masih sering false positive

// Buffer Edge Impulse
static float input_buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = {0};
static size_t input_ix = 0;

// Cooldown
unsigned long lastFallTime = 0;
bool inCooldown = false;

// =========================
// FUNGSI I2C MPU6050
// =========================
void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 1);
  return Wire.available() ? Wire.read() : 0xFF;
}

void readAccelRaw(int16_t &ax, int16_t &ay, int16_t &az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6);
  if (Wire.available() >= 6) {
    ax = (Wire.read() << 8) | Wire.read();
    ay = (Wire.read() << 8) | Wire.read();
    az = (Wire.read() << 8) | Wire.read();
  }
}

// =========================
// INIT MPU6050
// =========================
bool initMPU() {
  uint8_t whoami = readRegister(0x75);
  Serial.printf("WHO_AM_I = 0x%02X\n", whoami);
  
  if (whoami != 0x68 && whoami != 0x70 && whoami != 0x71 && whoami != 0x72 && whoami != 0x73) {
    Serial.println("MPU6050 tidak terdeteksi!");
    return false;
  }
  writeRegister(0x6B, 0x00);
  delay(100);
  writeRegister(0x1C, 0x00);
  delay(50);
  Serial.println("MPU6050 initialized.");
  return true;
}

// =========================
// SIGNAL CALLBACK
// =========================
static int get_signal_data(size_t offset, size_t length, float *out_ptr) {
  memcpy(out_ptr, input_buffer + offset, length * sizeof(float));
  return 0;
}

// =========================
// RUN INFERENCE
// =========================
void runInference() {
  signal_t signal;
  signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
  signal.get_data = &get_signal_data;

  ei_impulse_result_t result = {0};
  EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);

  if (res != EI_IMPULSE_OK) {
    Serial.printf("Classifier error: %d\n", res);
    return;
  }

  Serial.println("\n=== HASIL INFERENSI ===");
  float fallConfidence = 0.0f;
  const char* bestLabel = "Unknown";
  float bestValue = 0.0f;

  for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    const char* label = result.classification[i].label;
    float value = result.classification[i].value;
    Serial.printf("%s: %.5f\n", label, value);

    if (value > bestValue) {
      bestValue = value;
      bestLabel = label;
    }
    if (strstr(label, "fall") || strstr(label, "Fall") || strstr(label, "jatuh")) {
      fallConfidence = value;   // simpan nilai Fall
    }
  }

  Serial.printf("Prediksi: %s (%.1f%%)\n", bestLabel, bestValue * 100);
  Serial.printf("Fall confidence: %.1f%%\n", fallConfidence * 100);

  // ==================== TRIGGER COOLDOWN HANYA SAAT FALL BENAR-BENAR TINGGI ====================
  if (fallConfidence >= FALL_THRESHOLD) {
    Serial.println("🚨 FALL DETECTED! Ambil tindakan...");
    lastFallTime = millis();
    inCooldown = true;
  }

  Serial.println("======================");
}

// =========================
// SETUP
// =========================
void setup() {
  Serial.begin(115200);
  delay(2000);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  Serial.println("=== Wearable Fall Detection - Cooldown HANYA Saat Fall ===");

  if (!initMPU()) {
    while (1) delay(1000);
  }

  Serial.printf("Model input size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
  Serial.printf("Fall threshold: %.2f\n", FALL_THRESHOLD);
  Serial.println("Sistem siap.\n");
}

// =========================
// LOOP - Cooldown hanya saat Fall
// =========================
void loop() {
  unsigned long now = millis();

  // Cooldown check
  if (inCooldown) {
    if (now - lastFallTime >= FALL_COOLDOWN_MS) {
      inCooldown = false;
      Serial.println("Cooldown selesai → kembali normal...");
    } else {
      delay(SAMPLE_INTERVAL_MS);   // tetap sampling tapi tidak inference
      return;
    }
  }

  // Normal operation (selalu inference kecuali cooldown)
  static unsigned long lastSample = 0;
  if (now - lastSample < SAMPLE_INTERVAL_MS) return;
  lastSample = now;

  int16_t axRaw, ayRaw, azRaw;
  readAccelRaw(axRaw, ayRaw, azRaw);

  float ax = axRaw / 16384.0f;
  float ay = ayRaw / 16384.0f;
  float az = azRaw / 16384.0f;

  input_buffer[input_ix++] = ax;
  input_buffer[input_ix++] = ay;
  input_buffer[input_ix++] = az;

  if (input_ix >= EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
    runInference();
    input_ix = 0;
  }
}
