#include <Arduino.h>
#include <Wire.h>
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "model-parameters/model_metadata.h"

// === KONFIGURASI ===
#define SDA_PIN 8
#define SCL_PIN 9
#define MPU_ADDR 0x68
#define SAMPLE_INTERVAL_MS 20 // 50Hz
#define FALL_COOLDOWN_MS 2000
#define FALL_THRESHOLD 0.85f // Confidence minimum untuk deteksi fall

// === BUFFER & STATE ===
static float input_buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = {0};
static size_t input_ix = 0;
static unsigned long lastFallTime = 0;
static unsigned long lastSampleTime = 0;
static bool inCooldown = false;

// === MPU6050 ===
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

static bool readAccel(float &ax, float &ay, float &az)
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)6);

  if (Wire.available() < 6)
    return false;

  // Baca raw 16-bit lalu konversi ke satuan g (range ±2g = 16384 LSB/g)
  ax = (int16_t)((Wire.read() << 8) | Wire.read()) / 16384.0f;
  ay = (int16_t)((Wire.read() << 8) | Wire.read()) / 16384.0f;
  az = (int16_t)((Wire.read() << 8) | Wire.read()) / 16384.0f;
  return true;
}

static bool initMPU()
{
  uint8_t whoami = readRegister(0x75);
  Serial.printf("[MPU] WHO_AM_I = 0x%02X\n", whoami);

  if (whoami != 0x68 && whoami != 0x70 &&
      whoami != 0x71 && whoami != 0x72 && whoami != 0x73)
  {
    Serial.println("[MPU] ERROR: Sensor tidak terdeteksi!");
    return false;
  }

  writeRegister(0x6B, 0x00); // Wake up
  delay(100);
  writeRegister(0x1C, 0x00); // Accel range ±2g
  delay(50);

  Serial.println("[MPU] OK");
  return true;
}

// === EDGE IMPULSE ===
static int get_signal_data(size_t offset, size_t length, float *out_ptr)
{
  memcpy(out_ptr, input_buffer + offset, length * sizeof(float));
  return EIDSP_OK;
}

static void runInference()
{
  signal_t signal;
  signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
  signal.get_data = &get_signal_data;

  ei_impulse_result_t result = {0};
  EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);

  if (err != EI_IMPULSE_OK)
  {
    Serial.printf("[EI] Error: %d\n", err);
    return;
  }

  Serial.println("\n=== INFERENSI ===");
  float fallConfidence = 0.0f;
  const char *bestLabel = "Unknown";
  float bestValue = 0.0f;

  for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++)
  {
    const char *label = result.classification[i].label;
    float value = result.classification[i].value;
    Serial.printf("  %-12s : %.1f%%\n", label, value * 100.0f);

    if (value > bestValue)
    {
      bestValue = value;
      bestLabel = label;
    }
    if (strstr(label, "fall") || strstr(label, "Fall") || strstr(label, "jatuh"))
    {
      fallConfidence = value;
    }
  }

  Serial.printf("  Prediksi : %s (%.1f%%)\n", bestLabel, bestValue * 100.0f);

  if (fallConfidence >= FALL_THRESHOLD)
  {
    Serial.println("  !! FALL DETECTED !!");
    lastFallTime = millis();
    inCooldown = true;
  }
  Serial.println("=================\n");
}

// === SETUP ===
void setup()
{
  Serial.begin(115200);
  delay(2000);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  Serial.println("=== Fall Detection ESP32-S3 ===");
  Serial.printf("Threshold : %.2f | Cooldown : %dms\n\n", FALL_THRESHOLD, FALL_COOLDOWN_MS);

  if (!initMPU())
  {
    Serial.println("[SYS] HALT: Cek koneksi MPU6050!");
    while (true)
      delay(1000);
  }

  Serial.println("[SYS] Siap monitoring...\n");
}

// === LOOP ===
void loop()
{
  unsigned long now = millis();

  // Cooldown setelah fall terdeteksi
  if (inCooldown)
  {
    if (now - lastFallTime >= FALL_COOLDOWN_MS)
    {
      inCooldown = false;
      Serial.println("[SYS] Cooldown selesai\n");
    }
    else
    {
      delay(SAMPLE_INTERVAL_MS);
      return;
    }
  }

  // Sampling 50Hz
  if (now - lastSampleTime < SAMPLE_INTERVAL_MS)
    return;
  lastSampleTime = now;

  // Baca sensor
  float ax, ay, az;
  if (!readAccel(ax, ay, az))
  {
    Serial.println("[MPU] Gagal baca!");
    return;
  }

  // Isi buffer
  input_buffer[input_ix++] = ax;
  input_buffer[input_ix++] = ay;
  input_buffer[input_ix++] = az;

  // Jalankan inferensi jika buffer penuh
  if (input_ix >= EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE)
  {
    runInference();
    input_ix = 0;
  }
}