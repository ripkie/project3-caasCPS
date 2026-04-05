#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "model-parameters/model_metadata.h"
#include "model-parameters/model_variables.h"

// =========================
// KONFIGURASI PIN
// =========================
#define SDA_PIN 17
#define SCL_PIN 18
#define MPU_ADDR 0x68

// =========================
// REGISTER MPU
// =========================
#define REG_PWR_MGMT_1 0x6B
#define REG_ACCEL_XOUT_H 0x3B
#define REG_ACCEL_CONFIG 0x1C
#define REG_WHO_AM_I 0x75

// =========================
// PARAMETER INFERENSI
// =========================
static const uint32_t SAMPLE_INTERVAL_MS = 50; // 20 Hz
static const size_t AXIS_COUNT = 3;
static const size_t SAMPLE_COUNT = 10; // 500 ms @ 20 Hz
static const size_t INPUT_FRAME_SIZE = SAMPLE_COUNT * AXIS_COUNT;

float input_buffer[INPUT_FRAME_SIZE];
size_t input_ix = 0;
unsigned long lastSampleTime = 0;

// =========================
// I2C FUNCTIONS
// =========================
void writeRegister(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t readRegister(uint8_t reg)
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_ADDR, 1);
  if (Wire.available())
  {
    return Wire.read();
  }
  return 0xFF;
}

void readAccelRaw(int16_t &ax, int16_t &ay, int16_t &az)
{
  ax = ay = az = 0;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_ADDR, 6);
  if (Wire.available() >= 6)
  {
    ax = (Wire.read() << 8) | Wire.read();
    ay = (Wire.read() << 8) | Wire.read();
    az = (Wire.read() << 8) | Wire.read();
  }
}

// =========================
// INIT MPU
// =========================
bool initMPU()
{
  uint8_t whoami = readRegister(REG_WHO_AM_I);
  Serial.print("WHO_AM_I = 0x");
  Serial.println(whoami, HEX);

  if (whoami != 0x70 && whoami != 0x71 && whoami != 0x73)
  {
    Serial.println("MPU tidak terdeteksi!");
    return false;
  }

  writeRegister(REG_PWR_MGMT_1, 0x00); // wake up
  delay(100);

  writeRegister(REG_ACCEL_CONFIG, 0x00); // +-2g
  delay(50);

  return true;
}

// =========================
// EDGE IMPULSE SIGNAL
// =========================
static int get_signal_data(size_t offset, size_t length, float *out_ptr)
{
  if ((offset + length) > INPUT_FRAME_SIZE)
  {
    return -1;
  }

  memcpy(out_ptr, input_buffer + offset, length * sizeof(float));
  return 0;
}

// =========================
// RUN INFERENCE
// =========================
void runInference()
{
  signal_t signal;
  signal.total_length = INPUT_FRAME_SIZE;
  signal.get_data = get_signal_data;

  ei_impulse_result_t result = {0};

  EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);
  if (res != EI_IMPULSE_OK)
  {
    Serial.print("run_classifier error: ");
    Serial.println((int)res);
    return;
  }

  float bestValue = 0.0f;
  const char *bestLabel = "Unknown";

  Serial.println("=== HASIL INFERENSI ===");
  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++)
  {
    const char *label = result.classification[ix].label;
    float value = result.classification[ix].value;

    Serial.print(label);
    Serial.print(": ");
    Serial.println(value, 5);

    if (value > bestValue)
    {
      bestValue = value;
      bestLabel = label;
    }
  }

  Serial.print("Prediksi: ");
  if (bestValue >= 0.60f)
  {
    Serial.print(bestLabel);
  }
  else
  {
    Serial.print("Unknown");
  }
  Serial.print(" (confidence=");
  Serial.print(bestValue, 5);
  Serial.println(")");
  Serial.println("======================");
}

// =========================
// SETUP
// =========================
void setup()
{
  Serial.begin(115200);
  delay(1500);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  Serial.println("=== ESP32-S3 EDGE IMPULSE INFERENCE ===");

  if (!initMPU())
  {
    while (1)
    {
      delay(1000);
    }
  }

  Serial.print("EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE = ");
  Serial.println(EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);

  if (EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE != INPUT_FRAME_SIZE)
  {
    Serial.println("Ukuran input model tidak cocok!");
    Serial.println("Cek lagi window size, frequency, atau jumlah axis di Edge Impulse.");
    while (1)
    {
      delay(1000);
    }
  }

  Serial.println("Sistem siap.");
}

// =========================
// LOOP
// =========================
void loop()
{
  unsigned long now = millis();
  if (now - lastSampleTime < SAMPLE_INTERVAL_MS)
  {
    return;
  }
  lastSampleTime = now;

  int16_t axRaw = 0, ayRaw = 0, azRaw = 0;
  readAccelRaw(axRaw, ayRaw, azRaw);

  float axG = axRaw / 16384.0f;
  float ayG = ayRaw / 16384.0f;
  float azG = azRaw / 16384.0f;

  input_buffer[input_ix++] = axG;
  input_buffer[input_ix++] = ayG;
  input_buffer[input_ix++] = azG;

  if (input_ix >= INPUT_FRAME_SIZE)
  {
    runInference();
    input_ix = 0;
  }
}