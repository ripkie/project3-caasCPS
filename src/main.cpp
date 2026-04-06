#include <Arduino.h>
#include <Wire.h>

// KONFIGURASI PIN
#define SDA_PIN 17
#define SCL_PIN 18

#define MPU_ADDR 0x68 // karena AD0 ke GND

// REGISTER MPU
#define REG_PWR_MGMT_1 0x6B
#define REG_ACCEL_XOUT_H 0x3B
#define REG_ACCEL_CONFIG 0x1C
#define REG_WHO_AM_I 0x75

// LABEL DATASET
String currentLabel = "Idle";
int currentLabelID = 0;

// Sampling
unsigned long lastSampleTime = 0;
const int sampleIntervalMs = 50; // 20 Hz

bool loggingEnabled = true;

// I2C FUNCTIONS
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

// INIT MPU
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

  writeRegister(REG_ACCEL_CONFIG, 0x00); // ±2g
  delay(50);

  return true;
}

// SETUP
void setup()
{
  Serial.begin(115200);
  delay(1500);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  Serial.println("=== DATASET LOGGER ESP32-S3 ===");
  Serial.println("Perintah:");
  Serial.println("i = Idle");
  Serial.println("w = Walking");
  Serial.println("f = Fall");
  Serial.println("s = Start/Stop");
  Serial.println();

  if (!initMPU())
  {
    while (1)
      delay(1000);
  }

  // Header CSV
  Serial.println("timestamp_ms,label_id,label,ax_raw,ay_raw,az_raw,ax_g,ay_g,az_g");
}

// LOOP
void loop()
{
  // INPUT COMMAND
  if (Serial.available())
  {
    char cmd = Serial.read();

    if (cmd == 'i')
    {
      currentLabel = "Idle";
      currentLabelID = 0;
      Serial.println("# Idle");
    }
    else if (cmd == 'w')
    {
      currentLabel = "Walking";
      currentLabelID = 1;
      Serial.println("# Walking");
    }
    else if (cmd == 'f')
    {
      currentLabel = "Fall";
      currentLabelID = 2;
      Serial.println("# Fall");
    }
    else if (cmd == 's')
    {
      loggingEnabled = !loggingEnabled;
      Serial.println(loggingEnabled ? "# Logging ON" : "# Logging OFF");
    }
  }

  if (!loggingEnabled)
    return;

  // SAMPLING DATA
  unsigned long now = millis();
  if (now - lastSampleTime >= sampleIntervalMs)
  {
    lastSampleTime = now;

    int16_t axRaw = 0, ayRaw = 0, azRaw = 0;
    readAccelRaw(axRaw, ayRaw, azRaw);

    // konversi ke g
    float axG = axRaw / 16384.0;
    float ayG = ayRaw / 16384.0;
    float azG = azRaw / 16384.0;

    // OUTPUT CSV
    Serial.print(now);
    Serial.print(",");
    Serial.print(currentLabelID);
    Serial.print(",");
    Serial.print(currentLabel);
    Serial.print(",");
    Serial.print(axRaw);
    Serial.print(",");
    Serial.print(ayRaw);
    Serial.print(",");
    Serial.print(azRaw);
    Serial.print(",");
    Serial.print(axG, 4);
    Serial.print(",");
    Serial.print(ayG, 4);
    Serial.print(",");
    Serial.println(azG, 4);
  }
}
