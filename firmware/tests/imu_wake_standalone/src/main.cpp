#include <Arduino.h>
#include <Wire.h>
#include "pins.h"

#define IMU_ADDR 0x68

static uint8_t read_reg(uint8_t reg) {
  Wire1.beginTransmission(IMU_ADDR);
  Wire1.write(reg);
  if (Wire1.endTransmission(true) != 0) return 0xFF;
  if (Wire1.requestFrom((uint16_t)IMU_ADDR, (uint8_t)1, (uint8_t)1) != 1) return 0xFF;
  return Wire1.read();
}

static bool write_reg(uint8_t reg, uint8_t value) {
  Wire1.beginTransmission(IMU_ADDR);
  Wire1.write(reg);
  Wire1.write(value);
  return Wire1.endTransmission(true) == 0;
}

static int16_t read_word(uint8_t reg_high) {
  Wire1.beginTransmission(IMU_ADDR);
  Wire1.write(reg_high);
  if (Wire1.endTransmission(true) != 0) return 0;
  if (Wire1.requestFrom((uint16_t)IMU_ADDR, (uint8_t)2, (uint8_t)1) != 2) return 0;
  int16_t v = (Wire1.read() << 8) | Wire1.read();
  return v;
}

static bool probe() {
  Wire1.beginTransmission(IMU_ADDR);
  return Wire1.endTransmission(true) == 0;
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\n=== IMU continuous check ===");
  Serial.printf("SDA=%d SCL=%d\n", PIN_IMU_SDA, PIN_IMU_SCL);

  if (!Wire1.setPins(PIN_IMU_SDA, PIN_IMU_SCL)) {
    Serial.println("Wire1.setPins FAILED");
  }
  Wire1.begin();
  Wire1.setClock(100000);
  Wire1.setTimeOut(200);
  delay(100);
}

void loop() {
  bool present = probe();
  uint8_t who = present ? read_reg(0x75) : 0xFF;
  uint8_t pwr = present ? read_reg(0x6B) : 0xFF;

  // Try to wake: write 0x00 to PWR_MGMT_1.
  bool wok = false;
  if (present) {
    wok = write_reg(0x6B, 0x00);
    delay(50);
  }
  uint8_t pwr2 = present ? read_reg(0x6B) : 0xFF;

  // Read motion data.
  int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
  if (present && (pwr2 & 0x40) == 0) {
    ax = read_word(0x3B);
    ay = read_word(0x3D);
    az = read_word(0x3F);
    gx = read_word(0x43);
    gy = read_word(0x45);
    gz = read_word(0x47);
  }

  Serial.printf(
    "probe=%u id=0x%02X pwr1=0x%02X wake_write=%u pwr2=0x%02X "
    "sleep=%u ax=%6d ay=%6d az=%6d gx=%6d gy=%6d gz=%6d\n",
    present, who, pwr, wok, pwr2, (pwr2 & 0x40) ? 1 : 0,
    ax, ay, az, gx, gy, gz);

  delay(1000);
}
