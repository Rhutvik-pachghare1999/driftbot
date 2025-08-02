/*
 * DriftBot Pin Diagnostic
 *
 * Safe, non-destructive GPIO / I2C probe for the ESP32-S3 DevKitC-1.
 *
 * What it does:
 *   1. Prints chip info (MAC, flash, PSRAM).
 *   2. Reads every user-accessible GPIO as INPUT_PULLUP and prints its level.
 *      This shows which pins are externally pulled high/low or floating.
 *   3. Scans the I2C bus on candidate SDA/SCL pairs to find sensors.
 *   4. If a VL53L1X or MPU6050 is found, tries a basic init to confirm it talks.
 *
 * Pins NOT touched (strapping / USB / UART / flash):
 *   0, 3, 19, 20, 43, 44, 45, 46
 *
 * Build/flash:
 *   cd firmware/tests/pin_diag
 *   pio run -t upload
 *   pio device monitor
 */

#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <VL53L1X.h>

// ──────────────────────────────────────────────────────────────────────────────
// Pin lists
// ──────────────────────────────────────────────────────────────────────────────

static const uint8_t gpios_to_probe[] = {
    // Left / default GPIOs (skip 0, 3)
    1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18,
    // Common extra GPIOs on DevKitC-1
    33, 34, 35, 36, 37, 38, 39, 40, 41, 42,
    // LED / end
    47, 48
};

struct I2cCandidate {
    const char* name;
    uint8_t sda;
    uint8_t scl;
};

static const I2cCandidate i2c_candidates[] = {
    { "GPIO 8/9  (ToF bus)", 8, 9 },
    { "GPIO 17/21 (spare)", 17, 21 },
};

// ──────────────────────────────────────────────────────────────────────────────
// Helpers
// ──────────────────────────────────────────────────────────────────────────────

static void print_chip_info() {
    Serial.println("\n═══════════════════════════════════════════════════");
    Serial.println("  DriftBot Pin Diagnostic");
    Serial.println("═══════════════════════════════════════════════════");
    Serial.printf("  Chip model:    %s\n", ESP.getChipModel());
    Serial.printf("  Chip revision: %d\n", ESP.getChipRevision());
    Serial.printf("  CPU freq:      %u MHz\n", getCpuFrequencyMhz());
    Serial.printf("  Flash size:    %d MB\n", ESP.getFlashChipSize() / (1024 * 1024));
    Serial.printf("  PSRAM size:    %d MB\n", ESP.getPsramSize() / (1024 * 1024));
    Serial.printf("  MAC:           %s\n", WiFi.macAddress().c_str());
    Serial.println("═══════════════════════════════════════════════════\n");
}

static void gpio_probe() {
    Serial.println("\n─── GPIO INPUT_PULLUP levels ───────────────────────");
    Serial.println("pin | level   note");
    Serial.println("────┼───────────────────────────────────────────────");
    for (uint8_t pin : gpios_to_probe) {
        pinMode(pin, INPUT_PULLUP);
        delayMicroseconds(50);          // let pull-up settle
        int level = digitalRead(pin);

        const char* note = "";
        if (pin == 1 || pin == 2)       note = "possible UART0 / boot";
        else if (pin == 4 || pin == 5 || pin == 6) note = "ToF XSHUT in main firmware";
        else if (pin == 7)              note = "IMU SDA (Wire1)";
        else if (pin == 8 || pin == 9)  note = "ToF I2C (Wire)";
        else if (pin == 10)             note = "steering servo";
        else if (pin == 11 || pin == 12)note = "motor PWM";
        else if (pin == 13 || pin == 14)note = "wheel encoders";
        else if (pin == 15 || pin == 16)note = "motor enable";
        else if (pin == 17)             note = "IMU SCL (Wire1)";
        else if (pin == 21)             note = "spare I2C";
        else if (pin == 18)             note = "scan servo";
        else if (pin == 48)             note = "RGB LED";
        else if (pin >= 33 && pin <= 42)note = "SPI/PSRAM region — use with care";

        Serial.printf(" %2u | %s  %s\n", pin, level ? "HIGH" : "LOW ", note);
    }
    Serial.println("────────────────────────────────────────────────────\n");
}

static void i2c_scan_pair(uint8_t sda, uint8_t scl) {
    Serial.printf("\n─── I2C scan: SDA=GPIO%u SCL=GPIO%u ─────────────────\n", sda, scl);

    // Release the previous bus config (if any) and install on new pins
    Wire.end();
    delay(50);
    if (!Wire.begin(sda, scl)) {
        Serial.println("[FAIL] Wire.begin() failed on this pair");
        return;
    }
    Wire.setClock(100000);   // 100 kHz — conservative
    Wire.setTimeOut(50);     // 50 ms per transaction
    delay(100);              // let slaves settle

    bool any = false;
    uint8_t first_dev = 0;

    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        uint8_t err = Wire.endTransmission();
        if (err == 0) {
            Serial.printf("  device found at 0x%02X\n", addr);
            any = true;
            if (!first_dev) first_dev = addr;
        } else if (addr == 0x29 || addr == 0x68 || addr == 0x69) {
            // Always log the error for addresses we care about
            Serial.printf("  probe 0x%02X error=%d\n", addr, err);
        }
    }

    if (!any) {
        Serial.println("  no devices found on this pair");
        return;
    }

    // Try a real sensor init if the expected addresses show up
    if (first_dev == 0x29) {
        VL53L1X sensor;
        sensor.setBus(&Wire);
        sensor.setTimeout(500);
        Serial.println("  Attempting VL53L1X::init() at 0x29...");
        if (sensor.init()) {
            Serial.println("  [OK] VL53L1X responded and initialized");
        } else {
            Serial.println("  [FAIL] VL53L1X detected but init() returned false");
        }
    }

    if (first_dev == 0x68 || first_dev == 0x69) {
        uint8_t addr = first_dev;
        Wire.beginTransmission(addr);
        Wire.write(0x75);                 // WHO_AM_I
        uint8_t tx_err = Wire.endTransmission(false);
        uint8_t n = Wire.requestFrom(addr, (uint8_t)1);
        uint8_t who = 0;
        if (Wire.available()) who = Wire.read();
        Serial.printf("  MPU6050 WHO_AM_I probe: tx_err=%d rx_count=%d who=0x%02X\n",
                      tx_err, n, who);
        if (tx_err == 0 && n == 1 && (who == 0x68 || who == 0x98)) {
            Serial.println("  [OK] MPU6050 responded");
        } else {
            Serial.println("  [FAIL] MPU6050 did not respond as expected");
        }
    }
}

static void i2c_probe() {
    Serial.println("\n─── Candidate I2C bus probes ──────────────────────");
    for (const auto& c : i2c_candidates) {
        Serial.printf("\n>>> %s\n", c.name);
        i2c_scan_pair(c.sda, c.scl);
    }
    Serial.println("\n────────────────────────────────────────────────────\n");
}

// ──────────────────────────────────────────────────────────────────────────────
// Arduino entry points
// ──────────────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    delay(3000);        // Wait for terminal / USB-UART to connect

    print_chip_info();
    gpio_probe();
    i2c_probe();

    Serial.println("\n[DIAG] Done. Halting so outputs stay stable.");
    Serial.flush();
}

void loop() {
    // Nothing to do — diagnostic is one-shot.
    delay(1000);
}
