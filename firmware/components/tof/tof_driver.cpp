/*
 * tof_driver.cpp — ToF Sensor Array Implementation
 *
 * BOOT SEQUENCE:
 *   1. Set all XSHUT pins LOW (all sensors off)
 *   2. Wake sensor 0 (XSHUT_0 HIGH), assign address 0x30
 *   3. Wake sensor 1 (XSHUT_1 HIGH), assign address 0x31
 *   4. Wake sensor 2 (XSHUT_2 HIGH), assign address 0x32
 *   5. Configure all: distance mode, timing budget
 *   6. Start continuous ranging on all sensors
 *
 * RUNTIME:
 *   tof_update() polls each sensor for new data (non-blocking).
 *   tof_get_json() formats the latest readings + servo angle as JSON.
 *
 * JSON FORMAT:
 *   {"servo_deg":90.0,"tof":[
 *     {"id":0,"angle":-30.0,"dist_mm":1234,"status":0},
 *     {"id":1,"angle":0.0,"dist_mm":2100,"status":0},
 *     {"id":2,"angle":30.0,"dist_mm":987,"status":0}
 *   ]}
 */

#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include "pins.h"
#include "tof_config.h"
#include "tof_driver.h"

// ── Per-sensor state ──────────────────────────────────────────────────────────
struct ToFSensor {
    VL53L1X       driver;       // Pololu library object
    uint8_t       address;      // Assigned I2C address
    uint8_t       xshut_pin;   // XSHUT GPIO
    float         mount_angle;  // Physical mounting angle (degrees)
    uint16_t      distance_mm;  // Latest reading
    uint8_t       status;       // 0=valid, other=error
    bool          initialized;  // Did init succeed?
};

// ── Sensor array ──────────────────────────────────────────────────────────────
static ToFSensor sensors[TOF_SENSOR_COUNT] = {
    { .driver = VL53L1X(), .address = TOF_ADDR_0, .xshut_pin = PIN_XSHUT_0,
      .mount_angle = TOF_ANGLE_0, .distance_mm = 0, .status = 255, .initialized = false },
    { .driver = VL53L1X(), .address = TOF_ADDR_1, .xshut_pin = PIN_XSHUT_1,
      .mount_angle = TOF_ANGLE_1, .distance_mm = 0, .status = 255, .initialized = false },
    { .driver = VL53L1X(), .address = TOF_ADDR_2, .xshut_pin = PIN_XSHUT_2,
      .mount_angle = TOF_ANGLE_2, .distance_mm = 0, .status = 255, .initialized = false },
};

// ── Timing ────────────────────────────────────────────────────────────────────
static unsigned long last_publish = 0;
static bool any_sensor_ok = false;

// ═══════════════════════════════════════════════════════════════════════════════
//  PUBLIC: tof_init()
// ═══════════════════════════════════════════════════════════════════════════════
bool tof_init() {
    // ── Step 1: Configure XSHUT pins — all sensors OFF ────────────────────────
    // OUTPUT LOW = sensor held in reset (off)
    // INPUT (floating) = sensor ON (internal pull-up on XSHUT brings it HIGH)
    for (int i = 0; i < TOF_SENSOR_COUNT; i++) {
        pinMode(sensors[i].xshut_pin, OUTPUT);
        digitalWrite(sensors[i].xshut_pin, LOW);  // All sensors OFF
    }
    delay(50);  // Let them fully power down

    // ── Step 2: Initialize I2C on the sensor bus ──────────────────────────────
    // Use 10kHz and a generous timeout during init. Three VL53L1X sensors
    // on long wires create heavy bus capacitance; a slower clock gives weak
    // open-drain drivers enough time to pull the lines low reliably.
    // The IMU is on a separate Wire1 bus, so it is not part of this scan.
    Wire.begin(PIN_TOF_SDA, PIN_TOF_SCL);
    // Very slow I2C during init: the shared bus has 4 slaves and long wires,
    // so rise time/capacitance can prevent the weaker MPU6050 from ACKing at
    // higher speeds. 10kHz is conservative but reliable.
    Wire.setClock(10000);
    Wire.setTimeOut(100);
    delay(50);

    // ── Step 3: Wake sensors one-by-one and assign unique addresses ───────────
    int success_count = 0;
    for (int i = 0; i < TOF_SENSOR_COUNT; i++) {
        // Release XSHUT for this sensor
        digitalWrite(sensors[i].xshut_pin, HIGH);
        delay(150);

        // Initialize at default address 0x29
        sensors[i].driver.setBus(&Wire);
        sensors[i].driver.setTimeout(500);

        if (!sensors[i].driver.init()) {
            Serial.printf("[TOF] Sensor %d FAILED to init\n", i);
            sensors[i].initialized = false;
            continue;
        }

        // Assign new unique address
        sensors[i].driver.setAddress(sensors[i].address);

        sensors[i].initialized = true;
        success_count++;
        Serial.printf("[TOF] Sensor %d OK → addr=0x%02X, angle=%.0f°\n",
                      i, sensors[i].address, sensors[i].mount_angle);
    }

    any_sensor_ok = (success_count > 0);
    Serial.printf("[TOF] %d/%d sensors addressed\n", success_count, TOF_SENSOR_COUNT);

    last_publish = millis();
    return any_sensor_ok;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  PUBLIC: tof_start() — start continuous ranging after all I2C slaves are ready
// ═══════════════════════════════════════════════════════════════════════════════
void tof_start() {
    if (!any_sensor_ok) {
        Serial.println("[TOF] start() skipped — no sensors initialized");
        return;
    }

    // Configure and start ranging on every initialized sensor.
    for (int i = 0; i < TOF_SENSOR_COUNT; i++) {
        if (!sensors[i].initialized) continue;

        sensors[i].driver.setDistanceMode((VL53L1X::DistanceMode)TOF_DISTANCE_MODE);
        sensors[i].driver.setMeasurementTimingBudget(TOF_TIMING_BUDGET_US);
        sensors[i].driver.startContinuous(TOF_INTER_MEAS_MS);
        sensors[i].driver.setTimeout(10);  // tight ranging timeout
    }

    // Lower I2C speed + timeout for robust runtime operation on the shared bus.
    // 10kHz is conservative for the multi-sensor, long-wire bus on this robot.
    Wire.setClock(10000);
    Wire.setTimeOut(10);

    Serial.println("[TOF] Continuous ranging started");
}

// ═══════════════════════════════════════════════════════════════════════════════
//  PUBLIC: tof_update() — poll sensors for new data (non-blocking)
// ═══════════════════════════════════════════════════════════════════════════════
void tof_update() {
    if (!any_sensor_ok) return;

    // Read only ONE sensor per update call (stagger reads to reduce bus load)
    static uint8_t current_sensor = 0;

    if (!sensors[current_sensor].initialized) {
        current_sensor = (current_sensor + 1) % TOF_SENSOR_COUNT;
        return;
    }

    if (sensors[current_sensor].driver.dataReady()) {
        uint16_t reading = sensors[current_sensor].driver.read(false);
        uint8_t status = sensors[current_sensor].driver.ranging_data.range_status;

        // Always record the latest status so diagnostics can see failures.
        // Only accept the distance if the reading is valid.
        sensors[current_sensor].status = status;
        if (status == 0 && reading > 0 && reading < 8000) {
            sensors[current_sensor].distance_mm = reading;
        }
    }

    current_sensor = (current_sensor + 1) % TOF_SENSOR_COUNT;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  PUBLIC: tof_json_ready() — check if it's time to publish
// ═══════════════════════════════════════════════════════════════════════════════
bool tof_json_ready() {
    if (!any_sensor_ok) return false;
    return (millis() - last_publish) >= TOF_PUBLISH_MS;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  PUBLIC: tof_get_json() — format JSON into buffer
// ═══════════════════════════════════════════════════════════════════════════════
int tof_get_json(char* buf, int buf_size, float servo_deg) {
    last_publish = millis();

    // Build JSON manually (no library needed, saves RAM)
    int len = snprintf(buf, buf_size,
        "{\"servo_deg\":%.1f,\"tof\":["
        "{\"id\":0,\"angle\":%.1f,\"dist_mm\":%u,\"status\":%u},"
        "{\"id\":1,\"angle\":%.1f,\"dist_mm\":%u,\"status\":%u},"
        "{\"id\":2,\"angle\":%.1f,\"dist_mm\":%u,\"status\":%u}"
        "]}",
        servo_deg,
        sensors[0].mount_angle, sensors[0].distance_mm, sensors[0].status,
        sensors[1].mount_angle, sensors[1].distance_mm, sensors[1].status,
        sensors[2].mount_angle, sensors[2].distance_mm, sensors[2].status
    );

    return len;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  PUBLIC: tof_handle_serial() — serial commands for this component
// ═══════════════════════════════════════════════════════════════════════════════
bool tof_handle_serial(const char* cmd) {
    // ── tof status — show sensor states ───────────────────────────────────────
    if (strcmp(cmd, "tof status") == 0) {
        Serial.println("┌─── ToF Status ─────────────────────────────────┐");
        for (int i = 0; i < TOF_SENSOR_COUNT; i++) {
            Serial.printf("│  Sensor %d: %s | addr=0x%02X | angle=%+.0f°\n",
                          i,
                          sensors[i].initialized ? "OK" : "FAIL",
                          sensors[i].address,
                          sensors[i].mount_angle);
            if (sensors[i].initialized) {
                Serial.printf("│            dist=%4umm | status=%u\n",
                              sensors[i].distance_mm, sensors[i].status);
            }
        }
        Serial.printf("│  Mode: %s | Budget: %dms | Publish: %dHz\n",
                      TOF_DISTANCE_MODE == 2 ? "Long(4m)" : "Short(1.3m)",
                      TOF_TIMING_BUDGET_US / 1000,
                      1000 / TOF_PUBLISH_MS);
        Serial.println("└────────────────────────────────────────────────┘");
        return true;
    }

    // ── tof read — single manual reading ──────────────────────────────────────
    if (strcmp(cmd, "tof read") == 0) {
        for (int i = 0; i < TOF_SENSOR_COUNT; i++) {
            if (sensors[i].initialized) {
                Serial.printf("[TOF %d] %4umm (status=%u)\n",
                              i, sensors[i].distance_mm, sensors[i].status);
            } else {
                Serial.printf("[TOF %d] NOT INITIALIZED\n", i);
            }
        }
        return true;
    }

    // ── tof json — print one JSON frame now ───────────────────────────────────
    if (strcmp(cmd, "tof json") == 0) {
        char json_buf[256];
        tof_get_json(json_buf, sizeof(json_buf), 0.0f);  // 0 servo for manual test
        Serial.println(json_buf);
        return true;
    }

    return false;  // Not our command
}

// ═══════════════════════════════════════════════════════════════════════════════
//  PUBLIC: tof_print_help()
// ═══════════════════════════════════════════════════════════════════════════════
void tof_print_help() {
    Serial.println("  tof status    Show sensor states and config");
    Serial.println("  tof read      Print current distances");
    Serial.println("  tof json      Print one JSON frame");
}

// ═══════════════════════════════════════════════════════════════════════════════
//  PUBLIC: tof_get_distance() / tof_get_status() — getters for ROS bridge
// ═══════════════════════════════════════════════════════════════════════════════
uint16_t tof_get_distance(uint8_t sensor_id) {
    if (sensor_id >= TOF_SENSOR_COUNT) return 0;
    if (!sensors[sensor_id].initialized) return 0;
    return sensors[sensor_id].distance_mm;
}

uint8_t tof_get_status(uint8_t sensor_id) {
    if (sensor_id >= TOF_SENSOR_COUNT) return 255;
    if (!sensors[sensor_id].initialized) return 255;
    return sensors[sensor_id].status;
}
