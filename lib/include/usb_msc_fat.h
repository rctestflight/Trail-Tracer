#pragma once

#include <Arduino.h>

// ─────────────────────────────────────────────────────────────────────────────
// DeviceConfig
//
// Holds all runtime-configurable device parameters.  On boot the values are
// loaded from NVS (Preferences) then overridden by config.json if the file has
// changed since the last reading.  When the host edits config.json over USB MSC
// and safely ejects the drive, the new values are parsed and this struct is
// updated.
// ─────────────────────────────────────────────────────────────────────────────
struct DeviceConfig {
    // Identification
    uint8_t vehicleID;
    uint8_t rx_channel;

    // Drive parameters
    int CRUISE_THROTTLE;
    int CREEP_THROTTLE;
    double throttle_expo;   // 0.0 = linear, 1.0 = strong exponential around center

    // Output 1 – front steering
    int PWM_OUT_1_DEFAULT;
    int PWM_OUT_1_MIN;
    int PWM_OUT_1_MAX;

    // Output 2 – throttle
    int PWM_OUT_2_DEFAULT;
    int PWM_OUT_2_MIN;
    int PWM_OUT_2_MAX;

    // Output 3 – rear steering
    int PWM_OUT_3_DEFAULT;
    int PWM_OUT_3_MIN;
    int PWM_OUT_3_MAX;

    // Output 4 – mirrored rear steering / aux
    int PWM_OUT_4_DEFAULT;
    int PWM_OUT_4_MIN;
    int PWM_OUT_4_MAX;

    // Front PID
    double kp;
    double ki;
    double kd;
    int    pid_direction;   // PID_v1: DIRECT=0, REVERSE=1
    bool   enable_sensor_2;

    // Rear PID
    double rear_kp;
    double rear_ki;
    double rear_kd;
    int    rear_pid_direction;

    // Steering options
    bool   reverse_rear_steer_in;
    bool   use_heading;
    double heading_weight;
    double sensor_spacing_mm;  // Distance between s1 and s2 sensors in mm

    // Stuck-vehicle detector
    float stuck_threshold_mm;
    float stuck_timeout_ms;   // milliseconds

    // BTx proximity
    uint8_t BTxFollowChannel;
    int     BTxThreshold;
    double  BTxAccelRate;
    int     BTxCreepDuration; // seconds

    // Sine swerve
    uint8_t SineSwerveAmplitude; // mm (0 = disabled)
    uint8_t SineSwerveFrequency; // seconds per cycle

    // Charging
    double CHARGE_VBATT_STOP;    // V
    double CHARGE_MAX_CURRENT;   // A
    int    CHARGE_MAX_SECONDS;

    // Features
    bool   espnow_enabled;

    // TPA (Throttle PID Attenuation)
    // 0.0 = no attenuation at max throttle, 1.0 = full attenuation at max throttle.
    // Scaling is applied linearly between PWM_OUT_2_DEFAULT and PWM_OUT_2_MAX.
    double tpa;
};

// ─────────────────────────────────────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Initialise the USB MSC device and FAT filesystem layer.
 *
 * Must be called from setup() BEFORE Serial.begin() (or at least before the
 * USB stack is started) so that the MSC descriptor is registered in time.
 *
 * On return, *cfg is populated from NVS (and config.json if it agrees with
 * the stored "last seen" state), ready to be applied to the firmware globals.
 *
 * @param cfg  Pointer to a DeviceConfig struct that will be filled with the
 *             boot-time configuration.
 */
void msc_fat_init(DeviceConfig* cfg);

/**
 * Maintenance function – call once per main loop iteration (or at least
 * frequently, e.g. every few ms).
 *
 * Detects when the USB host has stopped writing (3-second quiet window) and
 * then flushes the flash write-back cache, mounts the FAT filesystem, checks
 * whether config.json has changed and, if so, parses the file and updates *cfg.
 *
 * @param cfg  Pointer to the same DeviceConfig passed to msc_fat_init().
 * @return     true if *cfg was updated and the caller should re-apply it.
 */
bool msc_fat_loop(DeviceConfig* cfg);

/**
 * Persist the provided configuration to NVS immediately.
 *
 * This is intended for firmware-initiated setting changes (for example button
 * actions) so they survive reboot even when config.json is not edited by host.
 */
void msc_fat_save_nvs(const DeviceConfig* cfg);

/**
 * Persist the provided configuration to both NVS and config.json.
 *
 * Returns true when both saves succeed, false if JSON save fails.
 * NVS save is always attempted first.
 */
bool msc_fat_save_all(const DeviceConfig* cfg);
