// ─────────────────────────────────────────────────────────────────────────────
// usb_msc_fat.cpp
//
// USB Mass Storage Class device backed by a FAT filesystem in internal flash.
//
// Architecture:
//  - A dedicated flash partition labelled "storage" (1 MB, type=data/fat)
//    provides the raw block storage exposed over USB MSC with 512-byte sectors.
//  - A write-back erase-block cache absorbs the impedance mismatch between the
//    512-byte FAT/USB sector size and the 4096-byte flash erase granularity.
//  - FatFS (ff.h, ESP-IDF component) is mounted over a custom diskio driver
//    that directs all I/O through the same cache/flash functions used by MSC.
//  - USB MSC callbacks and FatFS access are kept mutually exclusive: FatFS is
//    only used when the USB host has been idle for ≥ MSC_WRITE_STABLE_MS ms.
//  - Parameters are persisted to NVS via the Preferences library and are also
//    stored in config.json on the FAT volume so the user can edit them.
// ─────────────────────────────────────────────────────────────────────────────

#include "usb_msc_fat.h"

#include "USB.h"
#include "USBMSC.h"

#include <Preferences.h>
#include <ArduinoJson.h>
#include <inttypes.h>

#include "esp_partition.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

extern "C" {
#include "ff.h"
#include "diskio_impl.h"
}

// ─────────────────────────────────────────────────────────────────────────────
// Constants
// ─────────────────────────────────────────────────────────────────────────────
static const char* TAG = "msc_fat";

#define STORAGE_PARTITION_LABEL   "storage"
#define STORAGE_SECTOR_SIZE       512u
#define FLASH_ERASE_BLOCK         4096u
#define SECTORS_PER_ERASE_BLOCK   (FLASH_ERASE_BLOCK / STORAGE_SECTOR_SIZE)   // 8
#define FAT_DRIVE                 "0:"
#define CONFIG_PATH_SUFFIX        "-config.json"
#define DEFAULT_CONFIG_PATH       "0:/config.json"
#define NVS_NAMESPACE             "wfrx_cfg"       // ≤ 15 chars
#define MSC_WRITE_STABLE_MS       3000u             // idle window before parse

// PWM safety limits enforced during validation
#define PWM_ABS_MIN  800
#define PWM_ABS_MAX  2200

// ─────────────────────────────────────────────────────────────────────────────
// Static state
// ─────────────────────────────────────────────────────────────────────────────

// Flash partition handle
static const esp_partition_t* s_partition = nullptr;
static uint32_t               s_sector_count = 0;

// 4-KB write-back erase-block cache.
// Protected by s_flash_mutex; accessed from USB task and main task.
static SemaphoreHandle_t s_flash_mutex  = nullptr;
static uint8_t           s_erase_cache[FLASH_ERASE_BLOCK];
static uint32_t          s_cached_eb    = UINT32_MAX; // current erase-block index
static bool              s_cache_dirty  = false;

// FatFS state
static FATFS s_fatfs;
static BYTE  s_pdrv = FF_DRV_NOT_USED;

// USB MSC
static USBMSC s_msc;
static volatile uint32_t s_last_write_ms  = 0;
static volatile bool     s_ever_written   = false; // at least one write since boot

// Config file path (discovered at runtime, may have a prefix)
static char s_config_path[64] = "0:/config.json";

// Config.json change tracking (size + FatFS date/time stamp)
static uint32_t s_cfg_size = UINT32_MAX;
static uint16_t s_cfg_fdate = 0;
static uint16_t s_cfg_ftime = 0;

// ─────────────────────────────────────────────────────────────────────────────
// Helpers: Config file discovery
// ─────────────────────────────────────────────────────────────────────────────

// Search FAT volume for ANY config file matching *-config.json pattern.
// Returns the first matching file found in the root directory.
// If found, updates s_config_path and returns true.
// If not found, s_config_path remains at default "0:/config.json".
static bool find_config_file()
{
    FF_DIR dir;
    FILINFO info;
    if (f_opendir(&dir, FAT_DRIVE "/") != FR_OK) {
        return false;
    }

    const char* no_ext_suffix = "-config";
    const size_t suffix_len = strlen(CONFIG_PATH_SUFFIX);
    const size_t no_ext_len = strlen(no_ext_suffix);

    char best_name[64] = {0};
    bool best_is_json = false;

    while (f_readdir(&dir, &info) == FR_OK && info.fname[0] != 0) {
        if (info.fattrib & AM_DIR) continue;

        const char* name = info.fname;
        const size_t n = strlen(name);
        bool match_json = (n >= suffix_len) &&
            (strcmp(name + (n - suffix_len), CONFIG_PATH_SUFFIX) == 0);
        bool match_no_ext = (n >= no_ext_len) &&
            (strcmp(name + (n - no_ext_len), no_ext_suffix) == 0);

        if (match_json || match_no_ext) {
            bool cand_is_json = match_json;
            if (best_name[0] == 0) {
                strncpy(best_name, name, sizeof(best_name) - 1);
                best_name[sizeof(best_name) - 1] = 0;
                best_is_json = cand_is_json;
                continue;
            }

            // Deterministic priority:
            // 1) prefer *-config.json over *-config
            // 2) for ties, lexicographically smaller name
            bool take = false;
            if (cand_is_json && !best_is_json) {
                take = true;
            } else if (cand_is_json == best_is_json && strcmp(name, best_name) < 0) {
                take = true;
            }

            if (take) {
                strncpy(best_name, name, sizeof(best_name) - 1);
                best_name[sizeof(best_name) - 1] = 0;
                best_is_json = cand_is_json;
            }
        }
    }

    f_closedir(&dir);
    if (best_name[0] == 0) {
        return false;
    }

    int written = snprintf(s_config_path, sizeof(s_config_path), "0:/%s", best_name);
    if (written <= 0 || written >= (int)sizeof(s_config_path)) {
        return false;
    }

    ESP_LOGI(TAG, "Found config file: %s", s_config_path);
    return true;
}

// Remove temporary editor sidecar files created next to the active config file.
// Example: config.json.sb-1ebf95c7-ihAnmH on macOS.
static void cleanup_config_sidecar_files()
{
    const char* slash = strrchr(s_config_path, '/');
    const char* base = slash ? (slash + 1) : s_config_path;

    char sidecar_prefix[64];
    snprintf(sidecar_prefix, sizeof(sidecar_prefix), "%s.sb-", base);
    const size_t prefix_len = strlen(sidecar_prefix);

    FF_DIR dir;
    FILINFO info;
    if (f_opendir(&dir, FAT_DRIVE "/") != FR_OK) {
        return;
    }

    while (f_readdir(&dir, &info) == FR_OK && info.fname[0] != 0) {
        if (info.fattrib & AM_DIR) continue;

        if (strncmp(info.fname, sidecar_prefix, prefix_len) == 0) {
            char tmp_path[64];
            snprintf(tmp_path, sizeof(tmp_path), "0:/%s", info.fname);
            FRESULT res = f_unlink(tmp_path);
            if (res == FR_OK) {
                ESP_LOGI(TAG, "Removed temp sidecar: %s", tmp_path);
            } else {
                ESP_LOGW(TAG, "Failed to remove sidecar %s (%d)", tmp_path, res);
            }
        }
    }

    f_closedir(&dir);
}

// ─────────────────────────────────────────────────────────────────────────────
// Helpers: Defaults
// ─────────────────────────────────────────────────────────────────────────────
static void populate_defaults(DeviceConfig& c,
                               const DeviceConfig* seed = nullptr)
{
    // If a seed is supplied (from NVS) use it; otherwise use compiled defaults.
    if (seed) {
        c = *seed;
        return;
    }
    c.vehicleID             = 1;
    c.rx_channel            = 1;
    c.CRUISE_THROTTLE       = 1650;
    c.CREEP_THROTTLE        = 1620;
    c.throttle_expo         = 0.0;
    c.PWM_OUT_1_DEFAULT     = 1500;
    c.PWM_OUT_1_MIN         = 1000;
    c.PWM_OUT_1_MAX         = 2000;
    c.PWM_OUT_2_DEFAULT     = 1500;
    c.PWM_OUT_2_MIN         = 1000;
    c.PWM_OUT_2_MAX         = 2000;
    c.PWM_OUT_3_DEFAULT     = 1500;
    c.PWM_OUT_3_MIN         = 1000;
    c.PWM_OUT_3_MAX         = 2000;
    c.PWM_OUT_4_DEFAULT     = 1500;
    c.PWM_OUT_4_MIN         = 1000;
    c.PWM_OUT_4_MAX         = 2000;
    c.kp                    = 5.0;
    c.ki                    = 0.0;
    c.kd                    = 0.0;
    c.pid_direction         = 0;   
    c.enable_sensor_2       = false;
    c.rear_kp               = 5.0;
    c.rear_ki               = 0.0;
    c.rear_kd               = 0.0;
    c.rear_pid_direction    = 0;   
    c.reverse_rear_steer_in = false;
    c.use_heading           = false;
    c.heading_weight        = 5.0;
    c.sensor_spacing_mm     = 350.0;
    c.stuck_threshold_mm    = 10.0f;
    c.stuck_timeout_ms      = 20000.0f;
    c.BTxFollowChannel      = 0;
    c.BTxThreshold          = 170;
    c.BTxAccelRate          = 0.5;
    c.BTxCreepDuration      = 6;
    c.SineSwerveAmplitude   = 0;
    c.SineSwerveFrequency   = 15;
    c.CHARGE_VBATT_STOP     = 11.9;
    c.CHARGE_MAX_CURRENT    = 6.0;
    c.CHARGE_MAX_SECONDS    = 180;
    c.espnow_enabled        = false;
    c.tpa                   = 0.0;
}

// ─────────────────────────────────────────────────────────────────────────────
// Helpers: Validation / clamping
// ─────────────────────────────────────────────────────────────────────────────
static inline int clamp_pwm(int v) {
    if (v < PWM_ABS_MIN) v = PWM_ABS_MIN;
    if (v > PWM_ABS_MAX) v = PWM_ABS_MAX;
    return v;
}

static void validate_config(DeviceConfig& c)
{
    // Channel
    if (c.rx_channel < 1 || c.rx_channel > 5) c.rx_channel = 1;

    // Throttle range sanity
    c.CRUISE_THROTTLE   = constrain(c.CRUISE_THROTTLE, 1000, 2000);
    c.CREEP_THROTTLE    = constrain(c.CREEP_THROTTLE,  1000, 2000);
    c.throttle_expo     = constrain(c.throttle_expo,   0.0, 1.0);

    // PWM output limits – must satisfy MIN > PWM_ABS_MIN, MAX < PWM_ABS_MAX
    // and DEFAULT must be within [MIN, MAX].
    auto fix_out = [](int& d, int& lo, int& hi) {
        lo = constrain(lo, PWM_ABS_MIN, PWM_ABS_MAX);
        hi = constrain(hi, PWM_ABS_MIN, PWM_ABS_MAX);
        if (lo > hi) {
            lo = constrain(hi - 100, PWM_ABS_MIN, PWM_ABS_MAX);
            hi = constrain(lo + 100, PWM_ABS_MIN, PWM_ABS_MAX);
        }
        d  = constrain(d, lo, hi);
    };
    fix_out(c.PWM_OUT_1_DEFAULT, c.PWM_OUT_1_MIN, c.PWM_OUT_1_MAX);
    fix_out(c.PWM_OUT_2_DEFAULT, c.PWM_OUT_2_MIN, c.PWM_OUT_2_MAX);
    fix_out(c.PWM_OUT_3_DEFAULT, c.PWM_OUT_3_MIN, c.PWM_OUT_3_MAX);
    fix_out(c.PWM_OUT_4_DEFAULT, c.PWM_OUT_4_MIN, c.PWM_OUT_4_MAX);

    // PID
    if (c.kp < 0.0)  c.kp = 0.0;
    if (c.ki < 0.0)  c.ki = 0.0;
    if (c.kd < 0.0)  c.kd = 0.0;
    if (c.pid_direction < 0 || c.pid_direction > 1) c.pid_direction = 0;
    if (c.rear_kp < 0.0)  c.rear_kp = 0.0;
    if (c.rear_ki < 0.0)  c.rear_ki = 0.0;
    if (c.rear_kd < 0.0)  c.rear_kd = 0.0;
    if (c.rear_pid_direction < 0 || c.rear_pid_direction > 1)
        c.rear_pid_direction = 0;

    // Stuck
    if (c.stuck_threshold_mm < 1.0f)   c.stuck_threshold_mm = 1.0f;
    if (c.stuck_timeout_ms   < 1000.0f) c.stuck_timeout_ms  = 1000.0f;

    // BTx
    if (c.BTxFollowChannel > 2)   c.BTxFollowChannel = 0;
    c.BTxThreshold = constrain(c.BTxThreshold, 11, 2000);
    if (c.BTxAccelRate     < 0.3) c.BTxAccelRate     = 0.3;
    if (c.BTxCreepDuration < 1)   c.BTxCreepDuration = 1;

    // Sine
    if (c.SineSwerveFrequency < 1) c.SineSwerveFrequency = 1;

    // Charging
    if (c.CHARGE_VBATT_STOP  < 6.0)  c.CHARGE_VBATT_STOP  = 6.0;
    if (c.CHARGE_VBATT_STOP  > 26.0) c.CHARGE_VBATT_STOP  = 26.0;
    if (c.CHARGE_MAX_CURRENT < 0.1)  c.CHARGE_MAX_CURRENT = 0.1;
    if (c.CHARGE_MAX_CURRENT > 8.0) c.CHARGE_MAX_CURRENT = 8.0;
    if (c.CHARGE_MAX_SECONDS < 5)   c.CHARGE_MAX_SECONDS = 5;
    if (c.CHARGE_MAX_SECONDS > 3600) c.CHARGE_MAX_SECONDS = 3600;

    // TPA (>1.0 = full attenuation reached before max throttle)
    c.tpa = constrain(c.tpa, 0.0, 3.0);
}

// ─────────────────────────────────────────────────────────────────────────────
// Helpers: NVS (Preferences)
// ─────────────────────────────────────────────────────────────────────────────
static void prefs_save(const DeviceConfig& c)
{
    Preferences p;
    p.begin(NVS_NAMESPACE, false);
    p.putUChar("vID",      c.vehicleID);
    p.putUChar("rxCh",     c.rx_channel);
    p.putInt("crsThr",     c.CRUISE_THROTTLE);
    p.putInt("crpThr",     c.CREEP_THROTTLE);
    p.putDouble("thrExp",  c.throttle_expo);
    p.putInt("p1Def",      c.PWM_OUT_1_DEFAULT);
    p.putInt("p1Min",      c.PWM_OUT_1_MIN);
    p.putInt("p1Max",      c.PWM_OUT_1_MAX);
    p.putInt("p2Def",      c.PWM_OUT_2_DEFAULT);
    p.putInt("p2Min",      c.PWM_OUT_2_MIN);
    p.putInt("p2Max",      c.PWM_OUT_2_MAX);
    p.putInt("p3Def",      c.PWM_OUT_3_DEFAULT);
    p.putInt("p3Min",      c.PWM_OUT_3_MIN);
    p.putInt("p3Max",      c.PWM_OUT_3_MAX);
    p.putInt("p4Def",      c.PWM_OUT_4_DEFAULT);
    p.putInt("p4Min",      c.PWM_OUT_4_MIN);
    p.putInt("p4Max",      c.PWM_OUT_4_MAX);
    p.putDouble("kp",      c.kp);
    p.putDouble("ki",      c.ki);
    p.putDouble("kd",      c.kd);
    p.putInt("pidDir",     c.pid_direction);
    p.putBool("enS2",      c.enable_sensor_2);
    p.putDouble("rkp",     c.rear_kp);
    p.putDouble("rki",     c.rear_ki);
    p.putDouble("rkd",     c.rear_kd);
    p.putInt("rPidDir",    c.rear_pid_direction);
    p.putBool("revRear",   c.reverse_rear_steer_in);
    p.putBool("useHdg",    c.use_heading);
    p.putDouble("hdgWt",   c.heading_weight);
    p.putDouble("snsSpc",  c.sensor_spacing_mm);
    p.putFloat("stThresh", c.stuck_threshold_mm);
    p.putFloat("stTout",   c.stuck_timeout_ms);
    p.putUChar("btxCh",    c.BTxFollowChannel);
    p.putInt("btxThr",     c.BTxThreshold);
    p.putDouble("btxAcc",  c.BTxAccelRate);
    p.putInt("btxDur",     c.BTxCreepDuration);
    p.putUChar("sineAmp",  c.SineSwerveAmplitude);
    p.putUChar("sineFrq",  c.SineSwerveFrequency);
    p.putDouble("chgVStp", c.CHARGE_VBATT_STOP);
    p.putDouble("chgIMax", c.CHARGE_MAX_CURRENT);
    p.putInt("chgTSec",    c.CHARGE_MAX_SECONDS);
    p.putBool("espNow",    c.espnow_enabled);
    p.putDouble("tpa",     c.tpa);
    p.end();
}

static bool prefs_load(DeviceConfig& c)
{
    Preferences p;
    if (!p.begin(NVS_NAMESPACE, true)) return false;
    if (!p.isKey("vID")) { p.end(); return false; } // namespace is empty

    c.vehicleID             = p.getUChar("vID",      c.vehicleID);
    c.rx_channel            = p.getUChar("rxCh",     c.rx_channel);
    c.CRUISE_THROTTLE       = p.getInt("crsThr",     c.CRUISE_THROTTLE);
    c.CREEP_THROTTLE        = p.getInt("crpThr",     c.CREEP_THROTTLE);
    c.throttle_expo         = p.getDouble("thrExp",  c.throttle_expo);
    c.PWM_OUT_1_DEFAULT     = p.getInt("p1Def",      c.PWM_OUT_1_DEFAULT);
    c.PWM_OUT_1_MIN         = p.getInt("p1Min",      c.PWM_OUT_1_MIN);
    c.PWM_OUT_1_MAX         = p.getInt("p1Max",      c.PWM_OUT_1_MAX);
    c.PWM_OUT_2_DEFAULT     = p.getInt("p2Def",      c.PWM_OUT_2_DEFAULT);
    c.PWM_OUT_2_MIN         = p.getInt("p2Min",      c.PWM_OUT_2_MIN);
    c.PWM_OUT_2_MAX         = p.getInt("p2Max",      c.PWM_OUT_2_MAX);
    c.PWM_OUT_3_DEFAULT     = p.getInt("p3Def",      c.PWM_OUT_3_DEFAULT);
    c.PWM_OUT_3_MIN         = p.getInt("p3Min",      c.PWM_OUT_3_MIN);
    c.PWM_OUT_3_MAX         = p.getInt("p3Max",      c.PWM_OUT_3_MAX);
    c.PWM_OUT_4_DEFAULT     = p.getInt("p4Def",      c.PWM_OUT_4_DEFAULT);
    c.PWM_OUT_4_MIN         = p.getInt("p4Min",      c.PWM_OUT_4_MIN);
    c.PWM_OUT_4_MAX         = p.getInt("p4Max",      c.PWM_OUT_4_MAX);
    c.kp                    = p.getDouble("kp",      c.kp);
    c.ki                    = p.getDouble("ki",      c.ki);
    c.kd                    = p.getDouble("kd",      c.kd);
    c.pid_direction         = p.getInt("pidDir",     c.pid_direction);
    c.enable_sensor_2       = p.getBool("enS2",      c.enable_sensor_2);
    c.rear_kp               = p.getDouble("rkp",     c.rear_kp);
    c.rear_ki               = p.getDouble("rki",     c.rear_ki);
    c.rear_kd               = p.getDouble("rkd",     c.rear_kd);
    c.rear_pid_direction    = p.getInt("rPidDir",    c.rear_pid_direction);
    c.reverse_rear_steer_in = p.getBool("revRear",   c.reverse_rear_steer_in);
    c.use_heading           = p.getBool("useHdg",    c.use_heading);
    c.heading_weight        = p.getDouble("hdgWt",   c.heading_weight);
    c.sensor_spacing_mm     = p.getDouble("snsSpc",  c.sensor_spacing_mm);
    c.stuck_threshold_mm    = p.getFloat("stThresh", c.stuck_threshold_mm);
    c.stuck_timeout_ms      = p.getFloat("stTout",   c.stuck_timeout_ms);
    c.BTxFollowChannel      = p.getUChar("btxCh",    c.BTxFollowChannel);
    c.BTxThreshold          = p.getInt("btxThr", c.BTxThreshold);
    c.BTxAccelRate          = p.getDouble("btxAcc",  c.BTxAccelRate);
    c.BTxCreepDuration      = p.getInt("btxDur",     c.BTxCreepDuration);
    c.SineSwerveAmplitude   = p.getUChar("sineAmp",  c.SineSwerveAmplitude);
    c.SineSwerveFrequency   = p.getUChar("sineFrq",  c.SineSwerveFrequency);
    c.CHARGE_VBATT_STOP     = p.getDouble("chgVStp", c.CHARGE_VBATT_STOP);
    c.CHARGE_MAX_CURRENT    = p.getDouble("chgIMax", c.CHARGE_MAX_CURRENT);
    c.CHARGE_MAX_SECONDS    = p.getInt("chgTSec",    c.CHARGE_MAX_SECONDS);
    c.espnow_enabled        = p.getBool("espNow",    c.espnow_enabled);
    c.tpa                   = p.getDouble("tpa",      c.tpa);
    p.end();
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Flash block device – erase-block write-back cache
// ─────────────────────────────────────────────────────────────────────────────

// Must hold s_flash_mutex before calling any of the functions below.

static esp_err_t flash_flush_cache_locked()
{
    if (!s_cache_dirty || s_cached_eb == UINT32_MAX) return ESP_OK;

    uint32_t eb_offset = s_cached_eb * FLASH_ERASE_BLOCK;
    esp_err_t err = esp_partition_erase_range(s_partition, eb_offset,
                                               FLASH_ERASE_BLOCK);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "erase failed @ 0x%08" PRIx32 ": %s",
                 eb_offset, esp_err_to_name(err));
        return err;
    }
    err = esp_partition_write(s_partition, eb_offset,
                               s_erase_cache, FLASH_ERASE_BLOCK);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "write failed @ 0x%08" PRIx32 ": %s",
                 eb_offset, esp_err_to_name(err));
        return err;
    }
    s_cache_dirty = false;
    return ESP_OK;
}

// Load erase-block 'eb' into the cache (flushing any dirty previous block first).
static esp_err_t flash_load_erase_block_locked(uint32_t eb)
{
    if (s_cached_eb == eb) return ESP_OK;  // already loaded

    esp_err_t err = flash_flush_cache_locked();
    if (err != ESP_OK) return err;

    uint32_t eb_offset = eb * FLASH_ERASE_BLOCK;
    err = esp_partition_read(s_partition, eb_offset,
                              s_erase_cache, FLASH_ERASE_BLOCK);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "read erase block failed @ 0x%08" PRIx32 ": %s",
                 eb_offset, esp_err_to_name(err));
        return err;
    }
    s_cached_eb   = eb;
    s_cache_dirty = false;
    return ESP_OK;
}

// Public (within this file) read / write / flush helpers.
// These acquire the mutex internally.

static bool flash_read_sector(uint32_t lba, uint8_t* buf)
{
    if (!s_partition || lba >= s_sector_count) return false;

    xSemaphoreTake(s_flash_mutex, portMAX_DELAY);
    uint32_t eb = lba / SECTORS_PER_ERASE_BLOCK;
    uint32_t offset_in_eb = (lba % SECTORS_PER_ERASE_BLOCK) * STORAGE_SECTOR_SIZE;

    esp_err_t err;
    if (s_cached_eb == eb) {
        // Serve from cache (which may have unsaved writes)
        memcpy(buf, s_erase_cache + offset_in_eb, STORAGE_SECTOR_SIZE);
        err = ESP_OK;
    } else {
        // Read directly from flash (cache holds a different block)
        err = esp_partition_read(s_partition,
                                  lba * STORAGE_SECTOR_SIZE,
                                  buf, STORAGE_SECTOR_SIZE);
    }
    xSemaphoreGive(s_flash_mutex);
    return (err == ESP_OK);
}

static bool flash_write_sector(uint32_t lba, const uint8_t* buf)
{
    if (!s_partition || lba >= s_sector_count) return false;

    xSemaphoreTake(s_flash_mutex, portMAX_DELAY);
    uint32_t eb = lba / SECTORS_PER_ERASE_BLOCK;
    uint32_t offset_in_eb = (lba % SECTORS_PER_ERASE_BLOCK) * STORAGE_SECTOR_SIZE;

    esp_err_t err = flash_load_erase_block_locked(eb);
    if (err == ESP_OK) {
        memcpy(s_erase_cache + offset_in_eb, buf, STORAGE_SECTOR_SIZE);
        s_cache_dirty = true;
    }
    xSemaphoreGive(s_flash_mutex);
    return (err == ESP_OK);
}

static bool flash_flush()
{
    xSemaphoreTake(s_flash_mutex, portMAX_DELAY);
    esp_err_t err = flash_flush_cache_locked();
    xSemaphoreGive(s_flash_mutex);
    return (err == ESP_OK);
}

// ─────────────────────────────────────────────────────────────────────────────
// FatFS diskio driver
// ─────────────────────────────────────────────────────────────────────────────

static DSTATUS diskio_init_cb(unsigned char pdrv)
{
    (void) pdrv;
    return s_partition ? 0 : STA_NOINIT;
}

static DSTATUS diskio_status_cb(unsigned char pdrv)
{
    (void) pdrv;
    return s_partition ? 0 : STA_NOINIT;
}

static DRESULT diskio_read_cb(unsigned char pdrv,
                               unsigned char* buff,
                               uint32_t sector,
                               unsigned count)
{
    (void) pdrv;
    for (unsigned i = 0; i < count; ++i) {
        if (!flash_read_sector(sector + i, buff + i * STORAGE_SECTOR_SIZE))
            return RES_ERROR;
    }
    return RES_OK;
}

static DRESULT diskio_write_cb(unsigned char pdrv,
                                const unsigned char* buff,
                                uint32_t sector,
                                unsigned count)
{
    (void) pdrv;
    for (unsigned i = 0; i < count; ++i) {
        if (!flash_write_sector(sector + i, buff + i * STORAGE_SECTOR_SIZE))
            return RES_ERROR;
    }
    // Flush after FatFS write sequences so the data persists before the next
    // power cycle.
    return flash_flush() ? RES_OK : RES_ERROR;
}

static DRESULT diskio_ioctl_cb(unsigned char pdrv, unsigned char cmd, void* buff)
{
    (void) pdrv;
    switch (cmd) {
        case CTRL_SYNC:
            return flash_flush() ? RES_OK : RES_ERROR;
        case GET_SECTOR_COUNT:
            *(DWORD*)buff = (DWORD)s_sector_count;
            return RES_OK;
        case GET_SECTOR_SIZE:
            *(WORD*)buff = (WORD)STORAGE_SECTOR_SIZE;
            return RES_OK;
        case GET_BLOCK_SIZE:
            *(DWORD*)buff = (DWORD)SECTORS_PER_ERASE_BLOCK;
            return RES_OK;
        default:
            return RES_PARERR;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// FatFS mount / unmount
// ─────────────────────────────────────────────────────────────────────────────

static bool fatfs_mount()
{
    FRESULT res = f_mount(&s_fatfs, FAT_DRIVE, 1 /* force mount */);
    if (res == FR_OK) return true;

    ESP_LOGW(TAG, "f_mount failed (%d); formatting...", res);
    // Format the volume (FM_FAT = FAT12/16, au=0 = default cluster size)
    BYTE work[FF_MAX_SS];
    res = f_mkfs(FAT_DRIVE, FM_FAT, 0, work, sizeof(work));
    if (res != FR_OK) {
        ESP_LOGE(TAG, "f_mkfs failed (%d)", res);
        return false;
    }
    res = f_mount(&s_fatfs, FAT_DRIVE, 1);
    if (res != FR_OK) {
        ESP_LOGE(TAG, "f_mount after mkfs failed (%d)", res);
        return false;
    }
    // Set volume label only if this FatFS build enables label APIs.
#if FF_USE_LABEL
    f_setlabel("WFRX CONFIG");
#endif
    return true;
}

static void fatfs_unmount()
{
    f_mount(nullptr, FAT_DRIVE, 0);
}

// ─────────────────────────────────────────────────────────────────────────────
// JSON serialise / parse
// ─────────────────────────────────────────────────────────────────────────────

// Build a pretty-printed JSON document from cfg and write it to s_config_path.
static bool config_write_json(const DeviceConfig& c)
{
    // Use a statically-sized document to avoid heap fragmentation.
    // The document holds approximately 40 key-value pairs; 2048 bytes is ample.
    JsonDocument doc;

    doc["vehicleID"]             = c.vehicleID;
    doc["rx_channel"]            = c.rx_channel;
    doc["CRUISE_THROTTLE"]       = c.CRUISE_THROTTLE;
    doc["CREEP_THROTTLE"]        = c.CREEP_THROTTLE;
    doc["throttle_expo"]         = c.throttle_expo;
    doc["PWM_OUT_1_DEFAULT"]     = c.PWM_OUT_1_DEFAULT;
    doc["PWM_OUT_1_MIN"]         = c.PWM_OUT_1_MIN;
    doc["PWM_OUT_1_MAX"]         = c.PWM_OUT_1_MAX;
    doc["PWM_OUT_2_DEFAULT"]     = c.PWM_OUT_2_DEFAULT;
    doc["PWM_OUT_2_MIN"]         = c.PWM_OUT_2_MIN;
    doc["PWM_OUT_2_MAX"]         = c.PWM_OUT_2_MAX;
    doc["PWM_OUT_3_DEFAULT"]     = c.PWM_OUT_3_DEFAULT;
    doc["PWM_OUT_3_MIN"]         = c.PWM_OUT_3_MIN;
    doc["PWM_OUT_3_MAX"]         = c.PWM_OUT_3_MAX;
    doc["PWM_OUT_4_DEFAULT"]     = c.PWM_OUT_4_DEFAULT;
    doc["PWM_OUT_4_MIN"]         = c.PWM_OUT_4_MIN;
    doc["PWM_OUT_4_MAX"]         = c.PWM_OUT_4_MAX;
    doc["kp"]                    = c.kp;
    doc["ki"]                    = c.ki;
    doc["kd"]                    = c.kd;
    doc["pid_direction"]         = (bool)c.pid_direction;   // false=NORMAL, true=REVERSE
    doc["enable_sensor_2"]       = c.enable_sensor_2;
    doc["use_heading"]           = c.use_heading;
    doc["heading_weight"]        = c.heading_weight;
    doc["sensor_spacing_mm"]     = c.sensor_spacing_mm;
    doc["rear_kp"]               = c.rear_kp;
    doc["rear_ki"]               = c.rear_ki;
    doc["rear_kd"]               = c.rear_kd;
    doc["rear_pid_direction"]    = (bool)c.rear_pid_direction;
    doc["reverse_rear_steer_in"] = c.reverse_rear_steer_in;
    doc["tpa"]                   = c.tpa;
    doc["stuck_threshold_mm"]    = c.stuck_threshold_mm;
    doc["stuck_timeout_ms"]      = c.stuck_timeout_ms;
    doc["BTxFollowChannel"]      = c.BTxFollowChannel;
    doc["BTxThreshold"]          = c.BTxThreshold;
    doc["BTxAccelRate"]          = c.BTxAccelRate;
    doc["BTxCreepDuration"]      = c.BTxCreepDuration;
    doc["SineSwerveAmplitude"]   = c.SineSwerveAmplitude;
    doc["SineSwerveFrequency"]   = c.SineSwerveFrequency;
    doc["CHARGE_VBATT_STOP"]     = c.CHARGE_VBATT_STOP;
    doc["CHARGE_MAX_CURRENT"]    = c.CHARGE_MAX_CURRENT;
    doc["CHARGE_MAX_SECONDS"]    = c.CHARGE_MAX_SECONDS;
    doc["espnow_enabled"]        = c.espnow_enabled;

    // Measure serialised size first
    size_t json_len = measureJsonPretty(doc);

    // Build output in memory first; only open/truncate the file after we know
    // serialization completed successfully.
    String json_str;
    if (!json_str.reserve((unsigned)json_len + 4)) {
        ESP_LOGE(TAG, "json_str reserve failed");
        return false;
    }
    size_t written_json = serializeJsonPretty(doc, json_str);
    if (written_json != json_len) {
        ESP_LOGE(TAG, "serializeJsonPretty truncated (%u/%u)",
                 (unsigned)written_json, (unsigned)json_len);
        return false;
    }

    String out_str;
    if (!out_str.reserve(json_str.length() + 4)) {
        ESP_LOGE(TAG, "out_str reserve failed");
        return false;
    }
    out_str += json_str;
    out_str += "\n";

    FIL fil;
    FRESULT res = f_open(&fil, s_config_path, FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK) {
        ESP_LOGE(TAG, "f_open write failed (%d)", res);
        return false;
    }

    UINT bw;
    res = f_write(&fil, out_str.c_str(), (UINT)out_str.length(), &bw);
    f_close(&fil);

    if (res != FR_OK || bw != out_str.length()) {
        ESP_LOGE(TAG, "f_write failed (%d)", res);
        return false;
    }
    return true;
}

// Parse s_config_path on the mounted FAT volume into cfg.
// Unknown / invalid fields are silently ignored; previously valid fields keep
// their current (NVS-loaded) value.
static bool config_read_json(DeviceConfig& c)
{
    FIL fil;
    FRESULT res = f_open(&fil, s_config_path, FA_READ);
    if (res != FR_OK) {
        ESP_LOGW(TAG, "config.json not found (%d)", res);
        return false;
    }

    FSIZE_t fsize = f_size(&fil);
    if (fsize == 0 || fsize > 8192) {
        f_close(&fil);
        ESP_LOGW(TAG, "config.json invalid size (%u)", (unsigned)fsize);
        return false;
    }

    String json_str;
    json_str.reserve((unsigned)fsize + 1);

    char rd_buf[256];
    UINT br;
    while (f_read(&fil, rd_buf, sizeof(rd_buf), &br) == FR_OK && br > 0) {
        json_str.concat(rd_buf, br);
    }
    f_close(&fil);

    // Allow human-readable comments in config.json by stripping C/C++ comment
    // syntax before handing content to ArduinoJson.
    String json_no_comments;
    json_no_comments.reserve(json_str.length());
    bool in_string = false;
    bool escaped = false;
    bool in_line_comment = false;
    bool in_block_comment = false;

    for (size_t i = 0; i < json_str.length(); ++i) {
        char ch = json_str[i];
        char next = (i + 1 < json_str.length()) ? json_str[i + 1] : '\0';

        if (in_line_comment) {
            if (ch == '\n') {
                in_line_comment = false;
                json_no_comments += ch;
            }
            continue;
        }

        if (in_block_comment) {
            if (ch == '*' && next == '/') {
                in_block_comment = false;
                ++i;
            }
            continue;
        }

        if (in_string) {
            json_no_comments += ch;
            if (escaped) {
                escaped = false;
            } else if (ch == '\\') {
                escaped = true;
            } else if (ch == '"') {
                in_string = false;
            }
            continue;
        }

        if (ch == '"') {
            in_string = true;
            json_no_comments += ch;
            continue;
        }

        if (ch == '/' && next == '/') {
            in_line_comment = true;
            ++i;
            continue;
        }

        if (ch == '/' && next == '*') {
            in_block_comment = true;
            ++i;
            continue;
        }

        json_no_comments += ch;
    }

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, json_no_comments);
    if (err) {
        ESP_LOGE(TAG, "JSON parse error: %s", err.c_str());
        return false;
    }

    // Helper macro: update only if key present and value is the expected type.
#define JSON_GET_INT(key, field)    if (doc[key].is<int>())    c.field = doc[key].as<int>()
#define JSON_GET_UINT8(key, field)  if (doc[key].is<int>())    c.field = (uint8_t)constrain(doc[key].as<int>(), 0, 255)
#define JSON_GET_DBL(key, field)    if (doc[key].is<double>()) c.field = doc[key].as<double>()
#define JSON_GET_FLT(key, field)    if (doc[key].is<double>()) c.field = (float)doc[key].as<double>()
#define JSON_GET_BOOL(key, field)   if (doc[key].is<bool>())   c.field = doc[key].as<bool>()

    JSON_GET_UINT8("vehicleID",             vehicleID);
    JSON_GET_UINT8("rx_channel",            rx_channel);
    JSON_GET_INT("CRUISE_THROTTLE",         CRUISE_THROTTLE);
    JSON_GET_INT("CREEP_THROTTLE",          CREEP_THROTTLE);
    JSON_GET_DBL("throttle_expo",           throttle_expo);
    JSON_GET_INT("PWM_OUT_1_DEFAULT",       PWM_OUT_1_DEFAULT);
    JSON_GET_INT("PWM_OUT_1_MIN",           PWM_OUT_1_MIN);
    JSON_GET_INT("PWM_OUT_1_MAX",           PWM_OUT_1_MAX);
    JSON_GET_INT("PWM_OUT_2_DEFAULT",       PWM_OUT_2_DEFAULT);
    JSON_GET_INT("PWM_OUT_2_MIN",           PWM_OUT_2_MIN);
    JSON_GET_INT("PWM_OUT_2_MAX",           PWM_OUT_2_MAX);
    JSON_GET_INT("PWM_OUT_3_DEFAULT",       PWM_OUT_3_DEFAULT);
    JSON_GET_INT("PWM_OUT_3_MIN",           PWM_OUT_3_MIN);
    JSON_GET_INT("PWM_OUT_3_MAX",           PWM_OUT_3_MAX);
    JSON_GET_INT("PWM_OUT_4_DEFAULT",       PWM_OUT_4_DEFAULT);
    JSON_GET_INT("PWM_OUT_4_MIN",           PWM_OUT_4_MIN);
    JSON_GET_INT("PWM_OUT_4_MAX",           PWM_OUT_4_MAX);
    JSON_GET_DBL("kp",                      kp);
    JSON_GET_DBL("ki",                      ki);
    JSON_GET_DBL("kd",                      kd);
    JSON_GET_BOOL("pid_direction",          pid_direction);
    JSON_GET_BOOL("enable_sensor_2",        enable_sensor_2);
    JSON_GET_DBL("rear_kp",                 rear_kp);
    JSON_GET_DBL("rear_ki",                 rear_ki);
    JSON_GET_DBL("rear_kd",                 rear_kd);
    JSON_GET_BOOL("rear_pid_direction",     rear_pid_direction);
    JSON_GET_BOOL("reverse_rear_steer_in",  reverse_rear_steer_in);
    JSON_GET_BOOL("use_heading",            use_heading);
    JSON_GET_DBL("heading_weight",          heading_weight);
    JSON_GET_DBL("sensor_spacing_mm",       sensor_spacing_mm);
    JSON_GET_FLT("stuck_threshold_mm",      stuck_threshold_mm);
    JSON_GET_FLT("stuck_timeout_ms",        stuck_timeout_ms);
    JSON_GET_UINT8("BTxFollowChannel",      BTxFollowChannel);
    JSON_GET_INT("BTxThreshold",            BTxThreshold);
    JSON_GET_INT("BTxFollowThreshold",      BTxThreshold);
    JSON_GET_DBL("BTxAccelRate",            BTxAccelRate);
    JSON_GET_INT("BTxCreepDuration",        BTxCreepDuration);
    JSON_GET_UINT8("SineSwerveAmplitude",   SineSwerveAmplitude);
    JSON_GET_UINT8("SineSwerveFrequency",   SineSwerveFrequency);
    JSON_GET_DBL("CHARGE_VBATT_STOP",       CHARGE_VBATT_STOP);
    JSON_GET_DBL("CHARGE_MAX_CURRENT",      CHARGE_MAX_CURRENT);
    JSON_GET_INT("CHARGE_MAX_SECONDS",      CHARGE_MAX_SECONDS);
    JSON_GET_BOOL("espnow_enabled",         espnow_enabled);
    JSON_GET_DBL("tpa",                      tpa);

#undef JSON_GET_INT
#undef JSON_GET_UINT8
#undef JSON_GET_DBL
#undef JSON_GET_FLT
#undef JSON_GET_BOOL

    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Config file change detection helpers
// ─────────────────────────────────────────────────────────────────────────────

// Record the current size/timestamp of config.json (must be called with FAT
// Record the current size/timestamp of the config file (must be called with FAT
// mounted).
static void record_config_stamp()
{
    FILINFO info;
    if (f_stat(s_config_path, &info) == FR_OK) {
        s_cfg_size  = (uint32_t)info.fsize;
        s_cfg_fdate = info.fdate;
        s_cfg_ftime = info.ftime;
    } else {
        s_cfg_size  = UINT32_MAX;
        s_cfg_fdate = 0;
        s_cfg_ftime = 0;
    }
}

// Returns true if config file differs from what we last recorded.
static bool config_has_changed()
{
    FILINFO info;
    if (f_stat(s_config_path, &info) != FR_OK) return false;
    return (info.fsize  != s_cfg_size  ||
            info.fdate  != s_cfg_fdate ||
            info.ftime  != s_cfg_ftime);
}

// ─────────────────────────────────────────────────────────────────────────────
// USB MSC callbacks
// ─────────────────────────────────────────────────────────────────────────────

static int32_t on_msc_read(uint32_t lba, uint32_t offset,
                             void* buffer, uint32_t bufsize)
{
    uint8_t* dst = static_cast<uint8_t*>(buffer) + offset;
    uint32_t sectors = bufsize / STORAGE_SECTOR_SIZE;
    for (uint32_t i = 0; i < sectors; ++i) {
        if (!flash_read_sector(lba + i, dst + i * STORAGE_SECTOR_SIZE))
            return -1;
    }
    return (int32_t)bufsize;
}

static int32_t on_msc_write(uint32_t lba, uint32_t offset,
                              uint8_t* buffer, uint32_t bufsize)
{
    const uint8_t* src = buffer + offset;
    uint32_t sectors = bufsize / STORAGE_SECTOR_SIZE;
    for (uint32_t i = 0; i < sectors; ++i) {
        if (!flash_write_sector(lba + i, src + i * STORAGE_SECTOR_SIZE))
            return -1;
    }
    // Record that a write happened; the quiet timer is checked in msc_fat_loop().
    s_last_write_ms = (uint32_t)millis();
    s_ever_written  = true;
    return (int32_t)bufsize;
}

static bool on_msc_start_stop(uint8_t power_condition,
                                bool start, bool load_eject)
{
    (void)power_condition;
    if (load_eject && !start) {
        // Host requested eject: flush write cache immediately to ensure data
        // is committed before the user unplugs the cable.
        ESP_LOGI(TAG, "USB eject: flushing flash cache");
        flash_flush();
    }
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// msc_fat_init
// ─────────────────────────────────────────────────────────────────────────────
void msc_fat_init(DeviceConfig* cfg)
{
    // 1. Create the flash access mutex.
    s_flash_mutex = xSemaphoreCreateMutex();
    configASSERT(s_flash_mutex);

    // 2. Locate the "storage" partition.
    s_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                            ESP_PARTITION_SUBTYPE_ANY,
                                            STORAGE_PARTITION_LABEL);
    if (!s_partition) {
        ESP_LOGE(TAG, "Partition '%s' not found – check partitions.csv",
                 STORAGE_PARTITION_LABEL);
        // We cannot continue without the partition, but we still populate cfg
        // from defaults + NVS so the firmware can run.
        populate_defaults(*cfg);
        prefs_load(*cfg);
        validate_config(*cfg);
        return;
    }

    s_sector_count = s_partition->size / STORAGE_SECTOR_SIZE;
    ESP_LOGI(TAG, "Storage partition: %u KB, %u sectors",
             (unsigned)(s_partition->size / 1024), (unsigned)s_sector_count);

    // 3. Register the FatFS diskio driver.
    // Keep FatFS drive index fixed to 0 because paths are intentionally fixed
    // as "0:" and "0:/config.json" for compatibility with existing code.
    s_pdrv = 0;

    static const ff_diskio_impl_t diskio_impl = {
        .init   = diskio_init_cb,
        .status = diskio_status_cb,
        .read   = diskio_read_cb,
        .write  = diskio_write_cb,
        .ioctl  = diskio_ioctl_cb,
    };
    ff_diskio_register(s_pdrv, &diskio_impl);

    // 4. Start with compiled defaults, then override from NVS.
    populate_defaults(*cfg);
    prefs_load(*cfg);  // no-op if NVS is empty (first boot)
    validate_config(*cfg);

    // 5. Mount FAT, create config file on first boot, read it, unmount.
    if (fatfs_mount()) {
        // Search for any file matching *-config.json pattern; if found, use it.
        // Otherwise, s_config_path will remain at default "0:/config.json".
        find_config_file();
        cleanup_config_sidecar_files();

        FILINFO info;
        bool file_exists = (f_stat(s_config_path, &info) == FR_OK);

        if (!file_exists) {
            ESP_LOGI(TAG, "First boot: creating %s", s_config_path);
            config_write_json(*cfg);
            // Re-stat after write
            file_exists = (f_stat(s_config_path, &info) == FR_OK);
        } else {
            // File exists – parse it and override NVS values.
            ESP_LOGI(TAG, "config.json found; parsing");
            DeviceConfig file_cfg = *cfg;    // start from NVS base
            if (config_read_json(file_cfg)) {
                validate_config(file_cfg);
                *cfg = file_cfg;
                prefs_save(*cfg);
            }
        }
        record_config_stamp();
        fatfs_unmount();
    } else {
        ESP_LOGE(TAG, "FAT mount/format failed");
    }

    // 6. Configure and start USB MSC.
    s_msc.vendorID("WFRX");
    s_msc.productID("CONFIG_DRIVE");
    s_msc.productRevision("1.0");
    s_msc.onStartStop(on_msc_start_stop);
    s_msc.onRead(on_msc_read);
    s_msc.onWrite(on_msc_write);
    s_msc.mediaPresent(true);
    s_msc.begin((uint32_t)s_sector_count, STORAGE_SECTOR_SIZE);
    // USB.begin() is called by Serial.begin() / the Arduino core; no need to
    // call it explicitly here.
}

// ─────────────────────────────────────────────────────────────────────────────
// msc_fat_loop
// ─────────────────────────────────────────────────────────────────────────────
bool msc_fat_loop(DeviceConfig* cfg)
{
    if (!s_ever_written) return false;

    uint32_t now = (uint32_t)millis();
    uint32_t idle_ms = now - s_last_write_ms;

    if (idle_ms < MSC_WRITE_STABLE_MS) return false;  // still settling

    // Host has been idle for ≥ 3 s.  Reset the trigger so we only act once per
    // idle window.
    s_ever_written = false;

    // Flush any remaining write-back cache so flash is consistent.
    if (!flash_flush()) {
        ESP_LOGE(TAG, "Cache flush failed after USB idle");
        return false;
    }

    // Mount FAT and check whether config.json changed.
    if (!fatfs_mount()) {
        ESP_LOGE(TAG, "FAT remount failed after USB idle");
        return false;
    }

    // Some editors create temporary sidecar files (for example on macOS).
    // Remove them so they do not accumulate on the config volume.
    cleanup_config_sidecar_files();

    bool changed = config_has_changed();
    if (!changed) {
        ESP_LOGI(TAG, "config.json unchanged");
        fatfs_unmount();
        return false;
    }

    ESP_LOGI(TAG, "config.json changed; parsing");
    DeviceConfig new_cfg = *cfg;   // start from current values
    bool ok = config_read_json(new_cfg);

    if (ok) {
        validate_config(new_cfg);
        record_config_stamp();
        fatfs_unmount();
        *cfg = new_cfg;
        prefs_save(new_cfg);
        return true;
    } else {
        ESP_LOGW(TAG, "config.json parse failed; keeping existing config");
        record_config_stamp();
        fatfs_unmount();
        return false;
    }
}

void msc_fat_save_nvs(const DeviceConfig* cfg)
{
    if (!cfg) return;
    DeviceConfig tmp = *cfg;
    validate_config(tmp);
    prefs_save(tmp);
}

bool msc_fat_save_all(const DeviceConfig* cfg)
{
    if (!cfg) return false;

    DeviceConfig tmp = *cfg;
    validate_config(tmp);

    // Always persist to NVS first so settings survive reboot even if FAT write fails.
    prefs_save(tmp);

    if (!fatfs_mount()) {
        ESP_LOGE(TAG, "FAT mount failed while saving config.json");
        return false;
    }

    bool ok = config_write_json(tmp);
    if (ok) {
        record_config_stamp();
    } else {
        ESP_LOGE(TAG, "Failed to write config file: %s", s_config_path);
    }
    fatfs_unmount();
    return ok;
}
