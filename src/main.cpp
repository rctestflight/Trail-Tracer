//https://www.rctestflight.com/trailtracer

#include <Arduino.h>
#include <math.h>
#include <PID_v1.h>
#include "wfrx_stm32_comms.h"
#include "rctf_wfrx_status_bits.h"
#include "rctf_wfrx_exp32.h"
#include "pwm_read_rmt.h"
#include "usb_msc_fat.h"
#include <WiFi.h>
#include <esp_now.h>
#include "esp_log.h"

static const char* TAG = "wfrx";

StmTxPacket stmPacket;  // Data container for UART buffer and individual values

// Runtime values are populated from USB config/NVS during setup.
uint8_t vehicleID; // Used for ESP-NOW Telemetry. Set to the same value on receiving ESP32 
uint8_t rx_channel; // Set this to match the channel on the Trail Tracer Tx. Allows for multiple tracks to operate in close proximity
int CRUISE_THROTTLE; // The vehicle will drive at this throttle level when in reveiverless mode (press button on Rx to enter receiverless mode)
int CREEP_THROTTLE; // Throttle level used when triggered by BTx Channel 1
double throttle_expo; // Helps with fine manual throttle control for vehicles with crappy ESCs
int PWM_OUT_1_DEFAULT; // Center steering PWM value (typically 1500)
int PWM_OUT_1_MIN; // Minimum steering PWM value (typically 1000)
int PWM_OUT_1_MAX; // Maximum steering PWM value (typically 2000)
int PWM_OUT_2_DEFAULT; // Default throttle PWM value (typically 1500)
int PWM_OUT_2_MIN; // Minimum throttle PWM value (typically 1000)
int PWM_OUT_2_MAX; // Maximum throttle PWM value (typically 2000)
int PWM_OUT_3_DEFAULT; // Default PWM value for output 3 (typically used for rear steering) (typically 1500)
int PWM_OUT_3_MIN; // Minimum PWM value for output 3  (typically 1000)
int PWM_OUT_3_MAX; // Maximum PWM value for output 3 (typically 2000)
int PWM_OUT_4_DEFAULT; 
int PWM_OUT_4_MIN;
int PWM_OUT_4_MAX;
double kp; // Steering controller proportional gain (higher number = more aggressive steering)
double ki; // Steering controller integral gain (usually set to 0 or a small value, helps eliminate steady-state error but can cause instability if too high)
double kd; // Steering controller derivative gain (usually never needed)
int pid_direction; // PID direction (0 = normal, 1 = reversed)
bool enable_sensor_2; // Enable second sensor. Save and Reboot after changing
double rear_kp; // Rear steering controller proportional gain
double rear_ki; // Rear steering controller integral gain
double rear_kd; // Rear steering controller derivative gain
int rear_pid_direction; // Rear PID direction (0 = normal, 1 = reversed)
bool reverse_rear_steer_in; // Reverse rear steer input. Use this if manual RC steering results in rear wheels steering in the wrong direction 
bool use_heading; // Enable or disable use of dual sensor heading estimation for front steering (if false, only distance from wire is used for front steering). Rear steering always uses dual sensor heading estimation when enable_sensor_2 is true.
double heading_weight; // Weighting factor for heading estimation. Higher values put more weight on heading vs distance for determining steering output. If use_heading is false, this value has no effect.
double tpa; // Reduces the sensativity of steering controller as the throttle is increased. 0 is usually fine for slow vehicles, but faster vehicles that experience more severe oscillations at high speed may benefit from a higher value. A value of 0.5 means that at full throttle, the steering controller output is reduced to 50%
StuckVehicle stuckDetector(&stmPacket.items.s1_y, 15, 18000, 2000);  // Distance threshold (mm) and time threshold (milliseconds). For 1/10 scale vehicles start with 15mm and 15000ms. For 1/18 vehicles start with 10mm and 15000ms. 
uint8_t BTxFollowChannel; // If set to 0, BTx signals control creep throttle and sine swerve disable. For BTx follow functionality, set to the value of the BTx channel on the vehicle. 
double BTxAccelRate; // How fast/slow the vehicle accelerates for BTx follow
int BTxCreepDuration; // How many seconds BTx creep lasts for 
uint8_t SineSwerveAmplitude; // Setting this to 0 disables sine swerve. Non zero values specify the size of the sine swerve in millimeters. 
uint8_t SineSwerveFrequency; // frequency of the sine swerve in seconds 
double CHARGE_VBATT_STOP; // Voltage to stop charging and resume driving. usually set this to 4V per cell
double CHARGE_MAX_CURRENT; // Maximum current to draw when charging, used to prevent drawing too much current from weak chargers or when battery is very low. Usually keep this set at 6A (maximum) for most setups
int    CHARGE_MAX_SECONDS; // Maximum charge time in seconds. Vehicle will resume driving after this time even if voltage/current thresholds have not been met. Usually set this to 2-4 minutes (120-240 seconds)

// Green LED pin (Low = ON, high = OFF)
#define nLED GPIO_NUM_1

// GPIO from STM32 for new data interrupt
#define STM_nDATA_READY GPIO_NUM_8

//User button pin / BOOT pin
#define BUTTON_PIN GPIO_NUM_0

// PWM Inputs
#define PWM_IN_NUM 4
#define PWM_IN_1 GPIO_NUM_14
#define PWM_IN_2 GPIO_NUM_21
#define PWM_IN_3 GPIO_NUM_15
#define PWM_IN_4 GPIO_NUM_33

// PWM Outputs
#define PWM_OUT_1 GPIO_NUM_26
#define PWM_OUT_2 GPIO_NUM_34
#define PWM_OUT_3 GPIO_NUM_16
#define PWM_OUT_4 GPIO_NUM_13

// Spare Inputs
#define SPARE_IN_1 GPIO_NUM_9
#define SPARE_IN_2 GPIO_NUM_10

// Spare Outputs
#define SPARE_OUT_1 GPIO_NUM_11
#define SPARE_OUT_2 GPIO_NUM_12

// Spare GPIO
#define SPARE_GPIO_IO39 GPIO_NUM_39 //P18
#define SPARE_GPIO_IO40 GPIO_NUM_40 //P17

// Spare I2C/SPI, but can be as GPIO too
#define SPARE_NSS       GPIO_NUM_38
#define SPARE_SCL_SCK   GPIO_NUM_36
#define SPARE_SDA_DOUT  GPIO_NUM_37
#define SPARE_DIN       GPIO_NUM_35

// Main loop configuration
#define MAIN_LOOP_PERIOD 360000  // 1hr rollover
#define SERIAL1_FLUSH_MAX_BYTES 64  // Limit post-read drain to avoid long blocking stalls
#define SERIAL1_FLUSH_MAX_US 150  // Microsecond budget for post-read drain

// Button configuration
#define BUTTON_DEBOUNCE_PERIODS 3  // 0.03s
#define BUTTON_CLICK_MAX_LENGTH 40  // 0.4s
#define BUTTON_CAL_MIN_PRESS 200  // 2s
#define BUTTON_CAL_MAX_PRESS 400  // 4s
#define BUTTON_6S_PRESS 600  // 6s

// PWM OUT configuration
#define PWM_OUT_FREQUENCY 50  // Frequency (Hz) 50 is default
#define PWM_OUT_RESOLUTION_BITS 14  // 14-bits is maximum

// Charging Control
#define CHARGE_AUTO_ENABLE true  // bool to enable/disable automatic charging
int8_t charge_status = 0;  // charge status flag for state machine
uint32_t charge_start_millis = 0;
uint32_t last_charge_code = 0; 
uint32_t last_status_bits = 0;
bool lapFlag = false;

// Extra GPIO control
#define GPIO_39_RESTART_CYCLES 3  // number of 100Hz cycles to wait to restart (1000 = 10s)
int8_t gpio_39_status;
uint32_t gpio_39_timer;
#define GPIO_40_RESTART_CYCLES 3  // number of 100Hz cycles to wait to restart (1000 = 10s)
int8_t gpio_40_status;
uint32_t gpio_40_timer;


int vehicleStatus = 0; //0=stuck, 1=driving, 2=charging - used for telemetry 
int lastVehicleStatus = 1; // Track previous status
uint32_t stuckStartTime = 0; // Track when vehicle became stuck

uint16_t throttle_micros = 1500;
uint32_t drive_mode_timer = 0;
int8_t drive_mode_status = 0;
uint32_t led_blink_last_ms = 0;
bool led_blink_state = false;
uint16_t longpress_flash_cycles = 0;
uint32_t config_saved_led_until_ms = 0;  // When non-zero, hold LED solid-on until this timestamp
bool case2_use_cruise = true;  // Toggle output in case 2
bool case2_aux_entry_is_manual = false;  // Aux position when entering case 2
bool case2_aux_changed = false;  // Aux changed since entering case 2
int case2_throttle_current = PWM_OUT_2_DEFAULT;  // Ramped throttle in case 2
bool btx_throttle_override_active = false;  // Freeze case 1/2 throttle accumulation while BTx logic overrides output
uint32_t btx_user_override_until_ms = 0;  // While active, user input takes priority over BTx creep/follow

int throttle = PWM_OUT_2_DEFAULT;

int BTxThreshold = 200; // Threshold for BTx follow proximity. Initial Placeholder
float BTx1Signal = BTxThreshold;
float BTx2Signal = BTxThreshold;
float BTx1LastDistance = BTxThreshold;
float BTx2LastDistance = BTxThreshold;
float BTxFilterAlpha = 0.4; //0.456 = 5Hz low pass filter. Smaller value = more filtering
uint32_t sine_swerve_disabled_until_ms = 0;
double sine_wave_blend = 0.0;
bool SineSwerveEnabled = true; 
const double SineSwerveSnapRate = 0.01; // per-loop gain when sine swerve resumes
bool sine_wave_force_disabled = false;


// Steering ------------------------------------------------------
double target_x_pos = 0;  // target x distance from wire
double live_position = 0;
double rear_target_x_pos = 0;  // rear target x distance from wire
double rear_live_position = 0;
double sensor_spacing_mm;  // Distance between s1 and s2 sensors in mm
double wire_angle_deg = 0.0;  // Estimated angle of vehicle relative to wire (degrees)
// PID Initialization
double pid_output = 0.0;
double rear_pid_output = 0.0;
double steering_output = PWM_OUT_1_DEFAULT;
double rear_steering_output = PWM_OUT_3_DEFAULT;
double KP_DEFAULT = kp;
PID steeringPID(&live_position, &pid_output, &target_x_pos, kp, ki, kd, pid_direction);
PID rearSteeringPID(&rear_live_position, &rear_pid_output, &rear_target_x_pos, rear_kp, rear_ki, rear_kd, rear_pid_direction);

// PID Low pass filterint setup -----------------------------------------------------------------------------------
bool low_pass_enable = true;  // true = low pass active, false disables low-pass
float low_pass_cutoff = 5;  // Frequency of a 2nd order low-pass filter.  Smaller number = more filtering
float low_pass_alpha = 0.0;  // filter coefficient that's calculated from the cutoff value
float low_pass_beta = 0.0;  // filter coefficient that's calculated from the cutoff value
bool low_pass_compute = false;  // bool to only compute when PID loop updates
bool rear_low_pass_compute = false;  // bool to only compute when rear PID loop updates
bool low_pass_first_cycle = true;
bool rear_low_pass_first_cycle = true;
float low_pass_last_val_1 = 0.0;
float low_pass_last_val_2 = 0.0;
float rear_low_pass_last_val_1 = 0.0;
float rear_low_pass_last_val_2 = 0.0;

volatile uint32_t data_ready = 0;  // incremented by ISR each time STM32 asserts nDATA_READY
uint32_t uart_overrun_count = 0;   // diagnostic counter, incremented on each overrun recovery
uint32_t crc_error_count = 0;      // diagnostic counter, incremented on each CRC mismatch
uint16_t last_computed_crc = 0;    // last CRC computed by the ESP32, for diagnostics
uint32_t data_sent = 0;  // flag that debug UART statements have been sent
uint32_t main_loop_counter;  // counter incremented on each new STM32 data packet
int32_t bttn_counter = 0;  // button counter for debounce logic
int32_t bttn_state = 0;  // debounced state of the user button
int32_t bttn_click_counter = 0;  // button counter for "click" logic
int32_t bttn_long_counter = 0;  // button counter for "long" press logic
int32_t bttn_3s_armed = 0;  // calibration press window armed
int32_t bttn_6s_state = 0;  // state of a 6s press


void receiveStmUart();  // ISR routine to set data ready flag
void flushSerial1RxBounded();  // Drain limited stale UART bytes without blocking the loop
static uint16_t crc16_ibm(const uint8_t *data, size_t len);  // CRC-16/IBM: poly 0x8005, init 0xFFFF, no reflection
void stm32_boot_handshake();  // IAP hello/boot handshake — call once after Serial1.begin()
void writePwmMicros(uint8_t ch, uint32_t micros); // Write PWM output pulse width in microseconds
void mainLoopCounter();  // increment main loop counter
void buttonLogic();  // handle button events

// USB MSC / config management
DeviceConfig g_deviceConfig;                     // Runtime config struct (MSC + NVS backed)
void applyDeviceConfig(const DeviceConfig& cfg); // Apply cfg values to firmware globals

void buttonClickEvent();  // Called on button release if the press time is considered a "click"
void button3sPressEvent();  // Called on button release after a 2s to 4s press
void button6sPressEvent();  // Called on a 6-second button press

void chargeControl();
uint16_t driveMode();
int8_t gpio39Status();
int8_t gpio40Status();
int BTxFollow(int base_throttle, double &btx_throttle, bool &btx_throttle_init, bool &btx_follow_active, float btx_signal);

/*
Circular FIFO and control variables for handling commands sent to the STM32
Only one command can be sent per data update (100Hz)
Use the espPacketFifo_AddCmdxxx functions to add a command to the FIFO depending on payload datatype
The espPacketFifo_sendCmd function sends one command from the queue
There is no protection from overloading the FIFO, but commands are sparse so unlikely to be an issue
*/ 
#define ESP_PACKET_FIFO_LENGTH 50 // Number of commands that can be buffered
EspTxPacket espPacketFifo[ESP_PACKET_FIFO_LENGTH];  // buffer
uint32_t esp_packet_fifo_head = 0;  // index for adding data to buffer
uint32_t esp_packet_fifo_tail = 0;  // index for reading data out of buffer

// For cid, don't forget to do (cid | 0x80000000) for write commands
void espPacketFifo_addCmdInt32(uint32_t cid, int32_t cmd);  // Add a command with a int32 payload to the queue
void espPacketFifo_addCmdFloat(uint32_t cid, float cmd);  //  add a command with a float payload to the queue
void espPacketFifo_sendCmd();  // Send up to one command from the queue, call after each new STM32 packet is received

// ESP-NOW Stuff ---------------------------------------------------------------------
void broadcastTelemetry();
void telemetryFifo_addMessage(const String &message);
void telemetryFifo_sendMessage();

// Telemetry FIFO queue for ESP-NOW messages (prevents flooding)
#define TELEMETRY_FIFO_LENGTH 10
typedef struct {
  String message;
} TelemetryMessage;
TelemetryMessage telemetryFifo[TELEMETRY_FIFO_LENGTH];
uint32_t telemetry_fifo_head = 0;
uint32_t telemetry_fifo_tail = 0;
uint32_t lastTelemetrySendTime = 0;  // Track last send time for 100ms throttling

uint32_t brodcastMillis = 0;
uint32_t telemetryBrodcastMillis = 0;

void formatMacAddress(const uint8_t *macAddr, char *buffer, int maxLength) // Formats MAC Address
{
  snprintf(buffer, maxLength, "%02x:%02x:%02x:%02x:%02x:%02x", macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);
}

void receiveCallback(const uint8_t *macAddr, const uint8_t *data, int dataLen)
// Called when data is received
{
  // Only allow a maximum of 250 characters in the message + a null terminating byte
  char buffer[ESP_NOW_MAX_DATA_LEN + 1];
  int msgLen = min(ESP_NOW_MAX_DATA_LEN, dataLen);
  strncpy(buffer, (const char *)data, msgLen);

  // Make sure we are null terminated
  buffer[msgLen] = 0;

  // Format the MAC address
  char macStr[18];
  formatMacAddress(macAddr, macStr, 18);
}

void sentCallback(const uint8_t *macAddr, esp_now_send_status_t status) // Called when data is sent
{
  char macStr[18];
  formatMacAddress(macAddr, macStr, 18);
}

void broadcast(const String &message) {
  uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  esp_now_peer_info_t peerInfo = {};
  memcpy(&peerInfo.peer_addr, broadcastAddress, 6);

  // Only add the peer if it's not already added
  if (!esp_now_is_peer_exist(broadcastAddress)) {
    esp_now_add_peer(&peerInfo);
  }

  // Send message
  esp_err_t result = esp_now_send(broadcastAddress, (const uint8_t *)message.c_str(), message.length());

  // Print results to serial monitor
  if (result != ESP_OK) {
    Serial.println("ESP-NOW send failed");
  }
}


// IAP boot handshake ---------------------------------------------------------------------------------------------------------------
#define IAP_CID_HELLO             0x0001
#define IAP_CID_BOOT              0x0005
#define IAP_STATUS_OK             0x00
#define IAP_MAX_FRAME             32    // largest expected frame is 13 bytes; 32 gives headroom
#define IAP_HELLO_RETRIES         10
#define IAP_HELLO_TIMEOUT_MS      100
#define IAP_BOOT_TIMEOUT_MS       100
#define IAP_DATA_READY_TIMEOUT_MS 200

static void iap_encode_frame(uint16_t cid, uint8_t *buf) {
  // Zero-payload frame: length(2LE) + cid(2LE) + crc16_ibm(2LE)
  buf[0] = 0x06; buf[1] = 0x00;
  buf[2] = cid & 0xFF; buf[3] = (cid >> 8) & 0xFF;
  uint16_t crc = crc16_ibm(buf, 4);
  buf[4] = crc & 0xFF; buf[5] = (crc >> 8) & 0xFF;
}

static bool iap_read_frame(uint32_t timeout_ms, uint16_t *cid_out, uint8_t *payload_out, uint8_t *payload_len_out) {
  uint32_t deadline = millis() + timeout_ms;

  while (Serial1.available() < 4) { if (millis() > deadline) return false; }
  uint8_t hdr[4];
  Serial1.readBytes(hdr, 4);
  uint16_t length = hdr[0] | ((uint16_t)hdr[1] << 8);
  uint16_t cid    = hdr[2] | ((uint16_t)hdr[3] << 8);
  if (length < 6 || length > IAP_MAX_FRAME) return false;

  uint8_t rest_len = length - 4;  // payload + 2 CRC bytes
  uint8_t rest[IAP_MAX_FRAME];
  while (Serial1.available() < rest_len) { if (millis() > deadline) return false; }
  Serial1.readBytes(rest, rest_len);

  uint8_t plen = length - 6;
  uint16_t crc_recv = rest[plen] | ((uint16_t)rest[plen + 1] << 8);
  uint8_t crc_buf[4 + IAP_MAX_FRAME];
  memcpy(crc_buf, hdr, 4);
  memcpy(crc_buf + 4, rest, plen);
  if (crc16_ibm(crc_buf, 4 + plen) != crc_recv) return false;

  *cid_out = cid;
  *payload_len_out = plen;
  memcpy(payload_out, rest, plen);
  return true;
}

void stm32_boot_handshake() {
  // After a firmware upload only the ESP32 resets; the STM32 is already running
  // its application and will not respond to IAP HELLO.  Skip the handshake for
  // any reset reason other than a cold power-on so we don't restart-loop.
  esp_reset_reason_t reset_reason = esp_reset_reason();
  if (reset_reason != ESP_RST_POWERON) {
    Serial.print("STM32 handshake skipped (reset reason: ");
    Serial.print((int)reset_reason);
    Serial.println(")");
    return;
  }

  uint8_t frame[6];
  uint8_t payload[IAP_MAX_FRAME];
  uint8_t plen;
  uint16_t cid;

  // Flush, send HELLO, wait for HELLO response; retry on flush-race or garbled frame
  bool hello_ok = false;
  for (int i = 0; i < IAP_HELLO_RETRIES && !hello_ok; i++) {
    while (Serial1.available()) Serial1.read();
    iap_encode_frame(IAP_CID_HELLO, frame);
    Serial1.write(frame, 6);
    if (!iap_read_frame(IAP_HELLO_TIMEOUT_MS, &cid, payload, &plen)) continue;
    if (cid != IAP_CID_HELLO) continue;
    hello_ok = true;
  }
  if (!hello_ok) {
    Serial.println("IAP HELLO failed");
    return;  // STM32 may already be running; continue without handshake
  }

  // BOOT — STM32 state confirmed by HELLO, single attempt
  iap_encode_frame(IAP_CID_BOOT, frame);
  Serial1.write(frame, 6);
  if (!iap_read_frame(IAP_BOOT_TIMEOUT_MS, &cid, payload, &plen) ||
      cid != IAP_CID_BOOT || plen < 1 || payload[0] != IAP_STATUS_OK) {
    Serial.println("IAP BOOT failed");
    return;
  }

  // Wait for STM32 app to assert nDATA_READY for the first time
  uint32_t deadline = millis() + IAP_DATA_READY_TIMEOUT_MS;
  while (digitalRead(STM_nDATA_READY) != LOW) {
    if (millis() > deadline) {
      Serial.println("IAP nDATA_READY timeout");
      return;
    }
  }

  Serial.println("STM32 boot OK");
}

// Void Setup --------------------------------------------------------------------------------------------------------------
void setup() {

  // Initialise USB MSC + FAT filesystem and load config.
  msc_fat_init(&g_deviceConfig);
  applyDeviceConfig(g_deviceConfig);

  // Start Serial
  Serial.begin(115200);  // Start USB VCP
  Serial.setTxTimeoutMs(0);  // Never block if no CDC host is connected
  Serial0.setTxBufferSize(1024);  // Prevents log writes from blocking at 1 Mbaud
  Serial0.begin(1000000);  // Start debug UART (visible on tag-connect)
  Serial1.setRxBufferSize(2048);  // ~176 ms headroom at 100 Hz × 116 bytes/packet
  Serial1.begin(2000000);  // Start ESP32 <-> STM32 UART
  pinMode(STM_nDATA_READY, INPUT_PULLUP);


  // Configure pins
  pinMode(nLED, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  pinMode(PWM_IN_1, INPUT);
  pinMode(PWM_IN_2, INPUT);
  pinMode(PWM_IN_3, INPUT);
  pinMode(PWM_IN_4, INPUT);

  pinMode(PWM_OUT_1, OUTPUT);
  pinMode(PWM_OUT_2, OUTPUT);
  pinMode(PWM_OUT_3, OUTPUT);
  pinMode(PWM_OUT_4, OUTPUT);

  pinMode(SPARE_GPIO_IO39, INPUT_PULLDOWN);
  pinMode(SPARE_GPIO_IO40, INPUT_PULLDOWN);

  digitalWrite(nLED, 1);  // turn the LED off

  // Servo PWM setup
  ledcSetup(1, PWM_OUT_FREQUENCY, PWM_OUT_RESOLUTION_BITS);
  ledcSetup(2, PWM_OUT_FREQUENCY, PWM_OUT_RESOLUTION_BITS);
  ledcSetup(3, PWM_OUT_FREQUENCY, PWM_OUT_RESOLUTION_BITS);
  ledcSetup(4, PWM_OUT_FREQUENCY, PWM_OUT_RESOLUTION_BITS);
  ledcAttachPin(PWM_OUT_1, 1);
  ledcAttachPin(PWM_OUT_2, 2);
  ledcAttachPin(PWM_OUT_3, 3);
  ledcAttachPin(PWM_OUT_4, 4);
  writePwmMicros(1, PWM_OUT_1_DEFAULT);
  writePwmMicros(2, PWM_OUT_2_DEFAULT);
  writePwmMicros(3, PWM_OUT_3_DEFAULT);
  writePwmMicros(4, PWM_OUT_4_DEFAULT);

  // Set up PWM inputs (in progress)
  uint8_t pwm_input_pins[] = {PWM_IN_1, PWM_IN_2, PWM_IN_3, PWM_IN_4};
  pwm_read_rmt_init(pwm_input_pins, PWM_IN_NUM);

  // PID setup
  steeringPID.SetOutputLimits(PWM_OUT_1_MIN - PWM_OUT_1_DEFAULT, PWM_OUT_1_MAX - PWM_OUT_1_DEFAULT);  // Set the output contstraints
  steeringPID.SetSampleTime(10);  // Set the sample interval in milliseconds
  steeringPID.SetControllerDirection(pid_direction);  // Apply direction loaded from config
  steeringPID.SetMode(AUTOMATIC);  // Turn on the control loop
  rearSteeringPID.SetOutputLimits(PWM_OUT_3_MIN - PWM_OUT_3_DEFAULT, PWM_OUT_3_MAX - PWM_OUT_3_DEFAULT);  // Set rear output constraints
  rearSteeringPID.SetSampleTime(10);  // Set the sample interval in milliseconds
  rearSteeringPID.SetControllerDirection(rear_pid_direction);  // Apply direction loaded from config
  rearSteeringPID.SetMode(AUTOMATIC);  // Turn on the rear control loop

  // PID low pass setup
  low_pass_alpha = pow(2.71828183, (-1.0 / (100 * 1/(low_pass_cutoff * 6.2831))));
  low_pass_beta = 1 - low_pass_alpha;

  // Attach falling edge interrupt to STM_nDATA_READY pin
  attachInterrupt(digitalPinToInterrupt(STM_nDATA_READY), receiveStmUart, FALLING);
  pwm_reset_readings();

  stm32_boot_handshake();

  data_ready = 0;  // Clear any pending data ready flags that may have been set during boot

  // Set up charge port
  espPacketFifo_addCmdFloat(6 | 0x80000000, CHARGE_VBATT_STOP);  // Set voltage stop
  espPacketFifo_addCmdFloat(7 | 0x80000000, CHARGE_MAX_CURRENT);  // Set overcurrent
  espPacketFifo_addCmdInt32(8 | 0x80000000, (CHARGE_MAX_SECONDS * 2));  // Set charge time limit

  //RX channel setup
  espPacketFifo_addCmdInt32(1 | 0x80000000, (rx_channel - 1));  // write Rx channel
  espPacketFifo_addCmdInt32(2 | 0x80000000, enable_sensor_2 ? 0x00 : 0x80);  // Enable/disable sensor 2
  espPacketFifo_addCmdInt32(3 | 0x80000000, 0);  // Save new channel value

  if (g_deviceConfig.espnow_enabled) {
    //ESP-NOW Setup ----------------------------------------------------------------
    WiFi.mode(WIFI_STA); // Set device as a Wi-Fi Station

    // Initialize ESP-NOW
   if (esp_now_init() == ESP_OK) {
     Serial.println("ESP-NOW Init Success");
     esp_now_register_recv_cb(receiveCallback);  // Register receive callback
     esp_now_register_send_cb(sentCallback);     // Register send callback
   } else {
     Serial.println("ESP-NOW Init Failed");
     delay(1000);
     ESP.restart();
   }
  }
}

// Main Loop ---------------------------------------------------------------------------------------------------------------
void loop() {

  // Check if USB MSC host has finished editing config.json (3 s idle window).
  if (msc_fat_loop(&g_deviceConfig)) {
    applyDeviceConfig(g_deviceConfig);
    config_saved_led_until_ms = millis() + 1500;  // Signal config saved to user via LED
  }

  static uint32_t loop_min_us = 0xFFFFFFFF;
  static uint32_t loop_max_us = 0;
  static uint32_t loop_exec_count = 0;

  if(data_ready){
    uint32_t loop_start_us = micros();
    // Overrun detection: ISR fired more than once, or extra bytes are buffered from a
    // previous cycle.  Drain and skip this cycle; the next interrupt will bring a
    // fresh, complete packet that is guaranteed to be aligned.
    if(data_ready > 1 || Serial1.available() > (int)sizeof(stmPacket.uart_data)){
      while(Serial1.available()) Serial1.read();  // drain stale bytes first
      data_ready = 0;                             // then reset (ISR during drain → clean next cycle)
      uart_overrun_count++;
      // Fall through to the rest of loop() without touching stmPacket.
    } else {
    // Normal path: exactly one interrupt fired and one full packet is ready.
    Serial1.readBytes(stmPacket.uart_data, sizeof(stmPacket.uart_data));
    data_ready = 0;

    // Make sure any stray data is flushed allowing the next message to sync up
    flushSerial1RxBounded();

    // CRC covers every byte except the trailing crc field itself.
    uint16_t computed_crc = crc16_ibm(stmPacket.uart_data, sizeof(stmPacket.uart_data) - sizeof(stmPacket.items.crc));
    last_computed_crc = computed_crc;
    if(computed_crc != stmPacket.items.crc){
      crc_error_count++;
    } else {

    // update the double X position from the float in the latest STM32 packet
    live_position = stmPacket.items.s1_x;
    rear_live_position = stmPacket.items.s2_x;
    double sensor_dx_mm = rear_live_position - live_position;
    wire_angle_deg = atan2(sensor_dx_mm, sensor_spacing_mm) * (180.0 / 3.14159265358979323846);
    if(use_heading){
      live_position = live_position + heading_weight * wire_angle_deg;
    }
    rear_live_position = wire_angle_deg;

    uint32_t loop_now_ms = millis();

    // Filter and condition BTx signals
    BTx1Signal = constrain(stmPacket.items.s1_btx_radius[0], 10, (BTxThreshold + 2));
    BTx2Signal = constrain(stmPacket.items.s1_btx_radius[1], 10, (BTxThreshold + 2));
    BTx1Signal = BTxFilterAlpha * BTx1Signal + (1 - BTxFilterAlpha) * BTx1LastDistance;
    BTx2Signal = BTxFilterAlpha * BTx2Signal + (1 - BTxFilterAlpha) * BTx2LastDistance;
    BTx1LastDistance = BTx1Signal;
    BTx2LastDistance = BTx2Signal;

    if(BTx2Signal < BTxThreshold && BTxFollowChannel == 0){
      sine_swerve_disabled_until_ms = loop_now_ms + 10000U;
    }

    // Reset target each loop so optional modifiers can add on cleanly
    target_x_pos = 0.0;
    rear_target_x_pos = 0.0;

    // Sine wave steering component
    bool sine_wave_enabled = SineSwerveEnabled && (SineSwerveAmplitude != 0);
    if(sine_wave_enabled){
      bool sine_wave_allowed = (loop_now_ms >= sine_swerve_disabled_until_ms) && !sine_wave_force_disabled;
      double blend_target = sine_wave_allowed ? 1.0 : 0.0;
      if(sine_wave_blend < blend_target){
        sine_wave_blend = fmin(blend_target, sine_wave_blend + SineSwerveSnapRate);
      }else if(sine_wave_blend > blend_target){
        sine_wave_blend = fmax(blend_target, sine_wave_blend - SineSwerveSnapRate);
      }

      double time_sec = loop_now_ms / 1000.0;
      double sine_offset = (SineSwerveAmplitude) * sin((2.0 * 3.14159 / SineSwerveFrequency) * time_sec);
      target_x_pos = sine_wave_blend * sine_offset;
    }else{
      sine_wave_blend = 0.0;
    }

    // Use PWM input 4 to control the proportional gain of the steering controller 
    uint32_t kp_input_pwm = pwm_read_rmt_dur(3);
    double kp_target = KP_DEFAULT;
    if(kp_input_pwm >= 800){
      int kp_input_clamped = constrain((int)kp_input_pwm, 1000, 2000);
      kp_target = 1.0 + (double)(kp_input_clamped - 1000) * 0.007;
      kp_target = round(kp_target * 15.0) / 15.0;
    }
    if(kp != kp_target){
      kp = kp_target;
      steeringPID.SetTunings(kp, ki, kd);
    }

    // Steering control
    low_pass_compute = steeringPID.Compute();  // compute new steering command according to straight PID
    rear_low_pass_compute = rearSteeringPID.Compute();  // compute new rear steering command

    // apply low pass to PID output (mostly to reduce jitter due to kd term)
    if(low_pass_compute && low_pass_enable && (!low_pass_first_cycle)){
      pid_output = pid_output * low_pass_beta + low_pass_last_val_1 * low_pass_alpha;  // Compute first low-pass
      low_pass_last_val_1 = pid_output;  // save intermediate value
      pid_output = pid_output * low_pass_beta + low_pass_last_val_2 * low_pass_alpha;  // Compute second low-pass
      low_pass_last_val_2 = pid_output;  // save last value
    }
    if(low_pass_compute && low_pass_first_cycle && low_pass_enable){
      low_pass_last_val_1 = pid_output;
      low_pass_last_val_2 = pid_output;
      low_pass_first_cycle = false;
    }
    if(rear_low_pass_compute && low_pass_enable && (!rear_low_pass_first_cycle)){
      rear_pid_output = rear_pid_output * low_pass_beta + rear_low_pass_last_val_1 * low_pass_alpha;  // Compute first low-pass
      rear_low_pass_last_val_1 = rear_pid_output;  // save intermediate value
      rear_pid_output = rear_pid_output * low_pass_beta + rear_low_pass_last_val_2 * low_pass_alpha;  // Compute second low-pass
      rear_low_pass_last_val_2 = rear_pid_output;  // save last value
    }
    if(rear_low_pass_compute && rear_low_pass_first_cycle && low_pass_enable){
      rear_low_pass_last_val_1 = rear_pid_output;
      rear_low_pass_last_val_2 = rear_pid_output;
      rear_low_pass_first_cycle = false;
    }

    throttle = driveMode(); // get throttleMicros from drive mode logic

    if(charge_status > 0){
      stuckDetector.ResetStatus();  // ignore stuck detection when charging
    }

    if(gpio39Status()){
      throttle = CREEP_THROTTLE;
    }

    //Btx Logic -----------------------------------------------------------------------
    // Creep Speed -------------------------------
    bool btx_user_override_active = (millis() < btx_user_override_until_ms);

    if(BTxFollowChannel == 0){ //if BTx follow is turned off, use BTx 0 for creep speed
      int base_throttle = throttle;
      static double creep_throttle = PWM_OUT_2_DEFAULT;
      static bool creep_throttle_init = false;
      static uint32_t creep_until_ms = 0;
      static bool creep_returning = false;
      static bool creep_was_active = false;
      static int creep_return_base = PWM_OUT_2_DEFAULT;
      if(!creep_throttle_init){
        creep_throttle = base_throttle;
        creep_throttle_init = true;
      }

      if(btx_user_override_active || drive_mode_status == 0){
        creep_until_ms = 0;
        creep_returning = false;
        creep_was_active = false;
        creep_throttle = base_throttle;
        throttle = base_throttle;
        btx_throttle_override_active = false;
      }else{

      bool creep_triggered = (BTx1Signal < BTxThreshold
        && ((drive_mode_status == 2 && case2_use_cruise)
        || (drive_mode_status == 1 && (throttle > CREEP_THROTTLE))));

      if(creep_triggered){ 
        creep_until_ms = millis() + (uint32_t)BTxCreepDuration * 1000U;
        creep_returning = false;
      }
      bool creep_active = (creep_triggered || (millis() < creep_until_ms)); // Enter creep if triggered by BTx

      if(!creep_active && creep_was_active){
        creep_return_base = base_throttle;
        creep_returning = true;
      }
      creep_was_active = creep_active;

      if(creep_active){
        double target_throttle = CREEP_THROTTLE;
        double step = fabs(BTxAccelRate);
        creep_throttle = (step <= 0.0) ? target_throttle
          : (creep_throttle < target_throttle) ? fmin(creep_throttle + step, target_throttle)
          : (creep_throttle > target_throttle) ? fmax(creep_throttle - step, target_throttle)
          : creep_throttle;
        throttle = (int)lround(creep_throttle);
      }else if(creep_returning){
        double target_throttle = creep_return_base;
        double step = fabs(BTxAccelRate);
        creep_throttle = (step <= 0.0) ? target_throttle
          : (creep_throttle < target_throttle) ? fmin(creep_throttle + step, target_throttle)
          : (creep_throttle > target_throttle) ? fmax(creep_throttle - step, target_throttle)
          : creep_throttle;
        throttle = (int)lround(creep_throttle);
        if((int)lround(creep_throttle) == creep_return_base){
          creep_returning = false;
        }
      }else{
        creep_throttle = base_throttle;
        throttle = base_throttle;
      }

      btx_throttle_override_active = (creep_active || creep_returning);
      }

    }
    //BTx follow feature for BTx channel 1 --------------------------
    if(BTxFollowChannel == 1){ //if BTx follow ch 1 is selected, do the follow behavior. 
      int base_throttle = throttle;
      static double btx1_throttle = PWM_OUT_2_DEFAULT;
      static bool btx1_throttle_init = false;
      static bool btx1_follow_active = false;
      if(btx_user_override_active){
        btx1_throttle = base_throttle;
        btx1_follow_active = false;
        throttle = base_throttle;
        btx_throttle_override_active = false;
      }else{
        throttle = BTxFollow(base_throttle, btx1_throttle, btx1_throttle_init, btx1_follow_active, BTx1Signal);
        btx_throttle_override_active = btx1_follow_active;
      }
    }
    //BTx follow feature for BTx channel 2 ---------------------------
    if(BTxFollowChannel == 2){ //if BTx follow ch 2 is selected, do the follow behavior. 
      int base_throttle = throttle;
      static double btx2_throttle = PWM_OUT_2_DEFAULT;
      static bool btx2_throttle_init = false;
      static bool btx2_follow_active = false;
      if(btx_user_override_active){
        btx2_throttle = base_throttle;
        btx2_follow_active = false;
        throttle = base_throttle;
        btx_throttle_override_active = false;
      }else{
        throttle = BTxFollow(base_throttle, btx2_throttle, btx2_throttle_init, btx2_follow_active, BTx2Signal);
        btx_throttle_override_active = btx2_follow_active;
      }
    }

    if(throttle >= 1470 && throttle <= 1530){ //Reset stuck detector when throttle is low
      stuckDetector.ResetStatus();  
    }

    throttle = constrain(throttle, PWM_OUT_2_MIN, PWM_OUT_2_MAX);

    // In manual mode, disable PID steering contribution when user steering is beyond +/-100us from center.
    int manual_steer_offset = 0;
    int rear_manual_steer_offset = 0;
    double pid_steer_contribution = pid_output;
    double rear_pid_steer_contribution = rear_pid_output;

    // TPA: scale PID output down proportionally with throttle between (PWM_OUT_2_DEFAULT + 30) and PWM_OUT_2_MAX
    if(tpa > 0.0){
      const int tpa_start = PWM_OUT_2_DEFAULT + 30;
      const int tpa_range = PWM_OUT_2_MAX - tpa_start;
      if(tpa_range > 0 && throttle > tpa_start){
        double throttle_fraction = (double)constrain(throttle - tpa_start, 0, tpa_range) / (double)tpa_range;
        double tpa_scale = fmax(0.0, 1.0 - tpa * throttle_fraction);
        pid_steer_contribution *= tpa_scale;
        rear_pid_steer_contribution *= tpa_scale;
      }
    }

    if(drive_mode_status == 0 ){  // Manual mode
      uint32_t steering_input = pwm_read_rmt_dur(0);
      bool steering_valid = (steering_input >= 500 && steering_input <= 2500);
      if(steering_valid){
        manual_steer_offset = (int)steering_input - 1500;
        rear_manual_steer_offset = reverse_rear_steer_in ? -manual_steer_offset : manual_steer_offset;
        if(abs(manual_steer_offset) >= 100){
          pid_steer_contribution = 0.0;
          rear_pid_steer_contribution = 0.0;
        }
      }
    }

    steering_output = PWM_OUT_1_DEFAULT + pid_steer_contribution + manual_steer_offset; //calculate steering output
    rear_steering_output = PWM_OUT_3_DEFAULT + rear_pid_steer_contribution + rear_manual_steer_offset;
    steering_output = constrain(steering_output, PWM_OUT_1_MIN, PWM_OUT_1_MAX); //Keep steering output within limits
    rear_steering_output = constrain(rear_steering_output, PWM_OUT_3_MIN, PWM_OUT_3_MAX); //Keep rear steering output within limits

    auto inverse_rear_steer_to_pwm4 = [](double rear_output)->uint32_t {
      // Mirror rear steering command about center for PWM_OUT_4.
      double pwm4_value = PWM_OUT_4_DEFAULT - (rear_output - PWM_OUT_3_DEFAULT);
      return (uint32_t)constrain((int)lround(pwm4_value), PWM_OUT_4_MIN, PWM_OUT_4_MAX);
    };


     // Check for faults and write servo values. Will cut throttle and stop steering if one or more faults are present
     if( WFRX_STATUS_GET_BIT(stmPacket.items.status_bits, WFRX_STATUS_S1_SIGNAL_OVERLOAD)  //  signal overload fault bit   
      || WFRX_STATUS_GET_BIT(stmPacket.items.status_bits, WFRX_STATUS_VDD_FAULT)  // STM32 VDD fault bit
      || (stmPacket.items.temperature > 100)  // STM32 internal temp in °C
      ){ 
        writePwmMicros(1, PWM_OUT_1_DEFAULT);  // write center steering value
        writePwmMicros(2, PWM_OUT_2_DEFAULT);  // Cut throttle on any of the above faults.
        writePwmMicros(3, PWM_OUT_3_DEFAULT);  // write center steering value to reversed output 3
        writePwmMicros(4, PWM_OUT_4_DEFAULT);
       
     }else if((drive_mode_status == 1 || drive_mode_status == 2) //if in wire follow modes, stop for these reasons
       && (WFRX_STATUS_GET_BIT(stmPacket.items.status_bits, WFRX_STATUS_S1_SIGNAL_LOST)  // signal lost fault bit
          || stuckDetector.GetStatus()  // stuck vehicle status bit
          || (charge_status > 0)  // vehicle charge in progress
          || gpio40Status()) // GPIO 40 pulled high
          ){ 
        writePwmMicros(2, PWM_OUT_2_DEFAULT);  // Cut throttle on any of the above faults.
        sine_wave_force_disabled = true; sine_wave_blend = 0.0; target_x_pos = 0.0; rear_target_x_pos = 0.0;
        vehicleStatus = 2; //set status to 'charging' for telemetry purposes
        if (charge_status == 0) { // If there is a fault other than charging
         vehicleStatus = 0; //the vehicle should be considered stuck
         }

     }else{
       writePwmMicros(2, throttle);  // write throttle value to throttle output
       writePwmMicros(1, steering_output);  // write steering value to steering output
       writePwmMicros(3, rear_steering_output);  // write inverse steering value to output 3 
       writePwmMicros(4, inverse_rear_steer_to_pwm4(rear_steering_output));
       sine_wave_force_disabled = false;
       vehicleStatus = 1; //set status to driving for telemetry purposes
     }

    // LED blink for stuckDetector
    static uint32_t stuck_led_last_ms = 0;
    if (stuckDetector.GetStatus()) {
      uint32_t now = millis();
      if ((now - stuck_led_last_ms) < 7) {
        digitalWrite(nLED, 0); // LED ON (Low = ON)
      } else {
        digitalWrite(nLED, 1); // LED OFF
      }
      if ((now - stuck_led_last_ms) >= 250) {
        stuck_led_last_ms = now;
      }
    }

  mainLoopCounter();  // increment data loop counter
  buttonLogic();  // get button actions
  chargeControl();  // monitor and control charge port
  espPacketFifo_sendCmd();  // Send one STM32 UART command from the command queue
  gpio39Status();  // Monitor GPIO 39 and update status
  gpio40Status();  // Monitor GPIO 40 and update status

  }  // end else (CRC pass)
  }  // end else (normal packet path)
    uint32_t loop_elapsed_us = micros() - loop_start_us;
    if(loop_elapsed_us < loop_min_us) loop_min_us = loop_elapsed_us;
    if(loop_elapsed_us > loop_max_us) loop_max_us = loop_elapsed_us;
    loop_exec_count++;
  }  // end if(data_ready)
  if (g_deviceConfig.espnow_enabled) {
    broadcastTelemetry();  // Queue telemetry message if needed
    telemetryFifo_sendMessage();  // Send one telemetry message from queue (throttled to 100ms)
  }

    if(longpress_flash_cycles > 0){ // Flash LED on long-press - sensor calibration
      led_blink_state = !led_blink_state;
      digitalWrite(nLED, led_blink_state ? 0 : 1);  // Low = ON, High = OFF
      longpress_flash_cycles--;
    } else if (config_saved_led_until_ms && millis() < config_saved_led_until_ms) {
      digitalWrite(nLED, 0);  // Solid ON — config saved
    } else {
      config_saved_led_until_ms = 0;  // Clear once expired
    }

  // Serial Print debug data to the USB virtual COM port
  // Schedule some UART debug prints, making sure to not repeat in the same millisecond
  uint32_t interval = 100; // interval between prints in milliseconds
  if(millis() % interval == 0 && !data_sent){

    // Serial printing to be enabled in production code. Batched printf — single USB transaction, non-blocking (setTxTimeoutMs(0) set at boot):
    Serial.printf(" Ch1:%d Ch2:%d Ch3:%d Thr:%d Str:%g X1:%g Y1:%g X2:%g Y2:%g Angle:%.0f BTx1:%g BTx2:%g\n",
      pwm_read_rmt_dur(0), pwm_read_rmt_dur(1), pwm_read_rmt_dur(2),
      throttle, steering_output,
      stmPacket.items.s1_x, stmPacket.items.s1_y,
      stmPacket.items.s2_x, stmPacket.items.s2_y,
      wire_angle_deg,
      stmPacket.items.s1_btx_radius[0], stmPacket.items.s1_btx_radius[1]);

    // Individual prints kept for debugging — uncomment as needed:
    //Serial.print(" Ch1: "); Serial.print(pwm_read_rmt_dur(0)); // R/C PWM input 1 (steering input from receiver)
    //Serial.print(" Ch2: "); Serial.print(pwm_read_rmt_dur(1)); // R/C PWM input 2 (throttle input from receiver)
    //Serial.print(" Ch3: "); Serial.print(pwm_read_rmt_dur(2)); // R/C PWM input 3 (auxiliary input from receiver)
    //Serial.print(" Ch4: "); Serial.print(pwm_read_rmt_dur(3)); // R/C PWM input 4 (unused)
    //Serial.print(" Thr Out: "); Serial.print(throttle);
    //Serial.print(" Str Out: "); Serial.print(steering_output);
    //Serial.print("  Rear Steering Output: "); Serial.print(rear_steering_output);
    //Serial.print("  PID Output: "); Serial.print(pid_output);
    //Serial.printf("Drive Mode:  %12d\n\r", drive_mode_status);
    //Serial.print("gpio_39: "); Serial.print(gpio_39_status);
    //Serial.print("  gpio_40: "); Serial.print(gpio_40_status);
    //Serial.print(" BTx1: "); Serial.print(BTx1Signal);
    //Serial.print(" BTx2: "); Serial.print(BTx2Signal);
    //Serial.print(" X1: "); Serial.print(stmPacket.items.s1_x);
    //Serial.print(" Y1: "); Serial.print(stmPacket.items.s1_y);
    //Serial.print(" R1: "); Serial.print(stmPacket.items.s1_radius);
    //Serial.print("Channel: "); Serial.print(rx_channel);  // Rx board channel selected
    //Serial.print(" X2: "); Serial.print(stmPacket.items.s2_x);
    //Serial.print(" Y2: "); Serial.print(stmPacket.items.s2_y);
    //Serial.print(" R2: "); Serial.print(stmPacket.items.s2_radius);
    //Serial.print(" Angle: "); Serial.print(wire_angle_deg, 0); // in degrees
    //Serial.print(" BTx1: "); Serial.print(stmPacket.items.s1_btx_radius[0]); // Sensor 1 BTx raw
    //Serial.print(" BTx2: "); Serial.print(stmPacket.items.s1_btx_radius[1]); // Sensor 1 BTx raw
    //Serial.print("S2 BTx1: "); Serial.print(stmPacket.items.s2_btx_radius[0]); // Sensor 2 BTx raw
    //Serial.print("S2 BTx2: "); Serial.print(stmPacket.items.s2_btx_radius[1]); // Sensor 2 BTx raw
    //Serial.print("  target_x_pos : "); Serial.print(target_x_pos);
    //Serial.printf("5V:       %12.4f\n\r", stmPacket.items.v5v);  // 5V input voltage to Rx board
    //Serial.printf("Vbatt:    %12.4f V\n\r", stmPacket.items.vbatt);  // vbatt
    //Serial.printf("Vpsu:     %12.4f V\n\r", stmPacket.items.vcharge);  // PSU input voltage
    //Serial.printf("Icharge:  %12.4f A\n\r", stmPacket.items.icharge);  // Charge current
    //Serial.printf("Chrg stat:%12d\n\r", (charge_status));  // charge status
    //Serial.printf("Chrg cool:%12d\n\r", (WFRX_STATUS_GET_BIT(stmPacket.items.status_bits, WFRX_STATUS_CP_COOLDOWN)));  // charge cooldown
    //Serial.printf("Chrg Code:%12lu\n\r", (stmPacket.items.charge_code));  // charge code
    // Serial.printf("Re(S1C1): %12.4f\n\r", (stmPacket.items.coil_data[0][0]));
    // Serial.printf("Im(S1C1): %12.4f\n\r", (stmPacket.items.coil_data[0][1]));
    // Serial.printf("Re(S1C2): %12.4f\n\r", (stmPacket.items.coil_data[0][2]));
    // Serial.printf("Im(S1C2): %12.4f\n\r", (stmPacket.items.coil_data[0][3]));
    // Serial.printf("Re(S2C1): %12.4f\n\r", (stmPacket.items.coil_data[1][0]));
    // Serial.printf("Im(S2C1): %12.4f\n\r", (stmPacket.items.coil_data[1][1]));
    // Serial.printf("Re(S2C2): %12.4f\n\r", (stmPacket.items.coil_data[1][2]));
    // Serial.printf("Im(S2C2): %12.4f\n\r", (stmPacket.items.coil_data[1][3]));

    data_sent = 1;
  }
  if(millis() % interval >= 1){
    data_sent = 0;  // reset flag to allow new data to send
  }

  /*
  // Status log at ~1 s intervals (compatible with ESP-IDF log format)
  static uint32_t dbg_last_ms = 0;
  uint32_t dbg_now = millis();
  if(dbg_now - dbg_last_ms >= 1000){
    dbg_last_ms = dbg_now;
    ESP_LOGI(TAG, "lp:%lu mode:%d ovr:%lu crc_err:%lu x1:%.1f r1:%.1f thr:%d chrg:%d sb:0x%04lX",
      main_loop_counter,
      drive_mode_status,
      uart_overrun_count,
      crc_error_count,
      (float)stmPacket.items.s1_x,
      (float)stmPacket.items.s1_radius,
      throttle,
      charge_status,
      (unsigned long)stmPacket.items.status_bits);
    if(loop_exec_count > 0){
      ESP_LOGI(TAG, "loop_us: cnt:%lu min:%lu max:%lu",
        loop_exec_count, loop_min_us, loop_max_us);
    }
    loop_min_us = 0xFFFFFFFF;
    loop_max_us = 0;
    loop_exec_count = 0;
    // ESP_LOGI(TAG, "crc: calc=0x%04X stored=0x%04X %s",
    //   last_computed_crc,
    //   (uint16_t)stmPacket.items.crc,
    //   (last_computed_crc == stmPacket.items.crc) ? "OK" : "FAIL");
  }
  */

} // End of main loop -------------------------------------------------------------------------

// STM32 UART ISR, triggered by falling edge of nDATA_READY after each full packet
void IRAM_ATTR receiveStmUart(){
  data_ready++;
}

void flushSerial1RxBounded(){
  uint32_t start_us = micros();
  uint16_t flushed = 0;
  while(Serial1.available() && (flushed < SERIAL1_FLUSH_MAX_BYTES)){
    Serial1.read();
    flushed++;
    if((micros() - start_us) >= SERIAL1_FLUSH_MAX_US){
      break;
    }
  }
}

// CRC-16/IBM: poly 0x8005, init 0xFFFF, no input/output reflection, no final XOR.
static uint16_t crc16_ibm(const uint8_t *data, size_t len){
  uint16_t crc = 0xFFFF;
  for(size_t i = 0; i < len; i++){
    crc ^= (uint16_t)data[i] << 8;
    for(int b = 0; b < 8; b++){
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x8005 : (crc << 1);
    }
  }
  return crc;
}

// write a PWM output channel's on-time in microseconds
void writePwmMicros(uint8_t ch, uint32_t micros){
  uint16_t duty_cycle = (int32_t)((float)micros * 1e-6 * (float)(PWM_OUT_FREQUENCY * ((1 << PWM_OUT_RESOLUTION_BITS) - 1)));
  ledcWrite(ch, duty_cycle);
}

// Increment the main loop counter
void mainLoopCounter(){
  main_loop_counter++;
  if(main_loop_counter == MAIN_LOOP_PERIOD){
    main_loop_counter = 0;
  }
}

// Apply a centre-preserving exponential curve to throttle PWM input.
static uint32_t applyThrottleExpoPwm(uint32_t pwm_in){
  int clamped = constrain((int)pwm_in, 1000, 2000);
  double x = ((double)clamped - 1500.0) / 500.0;  // [-1, +1]
  double expo = constrain(throttle_expo, 0.0, 1.0);
  double y = ((1.0 - expo) * x) + (expo * x * x * x);
  int shaped = (int)lround(1500.0 + (y * 500.0));
  return (uint32_t)constrain(shaped, 1000, 2000);
}

// Cruise Control logic ---------------------------------------------------------
uint16_t driveMode(){
  // Passthrough input throttle command until PWM_IN_3 goes above 1500µs for 500ms, then latch on to the last throttle command. 
  // Incriment and decriment the latched throttle level based on the input from throttle input.  
  // Return to passthrough mode as soon as PWM_IN_3 goes to 500 - 1500µs.  This should allow for the RC Tx to be turned off

  uint32_t steering_input = pwm_read_rmt_dur(0);  // read steering input
  uint32_t throttle_input_raw_pwm = pwm_read_rmt_dur(1);  // raw throttle PWM input
  uint32_t aux_input = pwm_read_rmt_dur(2);  // read aux channel input
  bool steering_valid = (steering_input >= 500 && steering_input <= 2500);  // check if steering is in valid range
  bool throttle_valid = (throttle_input_raw_pwm >= 500 && throttle_input_raw_pwm <= 2500);  // check if throttle is in valid range
  bool aux_valid = (aux_input >= 500 && aux_input <= 2500);  // check if aux channel is in valid range
  uint32_t throttle_input_pwm = throttle_valid ? applyThrottleExpoPwm(throttle_input_raw_pwm) : 1500;
  
  // Start in autonomous mode when no throttle PWM is present at startup
  static bool startup_checked = false;
  static uint32_t startup_start_ms = 0;
  if(!startup_checked){
    if(startup_start_ms == 0){
      startup_start_ms = millis();
    }
    startup_checked = true;
    if(throttle_valid){ // if throttle input is valid at startup
      drive_mode_status = 0; // start in manual mode
    }else if(millis() - startup_start_ms > 500){ // wait briefly for valid throttle
      drive_mode_status = 2;
      case2_use_cruise = false; // keep throttle neutral on boot into case 2
      case2_throttle_current = PWM_OUT_2_DEFAULT;
    }else{
      startup_checked = false; // keep waiting for a valid throttle signal
    }
  }
  
  // Ensure throttle is neutral whenever entering case 2
  static int8_t last_drive_mode_status = 0;
  if(drive_mode_status == 2 && last_drive_mode_status != 2){
    case2_throttle_current = PWM_OUT_2_DEFAULT;
  }
  last_drive_mode_status = drive_mode_status;

  switch (drive_mode_status){
    case 0:  // Throttle passthrough (Manual mode), waiting for aux channel trigger
      if(aux_input > 1450 && steering_valid){ //using 1450 instead of 1500 for compatablity with 3 pos switches
        drive_mode_timer ++;
      }else{
        drive_mode_timer = 0;
      }

      if(drive_mode_timer > 50){ // switch to cruise control mode after 50 cycles (500ms)
        drive_mode_status = 1;
      }

      if(!throttle_valid){  // verify throttle is in range
        throttle_input_pwm = 1500;
      }

      throttle_micros = throttle_input_pwm;
      
      if(millis() - led_blink_last_ms >= 50){
        led_blink_last_ms = millis();
        led_blink_state = !led_blink_state;
        digitalWrite(nLED, led_blink_state ? 0 : 1);  // Low = ON, High = OFF
      }
      return throttle_micros;  // passthrough throttle
      break;

    case 1:  // cruise control enabled
      if(aux_valid && steering_valid && aux_input < 1450){ //looking for aux channel to go to 500 - 1500µs
        drive_mode_status = 0;
      }

      // Manual throttle stick input takes priority over BTx creep/follow.
      if(throttle_valid && ((throttle_input_pwm > 1550) || (throttle_input_pwm < 1450 && throttle_input_pwm > 800))){
        btx_user_override_until_ms = millis() + 300;
      }

      if(!btx_throttle_override_active){
        if(throttle_input_pwm > 1550 && throttle_micros < PWM_OUT_2_MAX)  {  // Accelerate
          throttle_micros++;
        }else if(throttle_input_pwm < 1450 && throttle_input_pwm > 1150 && throttle_micros > PWM_OUT_2_DEFAULT){  // decelerate
          throttle_micros--;
        }else if(throttle_input_pwm > 800 && throttle_input_pwm < 1100){  // slow down quickly
          throttle_micros = throttle_micros - 5;
        }
      }


      if(!throttle_valid){  // verify throttle is in range
        throttle_input_pwm = 1500; //if not, set to neutral for safety
      }
      
      if(millis() - led_blink_last_ms >= 400){
        led_blink_last_ms = millis();
        led_blink_state = !led_blink_state;
        digitalWrite(nLED, led_blink_state ? 0 : 1);  // Low = ON, High = OFF
      }
      return throttle_micros;
      break;

    case 2:  // Wire-following autonomous mode with no Remote control input
      bool aux_in_manual = (pwm_read_rmt_dur(2) > 500 && pwm_read_rmt_dur(2) < 1450);
      if(aux_in_manual != case2_aux_entry_is_manual){
        case2_aux_changed = true;
      }
      if(case2_aux_changed && steering_valid){ // exit case 2 on any aux change
        drive_mode_status = aux_in_manual ? 0 : 1;
      }

      // Ramped throttle in case 2
      if(!btx_throttle_override_active){
        if(case2_use_cruise){
          if(case2_throttle_current < CRUISE_THROTTLE){
            case2_throttle_current++;
          }else if(case2_throttle_current > CRUISE_THROTTLE){
            case2_throttle_current--;
          }
        }else{
          if(case2_throttle_current < PWM_OUT_2_DEFAULT){
            case2_throttle_current++;
          }else if(case2_throttle_current > PWM_OUT_2_DEFAULT){
            case2_throttle_current--;
          }
        }
      }

      if(millis() - led_blink_last_ms >= 1000){
        led_blink_last_ms = millis();
        led_blink_state = !led_blink_state;
        digitalWrite(nLED, led_blink_state ? 0 : 1);  // Low = ON, High = OFF
      }
      return case2_throttle_current;
      break;
  }
  return 1500;
}


// Read and process the button state
void buttonLogic(){

  uint8_t gpio_state = !digitalRead(BUTTON_PIN);

  // regular press logic
	if(!bttn_state && gpio_state)  // button was not pressed & is currently pressed
		bttn_counter++;
	else if(bttn_state && !gpio_state)  // button was pressed & is not currently pressed
		bttn_counter--;
  else if(bttn_state && gpio_state)
    bttn_click_counter++;

	if(bttn_counter < 0){
    // Falling edge
    ESP_LOGI(TAG, "btn: release (held %ld cycles)", bttn_long_counter);
    if(bttn_3s_armed && !bttn_6s_state && bttn_long_counter < BUTTON_CAL_MAX_PRESS){
      button3sPressEvent();
    }
		bttn_state = 0;
		bttn_counter = 0;
    if(bttn_click_counter <= BUTTON_CLICK_MAX_LENGTH){
      buttonClickEvent();
    }
	}

	if(bttn_counter > BUTTON_DEBOUNCE_PERIODS){
    // Rising edge
    ESP_LOGI(TAG, "btn: press");
		bttn_state = 1;
		bttn_counter = BUTTON_DEBOUNCE_PERIODS;
    bttn_click_counter = 0;
	}

  // Timed press logic
  if(bttn_state){
    bttn_long_counter++;
  }else{
    bttn_long_counter = 0;
    bttn_3s_armed = 0;
    bttn_6s_state = 0;
  }

  if(!bttn_3s_armed && bttn_long_counter > BUTTON_CAL_MIN_PRESS){
    bttn_3s_armed = 1;
    ESP_LOGI(TAG, "btn: 3s window armed (release now to calibrate)");
  }

  if(!bttn_6s_state && bttn_long_counter > BUTTON_6S_PRESS){
    bttn_6s_state = 1;
    ESP_LOGI(TAG, "btn: 6s threshold reached");
    button6sPressEvent();
  }
}

// Called on button release if the press duration was less than the click threshold
void buttonClickEvent(){
  if(drive_mode_status == 2){
    case2_use_cruise = !case2_use_cruise; // Toggle case 2 output
    btx_user_override_until_ms = millis() + 2000;
    ESP_LOGI(TAG, "btn: click (mode2 cruise->%d)", case2_use_cruise);
  }else{
    drive_mode_status = 2; // Enter autonomous wire-following mode
    case2_use_cruise = true;
    case2_throttle_current = PWM_OUT_2_DEFAULT;
    uint32_t aux_pwm = pwm_read_rmt_dur(2);
    case2_aux_entry_is_manual = (aux_pwm > 500 && aux_pwm < 1500);
    case2_aux_changed = false;
    ESP_LOGI(TAG, "btn: click -> mode2 (auto)");
  }
}

// called when the long press threshold has been reached
void button3sPressEvent(){
  ESP_LOGI(TAG, "btn: 3s press -> calibrate center");

  // Calibrate cS1 center
  espPacketFifo_addCmdInt32(4 | 0x80000000, 0);  // Write S1 calibrate center
  espPacketFifo_addCmdInt32(5 | 0x80000000, 0);  // Write S2 calibrate center
  espPacketFifo_addCmdInt32(3 | 0x80000000, 0);  // Save new calibration

  longpress_flash_cycles = 0;
  config_saved_led_until_ms = millis() + 2000;  // Solid LED feedback for 2 seconds
  digitalWrite(nLED, 0);  // Low = ON
}

void button6sPressEvent(){
  pid_direction = (pid_direction == DIRECT) ? REVERSE : DIRECT;
  steeringPID.SetControllerDirection(pid_direction);
  g_deviceConfig.pid_direction = pid_direction;
  ESP_LOGI(TAG, "btn: 6s press -> pid_direction=%d", pid_direction);
  if (!msc_fat_save_all(&g_deviceConfig)) {
    Serial.println("Warning: failed to persist pid_direction to config file");
  }
}

void espPacketFifo_addCmdInt32(uint32_t cid, int32_t cmd){
  espPacketFifo[esp_packet_fifo_head].items.cid = cid;
  espPacketFifo[esp_packet_fifo_head].items.cmd.int32 = cmd;
  esp_packet_fifo_head ++;
  if(esp_packet_fifo_head == ESP_PACKET_FIFO_LENGTH){
    esp_packet_fifo_head = 0;
  }
}

void espPacketFifo_addCmdFloat(uint32_t cid, float cmd){
  espPacketFifo[esp_packet_fifo_head].items.cid = cid;
  espPacketFifo[esp_packet_fifo_head].items.cmd.flt32 = cmd;
  esp_packet_fifo_head ++;
  if(esp_packet_fifo_head == ESP_PACKET_FIFO_LENGTH){
    esp_packet_fifo_head = 0;
  }
}

void espPacketFifo_sendCmd(){
  if(esp_packet_fifo_head - esp_packet_fifo_tail){
    for(uint32_t i=0; i<(ESP_TX_PACKET_SIZE); i++){
      Serial1.write(espPacketFifo[esp_packet_fifo_tail].uart_data[i]);
    }
    esp_packet_fifo_tail ++;
    if(esp_packet_fifo_tail == ESP_PACKET_FIFO_LENGTH){
      esp_packet_fifo_tail = 0;
    }
  }
}

void chargeControl(){
  if(CHARGE_AUTO_ENABLE){
    switch (charge_status){
      case 0:  // Idle, waiting for input power
        if((stmPacket.items.vcharge > (stmPacket.items.vbatt + 0.4))){ //charge voltage must be 0.4v above battery voltage
          charge_status = 1;  // stops car
          lapFlag = true; // indicate vehicle has hit charger for telemetry lap counting
          charge_start_millis = millis();  // start 1 second timer
        }
        break;

      case 1:  // Input power present, waiting to request charge
        if((millis() - charge_start_millis) > 1000 && !WFRX_STATUS_GET_BIT(stmPacket.items.status_bits, WFRX_STATUS_CP_COOLDOWN)){
          espPacketFifo_addCmdInt32(9 | 0x80000000, 1);
          charge_start_millis = millis();
          charge_status = 2;
        }
        break;

      case 2:  // CP enable request sent, waiting to see charging has enabled
      if(WFRX_STATUS_GET_BIT(stmPacket.items.status_bits, WFRX_STATUS_CP_ENABLE)){
          charge_status = 3;
        }
        
        if((millis() - charge_start_millis) > 4000 || WFRX_STATUS_GET_BIT(stmPacket.items.status_bits, WFRX_STATUS_CP_COOLDOWN)){
          charge_start_millis = millis();
          charge_status = -1;  // abandon if charging does not start in 4 seconds, or a charging fault occurred
        }
        break;

      case 3: // Charging, waiting for charging to stop (for any reason)
        if(!WFRX_STATUS_GET_BIT(stmPacket.items.status_bits, WFRX_STATUS_CP_ENABLE)){
          charge_start_millis = millis();
          charge_status = -1;
        }
        break;

      case -1:  // charging has ended, resume throttle.  Do not allow re-attempt until after STM32 cooldown and 4-second ESP32 cooldown 
        if(!WFRX_STATUS_GET_BIT(stmPacket.items.status_bits, WFRX_STATUS_CP_COOLDOWN) && (millis() - charge_start_millis > 4000)){
          charge_status = 0;
        }
    }

  }else{
    if(WFRX_STATUS_GET_BIT(stmPacket.items.status_bits, WFRX_STATUS_CP_ENABLE)){
      espPacketFifo_addCmdInt32(9 | 0x80000000, 0);
    }
  }
}


int8_t gpio39Status(){
  if(digitalRead(SPARE_GPIO_IO39)){
    gpio_39_timer ++;  // increment restart timer when pin is high
    if(gpio_39_timer >= GPIO_39_RESTART_CYCLES){
      gpio_39_timer = GPIO_39_RESTART_CYCLES;
      gpio_39_status = 1;
    }
  }else{
    gpio_39_timer = 0;  // hold in reset when pin is low
    gpio_39_status = 0;
  }
  return gpio_39_status;
}

int8_t gpio40Status(){
  if(digitalRead(SPARE_GPIO_IO40)){
    gpio_40_timer ++;  // increment restart timer when pin is high
    if(gpio_40_timer >= GPIO_40_RESTART_CYCLES){
      gpio_40_timer = GPIO_40_RESTART_CYCLES;
      gpio_40_status = 1;
    }
  }else{
    gpio_40_timer = 0;  // hold in reset when pin is low
    gpio_40_status = 0;
  }
  return gpio_40_status;
}

void broadcastTelemetry(){
  if (   // Check if we should queue a telemetry message
      (millis() - telemetryBrodcastMillis >= 1000) // if at least 1 second since last broadcast
      | (stmPacket.items.charge_code != last_charge_code) //or charge code has changed
      | (stmPacket.items.status_bits != last_status_bits) //or status bits have changed
   ){
    String message = "- " + 
      String(vehicleID) + " " +
      String(vehicleStatus) + " " +
      String(millis()) + " " +
      String(stmPacket.items.vbatt, 2) + " " +
      String(stmPacket.items.vcharge, 2) + " " +
      String(stmPacket.items.icharge, 2) + " " +
      String(charge_status) + " " +
      String(stmPacket.items.status_bits) + " " +
      String(stmPacket.items.charge_code) + " " + 
      String(stmPacket.items.temperature) + " " +
      String(stmPacket.items.s1_radius) + " " +
      String(stmPacket.items.v5v, 2) + " " +
      String(steering_output) + " " +
      String(lapFlag)
      //+ " " + String(uart_overrun_count) // remove this after debugging stm <-> esp communication
      //+ " " + String(crc_error_count) // remove this after debugging stm <-> esp communication
      ;
    telemetryFifo_addMessage(message);  // Queue message instead of sending immediately
    telemetryBrodcastMillis = millis();
    lapFlag = false;  //reset lap flag after telemetry broadcast
  }
  last_charge_code = stmPacket.items.charge_code;
  last_status_bits = stmPacket.items.status_bits;
}

void telemetryFifo_addMessage(const String &message){ //Queue a telemetry message
  telemetryFifo[telemetry_fifo_head].message = message;
  telemetry_fifo_head++;
  if(telemetry_fifo_head == TELEMETRY_FIFO_LENGTH){
    telemetry_fifo_head = 0;
  }
}

void telemetryFifo_sendMessage(){ // Send one telemetry message from the queue (throttled to 100ms)
  // Only send if at least 100ms has passed since last send
  if(millis() - lastTelemetrySendTime >= 100){
    if(telemetry_fifo_head != telemetry_fifo_tail){
      // Queue has messages, send one
      broadcast(telemetryFifo[telemetry_fifo_tail].message);
      telemetry_fifo_tail++;
      if(telemetry_fifo_tail == TELEMETRY_FIFO_LENGTH){
        telemetry_fifo_tail = 0;
      }
      lastTelemetrySendTime = millis();
    }
  }
}

int BTxFollow(int base_throttle, double &btx_throttle, bool &btx_throttle_init, bool &btx_follow_active, float btx_signal){
  bool following_now = (btx_signal < BTxThreshold);

  if(!btx_throttle_init){
    btx_throttle = base_throttle;
    btx_throttle_init = true;
  }

  if(!following_now && !btx_follow_active){
    btx_throttle = base_throttle;
    return base_throttle;
  }

  if(following_now){
    btx_follow_active = true;
  }

  double target_throttle = base_throttle;
  if(following_now && base_throttle > PWM_OUT_2_DEFAULT){
    target_throttle = PWM_OUT_2_DEFAULT;
  }

  double step = fabs(BTxAccelRate);
  if(step <= 0.0){
    btx_throttle = target_throttle;
  }else if(btx_throttle < target_throttle){
    btx_throttle = fmin(btx_throttle + step, target_throttle);
  }else if(btx_throttle > target_throttle){
    btx_throttle = fmax(btx_throttle - step, target_throttle);
  }

  int output = (int)lround(btx_throttle);
  if(!following_now && btx_follow_active && output == base_throttle){
    btx_follow_active = false;
  }

  return output;
}

// -----------------------------------------------------------------------------
// applyDeviceConfig
//
// Copies all fields from cfg into the global firmware variables and reconfigures
// the PID controllers, stuck detector, and charge parameters to match.
// Call after msc_fat_init() and again whenever msc_fat_loop() returns true.
// -----------------------------------------------------------------------------
void applyDeviceConfig(const DeviceConfig& cfg)
{
  // Identification
  vehicleID           = cfg.vehicleID;
  rx_channel          = cfg.rx_channel;

  // Drive parameters
  CRUISE_THROTTLE     = cfg.CRUISE_THROTTLE;
  CREEP_THROTTLE      = cfg.CREEP_THROTTLE;
  throttle_expo       = cfg.throttle_expo;

  // PWM output limits and centres
  PWM_OUT_1_DEFAULT   = cfg.PWM_OUT_1_DEFAULT;
  PWM_OUT_1_MIN       = cfg.PWM_OUT_1_MIN;
  PWM_OUT_1_MAX       = cfg.PWM_OUT_1_MAX;
  PWM_OUT_2_DEFAULT   = cfg.PWM_OUT_2_DEFAULT;
  PWM_OUT_2_MIN       = cfg.PWM_OUT_2_MIN;
  PWM_OUT_2_MAX       = cfg.PWM_OUT_2_MAX;
  PWM_OUT_3_DEFAULT   = cfg.PWM_OUT_3_DEFAULT;
  PWM_OUT_3_MIN       = cfg.PWM_OUT_3_MIN;
  PWM_OUT_3_MAX       = cfg.PWM_OUT_3_MAX;
  PWM_OUT_4_DEFAULT   = cfg.PWM_OUT_4_DEFAULT;
  PWM_OUT_4_MIN       = cfg.PWM_OUT_4_MIN;
  PWM_OUT_4_MAX       = cfg.PWM_OUT_4_MAX;

  // Front PID gains and direction
  kp                  = cfg.kp;
  ki                  = cfg.ki;
  kd                  = cfg.kd;
  KP_DEFAULT          = cfg.kp;
  pid_direction       = cfg.pid_direction;
  steeringPID.SetControllerDirection(pid_direction);
  steeringPID.SetTunings(kp, ki, kd);
  steeringPID.SetOutputLimits(PWM_OUT_1_MIN - PWM_OUT_1_DEFAULT,
                               PWM_OUT_1_MAX - PWM_OUT_1_DEFAULT);

  // Rear PID gains and direction
  rear_kp             = cfg.rear_kp;
  rear_ki             = cfg.rear_ki;
  rear_kd             = cfg.rear_kd;
  rear_pid_direction  = cfg.rear_pid_direction;
  rearSteeringPID.SetControllerDirection(rear_pid_direction);
  rearSteeringPID.SetTunings(rear_kp, rear_ki, rear_kd);
  rearSteeringPID.SetOutputLimits(PWM_OUT_3_MIN - PWM_OUT_3_DEFAULT,
                                   PWM_OUT_3_MAX - PWM_OUT_3_DEFAULT);

  // Steering options
  reverse_rear_steer_in = cfg.reverse_rear_steer_in;
  use_heading           = cfg.use_heading;
  heading_weight        = -cfg.heading_weight;
  sensor_spacing_mm     = cfg.sensor_spacing_mm;

  // Stuck detector
  stuckDetector.SetThreshold(cfg.stuck_threshold_mm);
  stuckDetector.SetTimeout(cfg.stuck_timeout_ms);

  // BTx proximity
  BTxFollowChannel    = cfg.BTxFollowChannel;
  BTxThreshold        = cfg.BTxThreshold;
  BTxAccelRate        = cfg.BTxAccelRate;
  BTxCreepDuration    = cfg.BTxCreepDuration;

  // TPA
  tpa                 = cfg.tpa;

  // Sensor 2
  enable_sensor_2     = cfg.enable_sensor_2;

  // Sine swerve
  SineSwerveAmplitude = cfg.SineSwerveAmplitude;
  SineSwerveFrequency = cfg.SineSwerveFrequency;

  // Charging -- queue updated parameters to STM32
  CHARGE_VBATT_STOP   = cfg.CHARGE_VBATT_STOP;
  CHARGE_MAX_CURRENT  = cfg.CHARGE_MAX_CURRENT;
  CHARGE_MAX_SECONDS  = cfg.CHARGE_MAX_SECONDS;
  espPacketFifo_addCmdFloat(6 | 0x80000000, (float)CHARGE_VBATT_STOP);
  espPacketFifo_addCmdFloat(7 | 0x80000000, (float)CHARGE_MAX_CURRENT);
  espPacketFifo_addCmdInt32(8 | 0x80000000, CHARGE_MAX_SECONDS * 2);

  // RX channel
  espPacketFifo_addCmdInt32(1 | 0x80000000, (int32_t)(rx_channel - 1));
  espPacketFifo_addCmdInt32(3 | 0x80000000, 0);  // Save channel value to STM32 NVM
}
