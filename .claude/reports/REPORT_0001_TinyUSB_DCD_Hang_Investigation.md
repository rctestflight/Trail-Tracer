# REPORT 0001 — TinyUSB DCD Hang Investigation

**STATUS:** COMPLETE  
**Instruction file:** INSTRUCTIONS_0001_TinyUSB_DCD_Hang_Investigation.md  
**Date:** 2026-04-05

---

## Summary

The hang at ~65 seconds is caused by a known TinyUSB DCD bug in arduino-esp32 2.0.17 (IDF 4.4.x): when a USB host resets or suspends the bus (~60–65s of inactivity on some hosts), the ESP32-S2 DCD interrupt handler encounters an unhandled state, logs "Unknown Condition", and leaves TinyUSB permanently broken. After this, the next `Serial.print()` call enters an infinite spin loop inside `USBCDC::write()` — `tud_cdc_n_write_available()` returns 0, the flush calls never drain the buffer (DCD is broken), and since there is no yield or timeout in that loop, the Arduino task locks permanently. Because the Arduino task never returns, PWM outputs, STM32 comms, and all other vehicle functions freeze along with it — the "whole ESP32 hangs" is a single-core starvation of the loop task. Two additional bugs were found unrelated to USB: `pwm_read_rmt.c` allocates the PWM duration buffer as `uint16_t`-sized but accesses it as `int32_t`, writing out-of-bounds for channels 2 and 3 on every RMT interrupt; and `Serial1.readBytes()` will block for 1000ms if the STM32 data pin fires spuriously. The quickest fix for the hang is to replace `Serial` debug output with `Serial0` (hardware UART0, already initialized), which bypasses TinyUSB entirely.

---

## Findings

### USB CDC Usage

`Serial` on the Lolin S2 Mini maps to `USBCDC` (TinyUSB CDC), not a hardware UART. This is enforced by two build flags set in the board JSON (`-DARDUINO_USB_CDC_ON_BOOT=1 -DARDUINO_USB_MODE=0`), which cause `USBCDC.h` to `extern USBCDC Serial`.

Debug output uses `Serial.print()` at ~100ms intervals (lines 753–795 in `main.cpp`). The active prints per cycle are:
- `"Steering Output: "` + float + `"  PID Output: "` + float + `"  target_x_pos : "` + float + `"\n"` ≈ **~70 characters**

The TinyUSB CDC TX FIFO is only **64 bytes** (`CONFIG_TINYUSB_CDC_TX_BUFSIZE 64`). The per-cycle print exceeds the FIFO in a single flush, meaning every 100ms cycle must wait for the host to drain the buffer.

**No `Serial.availableForWrite()` checks exist anywhere in the codebase.** The `USBCDC::write()` implementation contains an infinite `while(to_send)` loop that spins calling `tud_cdc_n_write_flush()` until space is available. If the TinyUSB DCD is broken, this loop never exits.

`Serial0` is also initialized (`Serial0.begin(115200)` in `setup()`) and maps to hardware UART0 (pins 43/44, visible on the tag-connect header). It is currently only used to start the baud rate — no debug output is routed through it.

`Serial1` (UART1 at 2 Mbaud, pins 17/18) is the STM32 comms interface. It is not involved in the hang.

### Version Analysis

- **arduino-esp32 version:** 2.0.17 (PlatformIO package `3.20017.241212+sha.dcc1105b`, dated 2024-12-12)
- **IDF version:** 4.4.x (embedded in the package)
- **TinyUSB DCD:** pre-compiled into `tools/sdk/esp32s2/lib/libarduino_tinyusb.a`
- **No `platform_packages` override or version pin** in `platformio.ini`

The "Unknown Condition" error originates in `dcd_esp32sx.c` (TinyUSB's ESP32-S2 DCD driver). In IDF 4.4.x, the DCD interrupt handler has a catch-all `else` path that fires when a USB interrupt arrives in an unexpected hardware state (e.g., during a SETUP packet after a bus reset, or during suspend). It logs `E (xxx) TUSB:DCD: Unknown Condition` and leaves the DCD in an unrecoverable state. This was fixed in IDF 5.x / arduino-esp32 3.x.

### Timing Analysis

There is **no user-code timer, task, or counter that fires at 65 seconds**. The `MAIN_LOOP_PERIOD = 360000` is a 1-hour rollover at 100Hz (3600s). The `StuckVehicle` timeout is 15s (repeated). `CHARGE_MAX_SECONDS = 60` is a parameter but only affects charging behavior (not a timer that fires unconditionally).

The 65-second window is consistent with **USB host suspend behavior**: many USB host controllers issue a USB suspend (SOF gap > 3ms) or a bus reset to idle devices after approximately 60–65 seconds with no data transfer activity. If no CDC terminal is open on the host side, the host may issue such a reset at this point. TinyUSB's ESP32-S2 DCD in IDF 4.4.x does not handle this gracefully.

### Task/Interrupt Analysis

The codebase creates **no user FreeRTOS tasks**. There are two relevant tasks in the system:
1. **Arduino `loop()` task** — priority 1 (low)
2. **TinyUSB `usbd` task** — priority `configMAX_PRIORITIES - 1` (highest; created in `esp32-hal-tinyusb.c:718`)

The TinyUSB task has maximum priority so it preempts the Arduino task at every OS tick. This means TinyUSB IS running during the `write()` spin — the hang is **not** a priority starvation issue. The hang occurs because the DCD is permanently broken and `tud_task()` cannot drain the USB TX hardware, so `tud_cdc_n_write_available()` always returns 0 regardless of how many times the TinyUSB task runs.

The `USBCDC::write()` inner loop (`while(to_send)`) has **no timeout and no `taskYIELD()`**. When `space == 0`, it calls `tud_cdc_n_write_flush()` and immediately `continue`s. Once the DCD is broken:
- `tud_cdc_n_connected()` still returns true (DTR/RTS not yet cleared)
- `space` stays 0 on every iteration
- The loop runs at full speed until the OS tick preempts it to run `usbd`
- `usbd` runs `tud_task()`, finds nothing can be sent, returns
- Arduino task resumes and immediately re-enters the spin

**Because the Arduino loop task never returns, every system function inside `loop()` is frozen**: PWM servo outputs stop updating, STM32 UART commands stop being sent, button logic stops, charge control stops. This is why the "whole ESP32 hangs" — it is a single thread doing all the work, and that thread is permanently stuck in a USB write.

The `STM_nDATA_READY` interrupt (`receiveStmUart()`) still fires correctly and sets `data_ready = 1`, but `loop()` never reads it.

**No watchdog catches this.** The ESP32 Task Watchdog Timer (TWDT) in arduino-esp32 2.x monitors only the idle task by default. The idle task never runs either (Arduino task holds CPU between OS ticks and TinyUSB task holds it at the tick), so the TWDT actually fires on the idle task too — but in arduino-esp32 2.x, TWDT panic on the idle task only resets if `CONFIG_ESP_TASK_WDT_PANIC=1`, which is not set by default.

The `STM_nDATA_READY` GPIO ISR and the RMT ISR (`rmt_isr_handler`) both continue to fire since they run at hardware interrupt level above FreeRTOS. But their results are never consumed.

### Additional Main Loop Hang Risks

Three more issues were found during main loop analysis. None of these are the *cause* of the current 65-second hang, but they are all potential independent hang or corruption sources.

#### 1. `pwm_read_rmt.c` — Heap Buffer Overflow from ISR (Severity: High)

In `pwm_read_rmt_init()`:
```c
int32_t *pwm_read_dur = NULL;           // declared as int32_t* — 4 bytes per element
// ...
pwm_read_dur = (uint16_t *) malloc(numberOfPins * sizeof(uint16_t));  // allocated as uint16_t-sized!
```

With `PWM_IN_NUM = 4`:
- Allocated: `4 × sizeof(uint16_t) = 8 bytes`
- Needed: `4 × sizeof(int32_t) = 16 bytes`

The ISR (`rmt_isr_handler`) writes:
```c
pwm_read_dur[i] = item->duration0;   // int32_t write, 4 bytes
```

For channels `i = 2` and `i = 3`, this writes 4 bytes each at offsets 8–11 and 12–15 — **beyond the 8-byte allocation**. This corrupts whatever is adjacent in the heap. The ISR fires at ~50–400 Hz per channel (every PWM input pulse), so the corruption is continuous.

This can destabilize any heap-allocated structure: FreeRTOS queues, TinyUSB internal buffers, the WiFi stack, or malloc metadata. It is a plausible secondary contributor to the TinyUSB "Unknown Condition" if it corrupts a TinyUSB internal state variable. The fix is one character: change `sizeof(uint16_t)` to `sizeof(int32_t)`.

#### 2. `Serial1.readBytes()` — 1-Second Blocking Timeout

```cpp
// main.cpp:508
Serial1.readBytes(stmPacket.uart_data, STM_TX_PACKET_SIZE);  // blocks up to 1000ms
```

`HardwareSerial::readBytes()` uses the Stream default timeout of **1000ms**. This is called every time `data_ready` is set. If the STM32's DATA_READY pin fires for any reason but the full 116-byte packet doesn't arrive (noise, framing error, STM32 reset, or UART overrun), this call blocks for 1 second.

During that 1 second: all PWM outputs freeze, servo positions hold at last value, no new STM32 commands are sent. This is a recoverable stall (not a permanent hang), but at 100Hz loop rate a single occurrence drops ~100 control cycles.

**Fix:** Call `Serial1.setTimeout(10)` in `setup()` before the first `readBytes()`. At 2 Mbaud, 116 bytes takes 0.58ms; a 10ms timeout gives 17× margin while limiting the stall to one loop period.

#### 3. `STM_TX_PACKET_SIZE` / Struct Size Mismatch

`STM_TX_PACKET_SIZE = 116` but the `__StmTxPacketStruct` is **120 bytes** (all fields are 4-byte aligned; no padding; sum = 120). The `readBytes()` call reads only 116 bytes, so `checksum` (the last `uint32_t`) is never populated from UART — it always reads as whatever was in the buffer at startup. The union `uart_data[116]` is also 4 bytes shorter than the struct, meaning accessing `items.checksum` technically reads outside `uart_data`. The `checksum` field is not used in any logic so this causes no current functional issue, but it should be corrected in coordination with the STM32 firmware.

### PSRAM Configuration

The Lolin S2 Mini board JSON sets `-DBOARD_HAS_PSRAM` in `extra_flags`. The sdkconfig for the `dio_qspi` flash variant (used by the S2 Mini, `flash_mode: dio`) has:

```
CONFIG_ESP32S2_SPIRAM_SUPPORT=1
CONFIG_SPIRAM=1
CONFIG_SPIRAM_USE_MALLOC=1
CONFIG_SPIRAM_SIZE=-1  (auto-detect)
```

At boot, the firmware attempts PSRAM initialization, fails because no PSRAM exists, and logs the expected error. However, `CONFIG_SPIRAM_USE_MALLOC=1` causes the heap allocator to attempt to use PSRAM-backed memory in some allocations. With no PSRAM, this adds overhead and may contribute to rare heap allocation failures, but it is **not the direct cause of the TinyUSB hang**. It should still be disabled.

To disable PSRAM, add to `platformio.ini`:
```ini
board_build.extra_flags = -UBOARD_HAS_PSRAM
```
This undefines `BOARD_HAS_PSRAM` at compile time, preventing PSRAM-aware code paths from activating.

---

## Recommendations

1. **[Immediate fix] Switch debug output to `Serial0`**  
   Replace all `Serial.print()`/`Serial.println()`/`Serial.printf()` calls in the debug block (lines 753–795 of `main.cpp`) with `Serial0.print()` etc. `Serial0` is hardware UART0 (already initialized at 115200, visible on the tag-connect header), is completely independent of TinyUSB, and cannot hang. This eliminates the hang without any other changes.

2. **[Belt-and-suspenders] Guard Serial writes with `availableForWrite()`**  
   If `Serial` (USB CDC) must be used for any output, wrap writes with:
   ```cpp
   if (Serial && Serial.availableForWrite() > 64) {
     Serial.print(...);
   }
   ```
   The `Serial &&` check tests `tud_cdc_n_connected()`, preventing writes when no host is connected. The `availableForWrite() > 64` check avoids entering the blocking spin loop.

3. **[High priority] Fix `pwm_read_rmt.c` heap overflow**  
   In `pwm_read_rmt_init()`, change:
   ```c
   // Before (WRONG — allocates 8 bytes for 4 int32_t elements):
   pwm_read_dur = (uint16_t *) malloc(numberOfPins * sizeof(uint16_t));
   // After:
   pwm_read_dur = (int32_t *) malloc(numberOfPins * sizeof(int32_t));
   ```
   This stops the ISR from corrupting the heap on every RMT trigger for channels 2 and 3. This bug has been present since the code was written and is a source of latent instability across all configurations.

5. **[Low risk, easy] Add `Serial1.setTimeout(10)` in setup()**  
   Add before the first use of Serial1:
   ```cpp
   Serial1.setTimeout(10);  // 10ms max wait; 116 bytes at 2Mbaud takes 0.58ms
   ```
   Limits the `readBytes()` stall from 1000ms to 10ms on any UART fault.

6. **[Cleanup] Disable PSRAM in platformio.ini**  
   ```ini
   board_build.extra_flags = -UBOARD_HAS_PSRAM
   ```
   Prevents the PSRAM init failure at boot and avoids heap allocator involvement with non-existent PSRAM.

7. **[Framework upgrade] Update to arduino-esp32 3.x (espressif32 6.x)**  
   In `platformio.ini`, pin the platform version:
   ```ini
   platform = espressif32@6.9.0
   ```
   arduino-esp32 3.x uses IDF 5.3.x, which includes a rewritten TinyUSB DCD for ESP32-S2 that handles USB bus resets and suspend correctly. The "Unknown Condition" class of errors is fixed. (Note: this is a major version upgrade; test carefully.)

---

## Payload

### Active Serial Usage in main.cpp

| Line | Call | Context |
|------|------|---------|
| 421 | `Serial.begin(115200)` | setup() — starts USB CDC |
| 422 | `Serial0.begin(115200)` | setup() — starts HW UART0 (no output routed here) |
| 423 | `Serial1.begin(2000000)` | setup() — STM32 comms UART |
| 409–411 | `Serial.println("ESP-NOW Init ...")` | ESPNOW_ENABLED only (disabled) |
| 753 | `Serial.print("Steering Output: ")` | 100ms debug loop — **ACTIVE** |
| 755 | `Serial.print("  PID Output: ")` | 100ms debug loop — **ACTIVE** |
| 772 | `Serial.print("  target_x_pos : ")` | 100ms debug loop — **ACTIVE** |
| 795 | `Serial.println()` | 100ms debug loop — **ACTIVE** |

All other Serial calls are commented out. The four active calls produce ~70 characters per 100ms — exceeding the 64-byte TinyUSB CDC TX FIFO in a single cycle.

### platformio.ini (full)

```ini
[env:lolin_s2_mini]
platform = espressif32
board = lolin_s2_mini
framework = arduino
monitor_speed = 115200
lib_deps =
    arduino-libraries/Servo@^1.2.1
    br3ttb/PID@^1.2.1

build_flags =
    -D CORE_DEBUG_LEVEL=5
```

No `platform_packages`, no `platform` version pin, no `board_build.extra_flags`.

### Board JSON extra_flags (from lolin_s2_mini.json)

```
-DARDUINO_LOLIN_S2_MINI
-DBOARD_HAS_PSRAM          ← enables PSRAM init attempt
-DARDUINO_USB_CDC_ON_BOOT=1 ← Serial → USBCDC
-DARDUINO_USB_MODE=0        ← TinyUSB mode (not hardware CDC)
```

### Relevant sdkconfig values (esp32s2/dio_qspi)

```
CONFIG_TINYUSB_CDC_TX_BUFSIZE 64    ← 64-byte TX FIFO
CONFIG_TINYUSB_CDC_RX_BUFSIZE 64
CONFIG_ESP32S2_SPIRAM_SUPPORT 1     ← PSRAM enabled
CONFIG_SPIRAM 1
CONFIG_SPIRAM_USE_MALLOC 1
CONFIG_SPIRAM_SIZE -1               ← auto-detect (fails on S2 Mini)
```

### USBCDC::write() hang path (USBCDC.cpp:375–420)

```cpp
size_t USBCDC::write(const uint8_t *buffer, size_t size) {
    if(...|| !tud_cdc_n_connected(itf)){ return 0; }  // safe if not connected
    // acquire tx_lock with 250ms timeout
    while(to_send){
        if(!tud_cdc_n_connected(itf)){ break; }       // bail if disconnected mid-write
        size_t space = tud_cdc_n_write_available(itf);
        if(!space){
            tud_cdc_n_write_flush(itf);
            continue;   // ← INFINITE LOOP if DCD is broken and space stays 0
        }
        ...
    }
}
```

**Hang condition:** `tud_cdc_n_connected()` returns true (state not yet updated after DCD failure), `space` = 0 forever, `flush()` never drains the buffer → spinning `continue` with no timeout or yield → system locked.

### FreeRTOS tasks (no user tasks)

Only framework tasks run: Arduino loopTask, TinyUSB tusb_device_task. No user `xTaskCreate()` calls in any source file.

### Timing cross-check

| Event | Period |
|-------|--------|
| STM32 data interrupt | ~100Hz (10ms) |
| Debug Serial print | every 100ms |
| StuckVehicle timeout | 15,000ms |
| CHARGE_MAX_SECONDS | 60s (only triggers if charging active) |
| MAIN_LOOP_PERIOD rollover | 3,600s |
| USB host suspend timeout | ~60–65s (host-dependent, typical Windows/Linux behavior) |

No user-code periodic event aligns with 65 seconds. The USB host suspend is the only plausible trigger.
