# INSTRUCTIONS_0001: TinyUSB DCD Hang Investigation

## Problem

The ESP32-S2-MINI (Lolin S2 Mini) hangs approximately 65 seconds after boot with this error:

```
E (65902) TUSB:DCD: Unknown Condition
```

After this error, the ESP becomes completely unresponsive. It does not recover — it requires a power cycle.

### Context

- **Board:** Lolin S2 Mini (ESP32-S2)
- **Framework:** Arduino via PlatformIO
- **Platform:** espressif32
- **Build flags:** `-D CORE_DEBUG_LEVEL=5` (verbose logging)
- **The ESP32-S2 uses TinyUSB for its native USB interface** — there is no separate USB-to-UART chip. `Serial` on this board defaults to USB CDC via TinyUSB.

### Boot log (normal until hang)

```
UART0 baud(115200) Mode(800001c) rxPin(44) txPin(43) — OK
UART1 baud(2000000) Mode(800001c) rxPin(18) txPin(17) — OK
WiFi STA started — OK
...
E (65902) TUSB:DCD: Unknown Condition — HANGS HERE
```

PSRAM init failure is also present but is expected (S2 Mini has no PSRAM). This is a separate issue.

## Tasks

### 1. Identify USB CDC usage in the codebase

Examine all source files in `src/` and `lib/` for:
- All uses of `Serial` (which maps to USB CDC on this board)
- Any `Serial.print`, `Serial.write`, `Serial.begin` calls
- Any large or frequent serial output that could overflow the USB CDC buffer
- Whether there are any `Serial.availableForWrite()` checks before writes
- Whether `Serial0` (hardware UART0) is used anywhere vs `Serial` (USB CDC)
- Any USB-related configuration or callbacks

### 2. Check for known TinyUSB / Arduino-ESP32 issues

Look at `platformio.ini` to determine the exact `platform` and framework versions being used. Check if there's a `platform_packages` override or version pin.

### 3. Analyze the timing

The hang occurs at ~65 seconds. Investigate:
- Is there a periodic task, timer, or loop iteration that would trigger around that time?
- Could this be a WiFi event or reconnection attempt interacting with USB?
- Is there any watchdog configuration?
- Are there any blocking calls or long-running operations in `loop()`?

### 4. Check for task/interrupt conflicts

The ESP32-S2 is single-core. Look for:
- FreeRTOS task creation and priorities
- Whether Serial writes happen from ISR context or from tasks that could conflict with TinyUSB's task
- Any `delay(0)` or `yield()` calls that would let TinyUSB process
- Whether the main loop has adequate yield time for the USB stack

### 5. Review PSRAM configuration

Check `platformio.ini` and any sdkconfig overrides for PSRAM settings. The S2 Mini has no PSRAM — if PSRAM is enabled in the board config, document what setting needs to change to disable it.

## Report Format

Write your report to `.claude/reports/REPORT_0001_TinyUSB_DCD_Hang_Investigation.md`

Structure your report as:

```
# REPORT_0001: TinyUSB DCD Hang Investigation

## Status: COMPLETE | PARTIAL | FAILED

## Summary
(2-3 sentence overview of findings)

## Findings

### USB CDC Usage
(What you found about Serial/USB usage patterns)

### Version Analysis
(Arduino-ESP32 core version, TinyUSB version, known issues)

### Timing Analysis
(What happens around the 65-second mark)

### Task/Interrupt Analysis
(Potential conflicts with TinyUSB)

### PSRAM Configuration
(Current config and what to change)

## Recommendations
(Ordered list of suggested fixes, most likely to resolve the issue first)

## Payload
(Raw details: relevant code snippets, full Serial usage listing, config dumps)
```
