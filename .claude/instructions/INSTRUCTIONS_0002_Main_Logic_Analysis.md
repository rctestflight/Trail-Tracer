# INSTRUCTIONS_0002: Main.cpp Logic and Peripheral Interaction Analysis

## Context

Read `.claude/reports/REPORT_0001_TinyUSB_DCD_Hang_Investigation.md` for prior findings.

The ESP32-S2 hangs ~65 seconds after boot with `TUSB:DCD: Unknown Condition`. The prior investigation focused on the USB CDC path and concluded TinyUSB was the root cause. **We no longer believe that.** The developer has run similar USB CDC debug output configurations on other projects without issue. The TinyUSB error is likely a **symptom** of something else going wrong — memory corruption, a runaway peripheral, or a logic error in the main application that destabilizes the system.

**Additional info:** There are **no PWM inputs physically connected** to any GPIO pins. The RMT peripheral is configured to read PWM signals, but the inputs are floating/unconnected.

## Objective

Perform a high-level analysis of `src/main.cpp` as a monolith. We want to understand the overall application logic, state machine, control flow, and identify anything that could cause system instability ~65 seconds after boot.

## Tasks

### 1. Document the Application Architecture

Read `src/main.cpp` end-to-end and produce a high-level summary:
- What is this application doing? (purpose, system context)
- What is the main state machine / operating mode structure?
- What are the major functional blocks in `setup()` and `loop()`?
- What global variables hold important state?
- What timers, counters, or periodic actions exist and at what intervals?

### 2. Analyze the RMT / PWM Read Path

Read `src/pwm_read_rmt.c` and trace how it's used in `main.cpp`.
- How is the RMT peripheral configured?
- What happens when the RMT peripheral is configured to read PWM but **no signal is present** on the input pins (pins floating)?
- Does the RMT driver have timeouts? Does it block? Does it fire interrupts continuously with garbage data from floating pins?
- Could RMT behavior with no input cause memory writes to unexpected locations, excessive interrupt load, or watchdog issues?
- Is there any error handling for "no signal" conditions?

### 3. Analyze the STM32 Comms Path

Read `src/wfrx_stm32_comms.cpp` and trace how it interacts with `main.cpp`.
- What protocol is used on UART1 (2 Mbaud)?
- What happens when STM32 data arrives (the `data_ready` interrupt)?
- Is there any validation of received data? Bounds checking?
- Could malformed or missing STM32 data cause out-of-range values that propagate through the control logic?

### 4. Analyze the ESP32 Application Logic

Read `src/rctf_wfrx_esp32.cpp` and understand what functions it provides to `main.cpp`.
- What does this module do?
- Are there any functions that could produce unexpected results with bad input data?

### 5. Trace What Happens at ~65 Seconds

With the full application logic understood, look for anything that could accumulate or trigger around 65 seconds:
- Any counters that overflow or wrap?
- Any buffers that fill up?
- Any WiFi events that fire on a timer (reconnection attempts, beacon intervals)?
- Any RMT buffer that fills with noise from floating pins and eventually overflows?
- Any state machine transition that happens after a timeout?
- Could the `StuckVehicle` 15-second timeout (which repeats) cause issues on its 4th or 5th firing (~60-75s)?

### 6. Identify Code Smells and Risk Areas

Flag anything in main.cpp that looks risky:
- Unguarded array accesses
- Global state modified from both ISR and main loop without volatiles or atomics
- Large local variables that could blow the stack
- Blocking operations in the main loop
- Missing null checks
- Any use of `String` class (heap fragmentation risk on ESP32)
- Division by zero risks
- Cast issues

## Report Format

Write your report to `.claude/reports/REPORT_0002_Main_Logic_Analysis.md`

```
# REPORT_0002: Main.cpp Logic and Peripheral Interaction Analysis

## Status: COMPLETE | PARTIAL | FAILED

## Summary
(2-3 sentence overview)

## Findings

### Application Architecture
(High-level description of what the system does and how it's structured)

### RMT / PWM Read Analysis
(What happens with no PWM input connected — is this a problem?)

### STM32 Comms Analysis
(Protocol, data flow, error handling)

### ESP32 Application Logic
(rctf_wfrx_esp32.cpp role and risks)

### 65-Second Timing Analysis
(What could trigger at that time?)

### Code Smells and Risk Areas
(Specific issues found, with line numbers)

## Recommendations
(Ordered by likelihood of being the root cause)

## Payload
(Key code snippets, state machine diagrams, variable listings)
```
