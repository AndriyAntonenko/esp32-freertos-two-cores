# FreeRTOS on two ESP32 cores

Coursework project. Three independent firmware builds share one source tree and are selected
by PlatformIO environment. All state passed between tasks travels through FreeRTOS queues;
no shared global carries application state.

## Hardware

TTGO LoRa32 v2.1: ESP32 dual core, SX1276 LoRa transceiver, SSD1306 128x64 OLED.

| Function | GPIO |
|---|---|
| Button (active low, internal pull-up) | 4 |
| LED | 25 |
| LoRa CS / SCK / MOSI / MISO / RST / DIO0 | 18 / 5 / 27 / 19 / 23 / 26 |
| OLED SDA / SCL (I2C addr 0x3C) | 21 / 22 |

Pins are defined in `include/board_pins.h`.

## Layout

```
include/app_roles.h      entry point per build
include/board_pins.h     pin map
src/main.cpp             app_main, dispatches on ROLE_* macro
src/btn_led_main.cpp     environment btn_led
src/lora_led_main.cpp    environment led_lora
src/mutex_deadlock.cpp   environment mutex_deadlock
src/packet.{h,cpp}       32-byte LoRa wire format
lib/sx127x/              SX1276 driver (SPI, time-on-air calculation)
lib/ssd1306/             SSD1306 driver (I2C, 5x7 font)
```

Each source file is wrapped in `#ifdef ROLE_*`, so only one implementation compiles per build.

## Build and run

```sh
pio run -e <env> -t upload -t monitor
```

Environments: `btn_led`, `led_lora`, `mutex_deadlock`. Serial is 115200 baud.

## Environment: btn_led

Button controls LED blink interval. Single click moves forward through
250, 500, 1000, 2000 ms and wraps. Double click moves one step back.

Debounce is 20 ms. The double-click window is 300 ms.

| Task | Core | Priority | Role |
|---|---|---|---|
| `button_task` | 0 | 10 | debounce, classify single vs double |
| `led_task` | 1 | 5 | owns the interval, drives the LED |
| `ui_task` | 0 | 3 | draws the OLED |

Queues:

- `button_event_queue`, depth 10. ISR to `button_task`. Carries timestamp and pin level.
- `led_event_queue`, depth 10. `button_task` to `led_task`. Carries direction only, forward or back.
- `ui_state_queue`, depth 1, written with `xQueueOverwrite`. `led_task` to `ui_task`. Carries interval, last direction, press count.

The button task never knows the interval and the LED task never knows about clicks. The
interval lives in exactly one place.

### Verifying

Watch the LED and the OLED. Each single click doubles the interval, each double click halves
it, and the serial log prints the new value:

```
I (12345) FREE_RTOS_TASK: New blink interval is 500 ms
```

## Environment: led_lora

LED blinks at a fixed 100 ms while the radio sends 32-byte LoRa packets. A button press
queues a packet. The point is that a transmission lasting about one second does not disturb
the 100 ms blink.

Radio configuration: 433 MHz, SF11, BW 125 kHz, CR 4/5, explicit header, CRC on,
preamble 8, sync word 0x12. Low Data Rate Optimize is derived from SF and BW, not passed in,
so it can never disagree with the registers.

| Task | Core | Priority | Role |
|---|---|---|---|
| `button_task` | 0 | 10 | debounce, classify single vs double |
| `led_task` | 0 | 5 | 100 ms blink, measures its own lateness |
| `radio_task` | 1 | 5 | serializes and transmits |
| `ui_task` | 1 | 3 | draws the OLED |

Queues:

- `button_event_queue`, depth 10. ISR to `button_task`.
- `radio_msg_queue`, depth 10. `button_task` to `radio_task`. Carries `packet::Action`, the logical event, not wire bytes.
- `ui_event_queue`, depth 4. `led_task` and `radio_task` to `ui_task`. Tagged events, merged by the UI task.

The LED task runs on core 0 and the radio on core 1. `Sx127x::send` blocks on the DIO0
interrupt rather than polling, so the radio task is not runnable during transmission and
consumes no CPU.

### Packet format

`packet::Msg`, 32 bytes, packed, little endian. Size is enforced by `static_assert`.

| Field | Bytes |
|---|---|
| `version` | 1 |
| `magic` (0x67) | 1 |
| `no` | 4 |
| `timestamp_us` | 8 |
| `event` (single / double) | 1 |
| `reserved` | 17 |

`packet::parsePacket` is provided for a future receiver build and validates version and magic.

### Verifying

The LED task compares each wake-up against an absolute microsecond deadline and logs the
worst case every 50 toggles. Transmission must not move that number:

```
I (10373) LORA_LED: LED max late 1us
I (11103) LORA_LED: send #3 (single): ESP_OK, 1288 ms after press
```

Two independent checks on airtime:

- Single-press latency minus the 300 ms double-click window gives the airtime, about 987 ms.
- During a saturated burst, consecutive sends complete 990 ms apart. That spacing is the
  airtime directly, with no press timing involved.

Both agree with `Sx127x::time_on_air_ms`, which implements the datasheet formula in
section 4.1.1.7.

Report lines appearing exactly 5000 ms apart across transmissions show zero cumulative drift.

## Environment: mutex_deadlock

Deliberate AB-BA lock-order inversion. Two workers take two mutexes in opposite order. A
100 ms delay between the two takes makes the deadlock fire on the first attempt instead of
racing.

`mutex_a` guards `blink_interval_ms`. The LED task reads that variable under the mutex,
which is correct code, and becomes collateral damage when the workers deadlock.

| Task | Core | Priority | Role |
|---|---|---|---|
| `worker_a` | 0 | 5 | takes `mutex_a`, then wants `mutex_b` |
| `worker_b` | 1 | 5 | takes `mutex_b`, then wants `mutex_a` |
| `led` | 0 | 5 | innocent victim, blocks on `mutex_a` |
| `heartbeat` | 1 | 5 | touches no mutex, proves the scheduler is alive |

Both workers run a three-second healthy phase first, feeding the Task Watchdog, so the log
shows a clear transition from working to stuck.

### Watchdog behaviour

A pure deadlock triggers no watchdog. Blocked tasks leave the run queue, the idle tasks keep
running, and the idle tasks are the only default Task Watchdog subscribers. The system is
permanently dead while every watchdog stays satisfied.

The workers therefore subscribe explicitly with `esp_task_wdt_add(nullptr)` and feed with
`esp_task_wdt_reset()`. Once deadlocked they stop feeding and the timeout names them.

Current config: timeout 5 s, both idle tasks monitored, `CONFIG_ESP_TASK_WDT_PANIC` not set,
so the watchdog reports repeatedly without rebooting. To get a reboot instead, call
`esp_task_wdt_reconfigure()` with `trigger_panic = true`.

### Verifying

```
E task_wdt: Task watchdog got triggered. The following tasks/users did not reset the watchdog in time:
E task_wdt:  - worker_b (CPU 1)
E task_wdt:  - worker_a (CPU 0)
E task_wdt: Tasks currently running:
E task_wdt: CPU 0: IDLE0
E task_wdt: CPU 1: IDLE1
```

Both cores are idle at the moment the hang is reported. Nothing is starved and there is CPU
to spare, yet the application cannot progress. That is the difference between starvation,
which watchdogs detect, and lack of progress, which they do not.

The heartbeat task keeps printing after the watchdog fires, which rules out a crash or reset.
It also reports `led_task state 2`, meaning `eBlocked`.

## Environment notes

FreeRTOS tick is 100 Hz, so one tick is 10 ms. LED lateness is measured in microseconds
against `esp_timer_get_time()` rather than in ticks, because tick granularity cannot resolve
anything below 10 ms.

In ESP-IDF the `xTaskCreate` stack argument is in bytes, not words as in vanilla FreeRTOS.

Built against ESP-IDF 6.0.1.
