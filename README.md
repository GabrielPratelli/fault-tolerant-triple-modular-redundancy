# TRM_System — Fault-Tolerant Embedded Supervision System

A 2-of-3 voting system built on Triple Modular Redundancy and N-version diversity, running across two different microcontrollers.

| Channel | Board | Framework | Algorithm | Source |
|:---:|---|---|---|---|
| A | ESP32 (core 0) | ESP-IDF | Hardware multiplication | `esp32-supervisor/src/main.c` |
| B | ESP32 (core 1) | ESP-IDF | Sum of odd terms, no multiplication | `esp32-supervisor/src/main.c` |
| C | Arduino Uno | Arduino | Lookup table | `arduino-supervisor/src/main.cpp` |

The ESP32 runs channels A and B concurrently on its two cores, each evaluating the same function with a different algorithm. The Arduino evaluates the same function a third way, with a lookup table, and acts as supervisor: it compares the three results, sorts what it finds into a fault bucket, and sends back a verdict. When an error gets serious enough the supervisor calls a system lock, and both boards hold it until the fault clears. The ESP32 keeps a redundant copy of its own state, so a value that gets corrupted is caught and replaced.

A fault that breaks the ESP32 multiplier may pass through the Arduino untouched, since the instruction set and the algorithm are both different. That lowers the chance of a single fault taking down the whole system without removing it.

## Contents

- [Architecture](#architecture)
- [Repository layout](#repository-layout)
- [Quick build and upload](#quick-build-and-upload)
- [Development and wiring](#development-and-wiring)
- [License](#license)

## Architecture

The ESP32 is the main computational unit. Its two cores run channels A and B under FreeRTOS, and it holds its critical state in duplicate so a corrupted copy can be detected and corrected.

The Arduino Uno is the supervisor. It evaluates the same function a third time with a lookup table, then arbitrates. Three results can agree unanimously, or the vote splits: majority on A, majority on B, a common-mode fault where two channels agree on a wrong answer, or all three different, and each case gets its own response.

Communication is a framed protocol on the Arduino's hardware UART at 115200 baud, with a CRC16 (CCITT) on every frame. Both firmwares have fault injection hooks, so single-channel, double-channel and common-mode faults can be reproduced on demand and watched as they happen.

## Repository layout

```
TRM_System/
├── esp32-supervisor/              ← channels A + B  (framework: espidf)
│   ├── platformio.ini
│   ├── CMakeLists.txt
│   ├── sdkconfig.esp32dev
│   └── src/main.c                 ← ESP32 firmware (FreeRTOS, UART1, TMR)
│
├── arduino-supervisor/            ← channel C / supervisor  (framework: arduino)
│   ├── platformio.ini
│   └── src/main.cpp               ← Arduino firmware (voting, AVR watchdog)
│
├── README.md
└── LICENSE
```

Build artifacts (`.pio/`) and tool caches are not committed. Everything is regenerated with `pio run`.

## Quick build and upload

```bash
# Channels A + B  (ESP32, ESP-IDF)
cd esp32-supervisor && pio run && pio run -t upload

# Channel C  (Arduino Uno, Arduino framework)
cd arduino-supervisor && pio run && pio run -t upload
```

Serial monitor, on the ESP32 only. It is the single console the system reports to:

```bash
pio device monitor      # 115200 baud
```

> **Golden rule: two different boards = two separate PlatformIO projects.**
> The two firmware files cannot live in the same project. They target different
> boards, frameworks and toolchains. Each folder is independent (config, build
> `.pio/`, serial port), so they never get in each other's way.

## Development and wiring

### 1. Folder layout

Each board is a standalone PlatformIO project:

```
TRM_System/
├── arduino-supervisor/        ← open this one in a VSCodium window
│   ├── platformio.ini         (board: uno / nanoatmega328[new], framework: arduino)
│   └── src/
│       └── main.cpp           (your channel C code)
└── esp32-supervisor/          ← open this in ANOTHER VSCodium window
    ├── platformio.ini         (board: esp32dev, framework: espidf)
    └── src/
        └── main.c             (your channel A + B code)
```

### 2. Wiring between the two boards

| Signal          | ESP32       | Arduino           |
|-----------------|-------------|-------------------|
| UART TX         | GPIO17 (TX) | pin **0** (RX)    |
| UART RX         | GPIO16 (RX) | pin **1** (TX)    |
| GND (common!)   | GND         | GND               |
| Actuator        | GPIO18      | pin 7             |
| Safe-state LED  | —           | LED_BUILTIN (13)  |

- Link baud rate: **115200** (hardware UART on pins 0/1, full-duplex).
- **A common ground is mandatory** between the two boards, otherwise the UART link is just noise.
- If your ESP32 module is a **WROVER** (with PSRAM), pins 16/17 are taken internally by the PSRAM. Change the pins in `esp32-supervisor/src/main.c` (e.g. `LINK_TX_PIN 27`, `LINK_RX_PIN 26`) and update pins 0/1 on the Arduino side.
- The PC talks to the ESP32 only, over USB (UART0). The Arduino no longer has a serial link to the PC: its hardware UART (pins 0/1) is entirely dedicated to the ESP32 link. The Arduino's USB port is used only for flashing, so disconnect the ESP32 while uploading.

### 3. Prerequisites

- VSCodium with the **PlatformIO IDE** extension (installed from Open VSX).
- PlatformIO Core with the `atmelavr` and `espressif32` platforms.

### 4. First run, step by step

1. **Open the Arduino project**: `File → Open Folder` → `~/Cose/TRM_System/arduino-supervisor`.
2. **Open the ESP32 project**: open a second VSCodium window (`Ctrl+Shift+N`) and use `File → Open Folder` → `~/Cose/TRM_System/esp32-supervisor`. Two separate windows means zero interference: each one only acts on its own folder and its own board.
3. On opening, PlatformIO initialises the project. A toolbar appears at the bottom: ✓ build, ➔ upload, 🔌 monitor.
4. The first build of the ESP32 project downloads the ESP-IDF framework (a few hundred MB, one-off) and takes a few minutes to compile. The Arduino project is ready straight away, since the AVR toolchain is already present.

#### Build

- The **✓ (Build)** icon at the bottom, or `pio run` from a terminal in that folder.
- Only the files in the current folder get compiled.

#### Upload to the right board

- Connect the board, then the **➔ (Upload)** icon.
- Each board has its own USB port:
  - Arduino Uno → usually `/dev/ttyACM0`, or `/dev/ttyUSB0` with a CH340 chip.
  - ESP32 → usually `/dev/ttyUSB0` (CP2102/CH340).
- If PlatformIO picks the wrong port while both boards are connected, force it in `platformio.ini`:

  ```ini
  [env:uno]
  upload_port = /dev/ttyACM0
  monitor_port = /dev/ttyACM0
  ```

  ```ini
  [env:esp32dev]
  upload_port = /dev/ttyUSB0
  monitor_port = /dev/ttyUSB0
  ```

  To list the active ports: `pio device list`.

#### Serial monitor

- The **🔌 (Serial Monitor)** icon. You do not need to open the monitor for both boards at once in the same window, but always close the monitor before uploading on the same port, since the monitor keeps the port busy.
- Baud is 115200 on both projects, already set in `platformio.ini`. Open the monitor on the ESP32 only: the Arduino no longer prints anything to the PC. Its events show up in the ESP32 logs, which receives the ACK/SAFE/LOCK replies.

### 5. Typical workflow

1. Edit the code in one of the two windows.
2. Build (✓) → fix any errors → Upload (➔).
3. Restart and verify with the serial monitor.
4. Repeat in the other window for the other board.
5. With both boards powered and connected to each other (common GND), read the logs on the ESP32. It prints `action applied` / `SAFE STATE` plus the ACK/SAFE messages received from the Arduino.

### 6. Troubleshooting ESP32 uploads

- **`Unable to verify flash chip connection (No serial data received.)`** after `Changing baud rate to 460800`: the esptool stub loses communication at high speed. Already fixed in `platformio.ini` with `upload_speed = 115200`. If it happens again, try a different USB cable (data, not charge only), plug the board straight into the PC instead of through a hub, and disconnect the wires to the Arduino during the upload, especially if the boards share a power supply.
- **`Flash memory size mismatch. Expected 4MB, found 2MB!`**: this board has a 2MB flash (2MB WROOM). Already set with `board_upload.flash_size = 2MB`.

### 7. Code notes (to avoid common mistakes)

- The ESP32 file is ESP-IDF, not Arduino. It uses `app_main()`, FreeRTOS and `driver/uart.h`, which is why the project uses `framework = espidf`. Building it with `framework = arduino` gives you link errors on `app_main`.
- The Arduino to ESP32 link uses the Uno's hardware UART (pin 0 = RX, 1 = TX) at 115200. The SoftwareSerial on pins 8/9 was removed. Pins 0/1 are shared with the Uno's USB-serial chip, so disconnect the ESP32 (or the USB) while flashing the Arduino, otherwise the upload may get corrupted.
- Fault injection: enable at most one among `INJECT_A`, `INJECT_B`, `INJECT_BOTH` (ESP32) and `INJECT_C` (Arduino), as documented in the comments.

## License

The source code of both firmwares is released under the [MIT](LICENSE) license.
