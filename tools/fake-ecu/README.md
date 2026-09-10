# fake-ecu

Standalone test firmware: simulates a vehicle ECU on the CAN bus so that
`src/obd_can.cpp` (native CAN, `-DUSE_CAN`) can be tested **without a real car**.

## Hardware

- 2nd board: LilyGO **T7-S3** (ESP32-S3-WROOM-1-N16R8) + CAN transceiver
- CAN pins on the test board: **TX = GPIO13**, **RX = GPIO14** (in `src/main.cpp`)
- Wiring to the obd_can_2mqtt board:
  - CAN-H ↔ CAN-H, CAN-L ↔ CAN-L, **GND ↔ GND**
  - **120 Ω** between CAN-H/CAN-L at **both** ends → a multimeter across the bus reads ~60 Ω

> **Transceiver:** the cheap **CJMCU-230** boards do not work reliably (weak
> driver, "receive only"). Tested and good: **Waveshare "SN65HVD230 CAN Board"**
> (onboard termination jumper, Rs wired correctly).
> Board `CAN_TX` → ESP TX GPIO, `CAN_RX` → ESP RX GPIO. Keep the termination
> jumper set on both boards; do **not** add extra resistors.

## Build & flash

```
pio run -e fake-ecu -t upload --upload-port COM5
pio device monitor -e fake-ecu -p COM5        # view the log, then press reset
```

The log (`=== Fake ECU starting ===`, `SELFTEST: ...`, `CAN bus ready ...`) runs
over the native USB port (USB-CDC, see `ARDUINO_USB_CDC_ON_BOOT=1` in `platformio.ini`).

### Build flags (optional)

| Flag | Effect |
|---|---|
| `-DFAKE_ECU_BITRATE_KBPS=125\|250\|500` | bus bitrate (default 500), must match `OBD_CAN_BITRATE_KBPS` of the test partner |
| `-DFAKE_ECU_DEBUG` | 1 Hz heartbeat on `0x555` + TWAI status + raw frame log over Serial (off by default) |
| `-DFAKE_ECU_NO_ACK` | `TWAI_MODE_NO_ACK` – receives but does not acknowledge (bus diagnostics) |

The one-shot CAN **self-test** at boot (loopback through the transceiver) always
runs and reports `SELFTEST: OK` or `SELFTEST: FAILED`.

## What is simulated

| Request | Response |
|---|---|
| Service 0x01 PID 0x0C / 0x0D / 0x05 / 0x11 / 0x42 | RPM (800–3000, sweeping), speed, coolant temp, throttle, 13.8 V |
| Service 0x09 PID 0x02 | VIN `WVWZZZ1KZAW123456` as a **multi-frame** response (exercises ISO-TP + Flow Control) |
| Service 0x03 | 2 DTCs: P0301, P0420 |
| Service 0x04 | positive acknowledgement (DTCs cleared) |
| unknown PID | Negative Response 0x7F ... 0x12 |

Request IDs: `0x7DF` (functional) and `0x7E0` (physical), response: `0x7E8`, 500 kBit/s, 11-bit.
