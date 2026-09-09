# fake-ecu

Standalone-Testfirmware: simuliert eine Fahrzeug-ECU am CAN-Bus, damit
`src/obd_can.cpp` (natives CAN, `-DUSE_CAN`) **ohne echtes Auto** getestet
werden kann.

## Hardware

- 2. Board: LilyGO **T7-S3** (ESP32-S3-WROOM-1-N16R8) + **SN65HVD230** CAN-Transceiver
- CAN-Pins auf dem Testboard: **TX = GPIO13**, **RX = GPIO14** (in `src/main.cpp`)
- Verkabelung zum obd_can_2mqtt-Board:
  - CAN-H ↔ CAN-H, CAN-L ↔ CAN-L, **GND ↔ GND**
  - je **120 Ω** zwischen CAN-H/CAN-L an **beiden** Leitungsenden

## Bauen & Flashen

```
pio run -e fake-ecu -t upload --upload-port COM5
pio device monitor -e fake-ecu -p COM5        # Log ansehen, dann Reset druecken
```

Der Log (`=== Fake-ECU startet ===`, `CAN-Bus bereit ...`) laeuft ueber den
nativen USB-Port (USB-CDC, siehe `ARDUINO_USB_CDC_ON_BOOT=1` in `platformio.ini`).

## Was simuliert wird

| Anfrage | Antwort |
|---|---|
| Service 0x01 PID 0x0C / 0x0D / 0x05 / 0x11 / 0x42 | RPM (800–3000, pendelnd), Speed, Kühlmitteltemp, Drossel, 13.8 V |
| Service 0x09 PID 0x02 | VIN `WVWZZZ1KZAW123456` als **Multi-Frame** (testet ISO-TP + Flow Control) |
| Service 0x03 | 2 DTCs: P0301, P0420 |
| Service 0x04 | positive Quittung (DTCs gelöscht) |
| unbekannte PID | Negative Response 0x7F ... 0x12 |

Request-IDs: `0x7DF` (funktional) und `0x7E0` (physisch), Antwort: `0x7E8`, 500 kBit/s, 11-Bit.
