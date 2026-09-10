// This program is free software; you can use it, redistribute it
// and / or modify it under the terms of the GNU General Public License
// (GPL) as published by the Free Software Foundation; either version 3
// of the License or (at your option) any later version.
//
// obd_can_config.h
//
// Board-specific settings for the native CAN connection (USE_CAN).
// Override values here or via build_flags (-D OBD_CAN_TX_PIN=xx) if the chosen
// pins clash with the modem/GPS/SD interface of your board (see
// device_sim7xxx.h / device_simA76xx.h for the pins already in use).

#pragma once

#ifndef OBD2_MQTT_OBD_CAN_CONFIG_H
#define OBD2_MQTT_OBD_CAN_CONFIG_H

// Tested as free on T-SIM7070G / T-SIM7000G WITHOUT a SimShield.
// SimShield users: GPIO32 clashes with SIMSHIELD_SD_CS, move it then!
#ifndef OBD_CAN_TX_PIN
#define OBD_CAN_TX_PIN          GPIO_NUM_32
#endif

#ifndef OBD_CAN_RX_PIN
#define OBD_CAN_RX_PIN          GPIO_NUM_33
#endif

// The vast majority of vehicles (incl. the VW MQB platform, e.g. e-Golf) use
// 500 kBit/s with 11-bit identifiers (ISO 15765-4) at the OBD2 diagnostic
// gateway. Adjust here if your vehicle uses 29-bit IDs or a different baud rate.
#ifndef OBD_CAN_USE_EXTENDED_ID
#define OBD_CAN_USE_EXTENDED_ID  0
#endif

// Bus bitrate in kBit/s. ISO 15765-4 allows 250 and 500; almost all cars use
// 500. Use 125 only as a diagnostic aid for a flaky harness (slower edges).
#ifndef OBD_CAN_BITRATE_KBPS
#define OBD_CAN_BITRATE_KBPS     500
#endif

#if OBD_CAN_BITRATE_KBPS == 500
#define OBD_CAN_TIMING_CONFIG    TWAI_TIMING_CONFIG_500KBITS()
#elif OBD_CAN_BITRATE_KBPS == 250
#define OBD_CAN_TIMING_CONFIG    TWAI_TIMING_CONFIG_250KBITS()
#elif OBD_CAN_BITRATE_KBPS == 125
#define OBD_CAN_TIMING_CONFIG    TWAI_TIMING_CONFIG_125KBITS()
#else
#error "OBD_CAN_BITRATE_KBPS: only 125, 250 or 500 are supported"
#endif

// Standard OBD2 addressing (ISO 15765-4)
#define OBD_CAN_REQUEST_ID       0x7DF   // functional request (broadcast to all ECUs)
#define OBD_CAN_RESPONSE_ID_MIN  0x7E8   // first possible ECU response ID
#define OBD_CAN_RESPONSE_ID_MAX  0x7EF   // last possible ECU response ID

// Timeouts / retry behaviour
#ifndef OBD_CAN_RESPONSE_TIMEOUT_MS
#define OBD_CAN_RESPONSE_TIMEOUT_MS   1000
#endif

#ifndef OBD_CAN_FLOW_CONTROL_TIMEOUT_MS
#define OBD_CAN_FLOW_CONTROL_TIMEOUT_MS  200
#endif

#endif //OBD2_MQTT_OBD_CAN_CONFIG_H
