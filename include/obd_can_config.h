// This program is free software; you can use it, redistribute it
// and / or modify it under the terms of the GNU General Public License
// (GPL) as published by the Free Software Foundation; either version 3
// of the License or (at your option) any later version.
//
// obd_can_config.h
//
// Board-spezifische Einstellungen fuer die native CAN-Anbindung (USE_CAN).
// Werte hier oder per build_flags (-D OBD_CAN_TX_PIN=xx) ueberschreiben,
// falls die gewaehlten Pins mit dem Modem/GPS/SD-Interface deines Boards
// kollidieren (siehe device_sim7xxx.h / device_simA76xx.h fuer belegte Pins).

#pragma once

#ifndef OBD2_MQTT_OBD_CAN_CONFIG_H
#define OBD2_MQTT_OBD_CAN_CONFIG_H

// Getestet als frei auf T-SIM7070G / T-SIM7000G OHNE SimShield.
// SimShield-Nutzer: GPIO32 kollidiert mit SIMSHIELD_SD_CS, dann verschieben!
#ifndef OBD_CAN_TX_PIN
#define OBD_CAN_TX_PIN          GPIO_NUM_32
#endif

#ifndef OBD_CAN_RX_PIN
#define OBD_CAN_RX_PIN          GPIO_NUM_33
#endif

// Die allermeisten Fahrzeuge (inkl. VW MQB-Plattform, z.B. e-Golf) nutzen
// am OBD2-Diagnose-Gateway 500 kBit/s mit 11-Bit-Identifiern (ISO 15765-4).
// Falls dein Fahrzeug 29-Bit-IDs oder eine andere Baudrate nutzt, hier anpassen.
#ifndef OBD_CAN_USE_EXTENDED_ID
#define OBD_CAN_USE_EXTENDED_ID  0
#endif

// Standard-OBD2-Adressierung (ISO 15765-4)
#define OBD_CAN_REQUEST_ID       0x7DF   // funktionale Anfrage (Broadcast an alle ECUs)
#define OBD_CAN_RESPONSE_ID_MIN  0x7E8   // erste moegliche ECU-Antwort-ID
#define OBD_CAN_RESPONSE_ID_MAX  0x7EF   // letzte moegliche ECU-Antwort-ID

// Timeouts / Retry-Verhalten
#ifndef OBD_CAN_RESPONSE_TIMEOUT_MS
#define OBD_CAN_RESPONSE_TIMEOUT_MS   1000
#endif

#ifndef OBD_CAN_FLOW_CONTROL_TIMEOUT_MS
#define OBD_CAN_FLOW_CONTROL_TIMEOUT_MS  200
#endif

#endif //OBD2_MQTT_OBD_CAN_CONFIG_H
