/*
 * This program is free software; you can use it, redistribute it
 * and / or modify it under the terms of the GNU General Public License
 * (GPL) as published by the Free Software Foundation; either version 3
 * of the License or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program, in a file called gpl.txt or license.txt.
 *  If not, write to the Free Software Foundation Inc.,
 *  59 Temple Place - Suite 330, Boston, MA  02111-1307 USA
 */

#include "obd.h"

#include <OBDStates.h>
#include <ExprParser.h>
#include "helper.h"

// Note: in the USE_CAN build "elm327" is NOT an ELMduino object any more, but
// the compatible CAN bridge from obd_can.h/.cpp - all calls below
// (currentDTCCodes(), nb_rx_state, DTC_Response, batteryVoltage(), ...) work
// unchanged because the class offers the same interface.
OBDClass::OBDClass() : OBDStates(&elm327), elm327() {
    protocol = AUTOMATIC;

    addCustomFunction("afRatio", [](const double fuelType) {
        switch (static_cast<int>(fuelType)) {
            case FUEL_TYPE_METHANOL:
                return AF_RATIO_METHANOL;
            case FUEL_TYPE_ETHANOL:
                return AF_RATIO_ETHANOL;
            case FUEL_TYPE_DIESEL:
                return AF_RATIO_DIESEL;
            case FUEL_TYPE_LPG:
            case FUEL_TYPE_CNG:
                return AF_RATIO_GAS;
            case FUEL_TYPE_PROPANE:
                return AF_RATIO_PROPANE;
            case FUEL_TYPE_ELECTRIC:
                return 0.0;
            default:
                return AF_RATIO_GASOLINE;
        }
    });
    addCustomFunction("density", [](const double fuelType)-> double {
        switch (static_cast<int>(fuelType)) {
            case FUEL_TYPE_METHANOL:
                return DENSITY_METHANOL;
            case FUEL_TYPE_ETHANOL:
                return DENSITY_ETHANOL;
            case FUEL_TYPE_DIESEL:
                return DENSITY_DIESEL;
            case FUEL_TYPE_LPG:
            case FUEL_TYPE_CNG:
                return DENSITY_GAS;
            case FUEL_TYPE_PROPANE:
                return DENSITY_PROPANE;
            case FUEL_TYPE_ELECTRIC:
                return 0.0;
            default:
                return DENSITY_GASOLINE;
        }
    });
    addCustomFunction("numDTCs", [&](const double numCodes)-> double {
        int numDTCs = 0;
        if (static_cast<u_int8_t>(numCodes) > 0) {
            elm327.currentDTCCodes();
            if (elm327.nb_rx_state == ELM_SUCCESS) {
                dtcsRead = true;
                dtcs.clear();
                numDTCs = static_cast<int>(elm327.DTC_Response.codesFound);
                if (numDTCs > 0) {
                    for (int i = 0; i < numDTCs; i++) {
                        dtcs.add(elm327.DTC_Response.codes[i]);
                    }
                }
            }
        }
        return numDTCs;
    });

    setVariableResolveFunction([&](const char *varName)-> double {
        if (varName != nullptr) {
            if (varName[0] == '$') {
                varName++;
            }

            if (std::strcmp(varName, "millis") == 0) {
                return millis();
            }
            size_t pos = 0;
            string vstr = varName;
            if ((pos = vstr.find('.')) != string::npos) {
                string vn = vstr.substr(0, pos);
                string op = vstr.substr(pos + 1);

                auto *state = getStateByName(vn.c_str());
                if (state != nullptr) {
                    if (op == "pu") {
                        return state->getPreviousUpdate();
                    }
                    if (op == "lu") {
                        return state->getLastUpdate();
                    }

                    if (op.substr(0, 1) == "b" && op.length() > 1 && (
                            state->valueType() == OBD_STATE_TYPE_INT || state->valueType() == OBD_STATE_TYPE_FLOAT)) {
                        if (state->getPayload() != nullptr) {
                            size_t si = op.find(':');
                            int i = strtol(
                                (si != string::npos ? op.substr(1, si - 1) : op.substr(1)).c_str(),
                                nullptr,
                                10);
                            int j = si != string::npos ? strtol(op.substr(si + 1).c_str(), nullptr, 10) : i + 1;
                            if (j - i <= 8) {
                                int sidx = (i - 1) * 2;
                                int eidx = (j - 1) * 2;
                                if (sidx > 0 && eidx > sidx && sidx <= strlen(state->getPayload()) && eidx <= strlen(
                                        state->getPayload())) {
                                    string bstr = string(state->getPayload()).substr(sidx, eidx - sidx);
                                    return strtol(bstr.c_str(), nullptr, 16);
                                }
                                Serial.println("Index out of bound");
                            } else {
                                Serial.println("Range overflows double");
                            }
                        } else {
                            log_v("Payload was empty");
                        }
                    }

                    if (op == "ov" && state->valueType() == OBD_STATE_TYPE_INT) {
                        auto *is = reinterpret_cast<OBDStateInt *>(state);
                        return is->getOldValue();
                    }
                    if (op == "ov" && state->valueType() == OBD_STATE_TYPE_FLOAT) {
                        auto *is = reinterpret_cast<OBDStateFloat *>(state);
                        return is->getOldValue();
                    }
                    if (op == "ov" && state->valueType() == OBD_STATE_TYPE_BOOL) {
                        auto *is = reinterpret_cast<OBDStateBool *>(state);
                        return is->getOldValue();
                    }
                }
            } else {
                return getStateValue(varName);
            }
        }

        return 0.0;
    });
}

bool OBDClass::parseJSON(std::string &json) {
    bool success = false;
    JsonDocument doc;
    if (!deserializeJson(doc, json)) {
        readJSON(doc);
        success = true;
    }

    return success;
}

template<typename T>
void OBDClass::fromJSON(T *state, JsonDocument &doc) {
    state->setEnabled(doc["enabled"].as<bool>());
    state->setVisible(doc["visible"].as<bool>());

    if (state->getType() == obd::READ) {
        if (!doc["readFunc"].isNull()) {
            setReadFuncByName<T>(doc["readFunc"].as<std::string>().c_str(), state);
        } else if (!doc["pid"].isNull()) {
            state->setPIDSettings(
                doc["pid"]["service"].as<uint8_t>(),
                doc["pid"]["pid"].as<uint16_t>(),
                doc["pid"]["header"].as<uint32_t>(),
                doc["pid"]["numResponses"].as<uint8_t>(),
                doc["pid"]["numExpectedBytes"].as<uint8_t>(),
                doc["pid"]["responseFormat"].as<obd::OBDResponseFormat>(),
                !doc["pid"]["scaleFactor"].isNull() ? doc["pid"]["scaleFactor"].as<std::string>().c_str() : "1",
                doc["pid"]["bias"].as<float>()
            );
        }
    } else if (state->getType() == obd::CALC) {
        if (!doc["expr"].isNull()) {
            state->setCalcExpression(doc["expr"].as<std::string>().c_str());
        }
    }

    if (!doc["value"]["format"].isNull()) {
        state->setValueFormat(doc["value"]["format"].as<std::string>().c_str());
    }
    if (!doc["value"]["func"].isNull()) {
        setFormatFuncByName<T>(doc["value"]["func"].as<std::string>().c_str(), state);
    } else if (!doc["value"]["expr"].isNull()) {
        state->setValueFormatExpression(doc["value"]["expr"].as<std::string>().c_str());
    }

    // is reset by setPIDSettings
    state->setUpdateInterval(doc["interval"].as<long>());
}

bool OBDClass::readStates(FS &fs) {
    bool success = false;

    File file = fs.open(STATES_FILE, FILE_READ);
    if (file && !file.isDirectory()) {
        JsonDocument doc;
        if (!deserializeJson(doc, file)) {
            readJSON(doc);
            success = true;
        }
        file.close();
    }

    return success;
}

std::string OBDClass::buildJSON() {
    std::string payload;

    JsonDocument doc;
    writeJSON(doc);
    serializeJson(doc, payload);

    return payload;
}

void OBDClass::readJSON(JsonDocument &doc) {
    clearStates();
    const JsonArray array = doc.as<JsonArray>();
    for (JsonDocument stateObj: array) {
        if (stateObj["valueType"] == OBD_STATE_TYPE_BOOL) {
            auto *state = new OBDStateBool(
                stateObj["type"].as<obd::OBDStateType>(),
                stateObj["name"].as<std::string>().c_str(),
                stateObj["description"].as<std::string>().c_str(),
                !stateObj["icon"].isNull() ? stateObj["icon"].as<std::string>().c_str() : "",
                !stateObj["unit"].isNull() ? stateObj["unit"].as<std::string>().c_str() : "",
                !stateObj["deviceClass"].isNull() ? stateObj["deviceClass"].as<std::string>().c_str() : "",
                stateObj["measurement"].as<bool>(),
                stateObj["diagnostic"].as<bool>()
            );
            fromJSON(state, stateObj);
            addState(state);
        } else if (stateObj["valueType"] == OBD_STATE_TYPE_FLOAT) {
            auto *state = new OBDStateFloat(
                stateObj["type"].as<obd::OBDStateType>(),
                stateObj["name"].as<std::string>().c_str(),
                stateObj["description"].as<std::string>().c_str(),
                !stateObj["icon"].isNull() ? stateObj["icon"].as<std::string>().c_str() : "",
                !stateObj["unit"].isNull() ? stateObj["unit"].as<std::string>().c_str() : "",
                !stateObj["deviceClass"].isNull() ? stateObj["deviceClass"].as<std::string>().c_str() : "",
                stateObj["measurement"].as<bool>(),
                stateObj["diagnostic"].as<bool>()
            );
            fromJSON(state, stateObj);
            addState(state);
        } else if (stateObj["valueType"] == OBD_STATE_TYPE_INT) {
            auto *state = new OBDStateInt(
                stateObj["type"].as<obd::OBDStateType>(),
                stateObj["name"].as<std::string>().c_str(),
                stateObj["description"].as<std::string>().c_str(),
                !stateObj["icon"].isNull() ? stateObj["icon"].as<std::string>().c_str() : "",
                !stateObj["unit"].isNull() ? stateObj["unit"].as<std::string>().c_str() : "",
                !stateObj["deviceClass"].isNull() ? stateObj["deviceClass"].as<std::string>().c_str() : "",
                stateObj["measurement"].as<bool>(),
                stateObj["diagnostic"].as<bool>()
            );
            fromJSON(state, stateObj);
            addState(state);
        }
    }
}

void OBDClass::writeJSON(JsonDocument &doc) {
    std::vector<OBDState *> states{};
    getStates([](const OBDState *) {
        return true;
    }, states);
    for (OBDState *state: states) {
        JsonDocument stateObj;
        state->toJSON(stateObj);
        doc.add(stateObj);
    }
}

bool OBDClass::writeStates(FS &fs) {
    bool success = false;

    File file = fs.open(STATES_FILE, FILE_WRITE);
    if (!file) {
        Serial.println("Failed to open file settings.json for writing.");
        return false;
    }

    JsonDocument doc;
    writeJSON(doc);
    success = serializeJson(doc, file);

    file.close();

    return success;
}

template<typename T>
T *OBDClass::setReadFuncByName(const char *funcName, T *state) {
    if (strcmp(funcName, "batteryVoltage") == 0 && strcmp(state->valueType(), OBD_STATE_TYPE_FLOAT) == 0) {
        state
                ->withReadFuncName("batteryVoltage")
                ->withReadFunc([&]() {
                    return elm327.batteryVoltage();
                });
    }

    return state;
}

template<typename T>
T *OBDClass::setFormatFuncByName(const char *funcName, T *state) {
    if (strcmp(state->valueType(), OBD_STATE_TYPE_INT) == 0) {
        auto *is = reinterpret_cast<OBDStateInt *>(state);
        if (strcmp(funcName, "toBitStr") == 0) {
            is
                    ->withValueFormatFuncName("toBitStr")
                    ->withValueFormatFunc([](const int value) {
                        char str[33];
                        snprintf(str, sizeof(str), "%s", std::bitset<32>(value).to_string().c_str());
                        return strdup(str);
                    });
        } else if (strcmp(funcName, "toMiles") == 0) {
            is
                    ->withValueFormatFuncName("toMiles")
                    ->withValueFormatFunc([](const int value) {
                        char str[16];
                        snprintf(str, sizeof(str), "%d", static_cast<int>(static_cast<float>(value) / KPH_TO_MPH));
                        return strdup(str);
                    });
        } else if (strcmp(funcName, "payload") == 0) {
            is
                    ->withValueFormatFuncName("payload")
                    ->withValueFormatFunc([is](const int value) {
                        return strdup(is->getPayload());
                    });
        }
    } else if (strcmp(state->valueType(), OBD_STATE_TYPE_FLOAT) == 0) {
        auto *is = reinterpret_cast<OBDStateFloat *>(state);
        if (strcmp(funcName, "toMiles") == 0) {
            is
                    ->withValueFormatFuncName("toMiles")
                    ->withValueFormatFunc([](const float value) {
                        char str[16];
                        snprintf(str, sizeof(str), "%4.2f", value / KPH_TO_MPH);
                        return strdup(str);
                    });
        } else if (strcmp(funcName, "toGallons") == 0) {
            is
                    ->withValueFormatFuncName("toGallons")
                    ->withValueFormatFunc([](const float value) {
                        char str[16];
                        snprintf(str, sizeof(str), "%4.2f", value / LITER_TO_GALLON);
                        return strdup(str);
                    });
        } else if (strcmp(funcName, "toMPG") == 0) {
            is
                    ->withValueFormatFuncName("toMPG")
                    ->withValueFormatFunc([&](const float value) {
                        char str[16];
                        snprintf(str, sizeof(str), "%4.2f", value == 0.0f ? 0.0f : 235.214583333333f / value);
                        return strdup(str);
                    });
        } else if (strcmp(funcName, "payload") == 0) {
            is
                    ->withValueFormatFuncName("payload")
                    ->withValueFormatFunc([is](const float value) {
                        return strdup(is->getPayload());
                    });
        }
    }
    return state;
}

int DTCs::getCount() const {
    return static_cast<int>(v_codes.size());
}

std::string *DTCs::getCode(int i) {
    if (i < 0) {
        return nullptr;
    }
    return &v_codes[i];
}

bool DTCs::add(const std::string &code) {
    bool found = false;
    for (const auto &c: v_codes) {
        if (c == code) return false;
    }

    v_codes.push_back(code);
    return true;
}

void DTCs::clear() {
    v_codes.clear();
}

#if !defined(USE_BLE) && !defined(USE_CAN)
void OBDClass::BTEvent(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
    if (event == ESP_SPP_CLOSE_EVT) {
        Serial.println("Bluetooth disconnected.");

        if (OBD.initDone && !OBD.stopConnect) {
            // FIXME get reconnect working - failed with "getChannels() failed timeout"
            // OBD.connect(true);
            ESP.restart();
        }
    }
}

BTScanResults *OBDClass::discoverBtDevices() {
    serialBt.discoverClear();

    Serial.println("Discover Bluetooth devices...");

    BTScanResults *btDeviceList = serialBt.getScanResults(); // maybe accessing from different threads!
    if (serialBt.discoverAsync([](BTAdvertisedDevice *pDevice) {
        Serial.printf(">>>>>>>>>>>Found a new device: %s\n", pDevice->toString().c_str());
    })) {
        delay(BT_DISCOVER_TIME);
        Serial.print("Stopping discover...");
        serialBt.discoverAsyncStop();
        Serial.println("stopped");
        delay(5000);

        if (btDeviceList->getCount() > 0) {
            return btDeviceList;
        }
    }

    return nullptr;
}
#endif

#ifdef USE_BLE
void OBDClass::onBLEDisconnect() {
    Serial.println("Bluetooth LE disconnected.");

    if (OBD.initDone && !OBD.stopConnect) {
        // FIXME get reconnect working
        // OBD.connect(true);
        ESP.restart();
    }
}

BLEScanResultsSet *OBDClass::discoverBLEDevices() {
    serialBLE.discoverClear();

    BLEScanResultsSet *bleDeviceList = serialBLE.getScanResults();
    Serial.println("Discover Bluetooth LE devices...");
    if (serialBLE.discoverAsync([](const NimBLEAdvertisedDevice *pDevice) {
        Serial.printf(">>>>>>>>>>>Found a new device: %s\n", pDevice->toString().c_str());
    })) {
        delay(BT_DISCOVER_TIME);
        Serial.print("Stopping discover...");
        serialBLE.discoverAsyncStop();
        Serial.println("stopped");
        delay(5000);

        if (bleDeviceList != nullptr && bleDeviceList->getCount() > 0) {
            return bleDeviceList;
        }
    }

    return nullptr;
}
#endif

void OBDClass::begin(const String &devName, const String &devMac, const char protocol,
                     const bool checkPidSupport, const bool debug, const bool specifyNumResponses) {
    this->devName = devName;
    this->devMac = devMac;
    this->protocol = protocol;
    this->checkPidSupport = checkPidSupport;
    this->debug = debug;
    this->specifyNumResponses = specifyNumResponses;
    stopConnect = false;
    if (diagMux == nullptr) {
        diagMux = xSemaphoreCreateMutex();
    }
#ifndef USE_CAN
#ifdef USE_BLE
    serialBLE.onDisconnect(onBLEDisconnect);
#else
    serialBt.register_callback(BTEvent);
#endif
#endif
}

void OBDClass::end() {
    stopConnect = true;
#ifdef USE_CAN
    elm327.end();
#else
#ifdef USE_BLE
    serialBLE.disconnect();
    serialBLE.end();
#else
    serialBt.disconnect();
    serialBt.end();
#endif
#endif
}

#ifdef USE_CAN
void OBDClass::connect(bool reconnect) {
    stopConnect = false;

    if (stopConnect || (reconnect && !initDone)) {
        return;
    }

    // No device pairing like Bluetooth needed - just (re)start the TWAI
    // driver. On failure retry with a backoff.
    int retryCount = 0;
    while (!elm327.begin() && retryCount < 3) {
        Serial.println("Couldn't start CAN interface - retrying");
        delay(BT_DISCOVER_TIME);
        retryCount++;
    }

    if (!elm327.connected) {
        delay(BT_DISCOVER_TIME);
        Serial.println("Restarting OBD (CAN) connect.");
        elm327.end();
        if (connectErrorCallback) {
            connectErrorCallback();
        }
        if (!stopConnect) {
            connect(reconnect);
        }
        return;
    }

    Serial.println("Connected to CAN bus");

    if (connectedCallback) {
        connectedCallback();
    }

    if (!reconnect) {
        setCheckPidSupport(this->checkPidSupport);
        elm327.specifyNumResponses = this->specifyNumResponses;
        initDone = true;
    }
}
#else
void OBDClass::connect(bool reconnect) {
    stopConnect = false;

connect:
    if (stopConnect || reconnect && !initDone) {
        return;
    }

#ifdef USE_BLE
    if (!serialBLE.begin("OBD2MQTT")) {
        Serial.println("========== serialBLE failed!");
        ESP.restart();
    }
#else
    if (!serialBt.begin("OBD2MQTT", true)) {
        Serial.println("========== serialBT failed!");
        ESP.restart();
    }
#endif

    if (devMac.isEmpty()) {
#ifdef USE_BLE
        BLEScanResultsSet *bleDeviceList = discoverBLEDevices();

        if (bleDeviceList == nullptr) {
            Serial.println("Didn't find any devices");
            if (connectErrorCallback) {
                connectErrorCallback();
            }
        } else {
            NimBLEAddress addr = NimBLEAddress();

            if (devDiscoveredCallback != nullptr && bleDeviceList->getCount() != 0) {
                devDiscoveredCallback(bleDeviceList);
            }

            Serial.printf("Search device: %s\n", devName.c_str());
            for (int i = 0; i < bleDeviceList->getCount(); i++) {
                NimBLEAdvertisedDevice *device = bleDeviceList->getDevice(i);
                if (strcmp(device->getName().c_str(), devName.c_str()) == 0) {
                    Serial.printf(" ----- %s  %s %d\n", device->getAddress().toString().c_str(),
                                  device->getName().c_str(), device->getRSSI());
                    addr = NimBLEAddress(device->getAddress());
                }
            }

            if (!stopConnect && addr) {
                Serial.printf("connecting to %s\n", addr.toString().c_str());
                if (serialBLE.connect(addr)) {
                    connectedBTAddress = addr.toString();
                }
            }
        }
#else
        BTScanResults *btDeviceList = discoverBtDevices();

        if (btDeviceList == nullptr) {
            Serial.println("Didn't find any devices");
            if (connectErrorCallback) {
                connectErrorCallback();
            }
        } else {
            BTAddress addr;
            int channel = 0;

            if (devDiscoveredCallback != nullptr && btDeviceList->getCount() != 0) {
                devDiscoveredCallback(btDeviceList);
            }

            Serial.printf("Search device: %s\n", devName.c_str());
            for (int i = 0; i < btDeviceList->getCount(); i++) {
                BTAdvertisedDevice *device = btDeviceList->getDevice(i);
                if (strcmp(device->getName().c_str(), devName.c_str()) == 0) {
                    Serial.printf(" ----- %s  %s %d\n", device->getAddress().toString().c_str(),
                                  device->getName().c_str(), device->getRSSI());
                    std::map<int, std::string> channels = serialBt.getChannels(device->getAddress());
                    Serial.printf("scanned for services, found %d\n", channels.size());
                    for (auto const &entry: channels) {
                        Serial.printf("     channel %d (%s)\n", entry.first, entry.second.c_str());
                    }
                    if (!channels.empty()) {
                        addr = device->getAddress();
                        channel = channels.begin()->first;
                    }
                }
            }

            if (!stopConnect && addr) {
                Serial.printf("connecting to %s - %d\n", addr.toString().c_str(), channel);
                if (serialBt.connect(addr, channel, ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE)) {
                    connectedBTAddress = addr.toString().c_str();
                }
            }
        }
#endif
    } else {
        byte mac[6];
        parseBytes(devMac.c_str(), ':', mac, 6, 16);
#ifdef USE_BLE
        NimBLEAddress addr = NimBLEAddress(mac, 0);

        if (!stopConnect && addr) {
            Serial.printf("connecting to %s\n", addr.toString().c_str());
            if (serialBLE.connect(addr)) {
                connectedBTAddress = addr.toString();
            }
        }
#else
        BTAddress addr = mac;
        int channel = 0;

        std::map<int, std::string> channels = serialBt.getChannels(addr);
        Serial.printf("scanned for services, found %d\n", channels.size());
        for (auto const &entry: channels) {
            Serial.printf("     channel %d (%s)\n", entry.first, entry.second.c_str());
        }

        if (!channels.empty()) {
            channel = channels.begin()->first;
        }

        if (!stopConnect && addr) {
            Serial.printf("connecting to %s - %d\n", addr.toString().c_str(), channel);
            if (serialBt.connect(addr, channel, ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE)) {
                connectedBTAddress = addr.toString().c_str();
            }
        }
#endif
    }

#ifdef USE_BLE
    if (!stopConnect && !serialBLE.isClosed() && serialBLE.connected()) {
        int retryCount = 0;
        while (!elm327.begin(serialBLE, debug, 2000, protocol) && retryCount < 3) {
            Serial.println("Couldn't connect to OBD scanner - Phase 2");
            delay(BT_DISCOVER_TIME);
            retryCount++;
        }
#else
    if (!stopConnect && !serialBt.isClosed() && serialBt.connected()) {
        int retryCount = 0;
        while (!elm327.begin(serialBt, debug, 2000, protocol) && retryCount < 3) {
            Serial.println("Couldn't connect to OBD scanner - Phase 2");
            delay(BT_DISCOVER_TIME);
            retryCount++;
        }
#endif
    } else if (!stopConnect) {
        Serial.println("Couldn't connect to OBD scanner - Phase 1");
    }

    // if connection stopped (AP connected) wait before reconnect
    while (stopConnect) {
        delay(BT_DISCOVER_TIME);
    }

    if (!elm327.connected) {
        delay(BT_DISCOVER_TIME);
        Serial.println("Restarting OBD connect.");
#ifdef USE_BLE
        serialBLE.end();
#else
        serialBt.end();
#endif
        if (connectErrorCallback) {
            connectErrorCallback();
        }
        goto connect;
    }

    Serial.println("Connected to ELM327");

    if (connectedCallback) {
        connectedCallback();
    }

    if (!reconnect) {
        if (protocol == AUTOMATIC &&
            elm327.sendCommand_Blocking("AT DP") == ELM_SUCCESS &&
            strlen(elm327.payload) > 0) {
            auto protocol = String(elm327.payload);
            protocol.replace("AUTO", "");
            Serial.printf("ELM327 protocol: %s\n", protocol.c_str());
        }
        setCheckPidSupport(this->checkPidSupport);
        elm327.specifyNumResponses = this->specifyNumResponses;
        initDone = true;
    }
}
#endif

void OBDClass::loop() {
#ifdef USE_CAN
    if (!stopConnect && elm327.connected) {
#elif defined(USE_BLE)
    if (!stopConnect && serialBLE && !serialBLE.isClosed()) {
#else
    if (!stopConnect && serialBt && !serialBt.isClosed()) {
#endif
#ifdef DEBUG_OBDSTATE
        OBDState *state = nextState();
        if (state != nullptr && state->getType() == obd::READ && state->getLastUpdate() != -1 && state->isSupported()) {
            if (state->valueType() == OBD_STATE_TYPE_INT) {
                auto s = reinterpret_cast<TypedOBDState<int> *>(state);
                Serial.printf("%s : %d -> %d\n", s->getName(), s->getOldValue(), s->getValue());
            }
            if (state->valueType() == OBD_STATE_TYPE_FLOAT) {
                auto s = reinterpret_cast<TypedOBDState<float> *>(state);
                Serial.printf("%s : %4.2f -> %4.2f\n", s->getName(), s->getOldValue(), s->getValue());
            }
            if (state->valueType() == OBD_STATE_TYPE_BOOL) {
                auto s = reinterpret_cast<TypedOBDState<bool> *>(state);
                Serial.printf("%s %d -> %d\n", s->getName(), s->getOldValue(), s->getValue());
            }
        }
#else
        nextState();
#endif
        serviceDiagScan();
    } else {
        delay(500);
    }

    buildLiveJson();
}

void OBDClass::buildLiveJson() {
    if (diagMux == nullptr || millis() - lastLiveBuild < 1000) {
        return;
    }
    lastLiveBuild = millis();

    std::vector<OBDState *> all;
    getStates([](OBDState *) { return true; }, all);

    JsonDocument doc;
    JsonArray arr = doc.to<JsonArray>();
    for (auto *s: all) {
        if (s == nullptr || (s->getType() != obd::READ && s->getType() != obd::CALC)) {
            continue;
        }
        JsonObject o = arr.add<JsonObject>();
        o["name"] = s->getName();
        o["desc"] = s->getDescription();
        o["unit"] = s->getUnit();
        o["diag"] = s->isDiagnostic();
        o["enabled"] = s->isEnabled();
        o["calc"] = s->getType() == obd::CALC;
        o["svc"] = s->getService();
        o["pid"] = s->getPid();
        o["hdr"] = s->getHeader();
        o["status"] = s->getUpdateStatus();
        const long lu = s->getLastUpdate();
        o["age"] = lu > 0 ? static_cast<long>(millis() - lu) : -1;

        const char *raw = s->getPayload();
        o["raw"] = raw != nullptr ? raw : "";

        if (strcmp(s->valueType(), OBD_STATE_TYPE_INT) == 0) {
            o["value"] = reinterpret_cast<TypedOBDState<int> *>(s)->getValue();
        } else if (strcmp(s->valueType(), OBD_STATE_TYPE_FLOAT) == 0) {
            o["value"] = reinterpret_cast<TypedOBDState<float> *>(s)->getValue();
        } else if (strcmp(s->valueType(), OBD_STATE_TYPE_BOOL) == 0) {
            o["value"] = reinterpret_cast<TypedOBDState<bool> *>(s)->getValue();
        }
    }

    std::string out;
    serializeJson(doc, out);

    if (xSemaphoreTake(diagMux, pdMS_TO_TICKS(50)) == pdTRUE) {
        liveJson.swap(out);
        xSemaphoreGive(diagMux);
    }
}

std::string OBDClass::liveDataJSON() {
    if (diagMux == nullptr) {
        return "[]";
    }
    std::string copy = "[]";
    if (xSemaphoreTake(diagMux, pdMS_TO_TICKS(200)) == pdTRUE) {
        copy = liveJson;
        xSemaphoreGive(diagMux);
    }
    return copy;
}

bool OBDClass::startDiagScan(const uint8_t service, const uint16_t from, const uint16_t to, const uint32_t header) {
#ifdef USE_CAN
    if (diagMux == nullptr || to < from || static_cast<uint32_t>(to - from) > 4095) {
        return false;
    }
    bool ok = false;
    if (xSemaphoreTake(diagMux, pdMS_TO_TICKS(200)) == pdTRUE) {
        if (!diagScan.running && !diagScan.requested) {
            diagScan.service = service;
            diagScan.header = header;
            diagScan.from = from;
            diagScan.to = to;
            diagScan.cur = from;
            diagScan.results.clear();
            diagScan.requested = true;
            ok = true;
        }
        xSemaphoreGive(diagMux);
    }
    return ok;
#else
    (void) service; (void) from; (void) to; (void) header;
    return false;
#endif
}

void OBDClass::serviceDiagScan() {
#ifdef USE_CAN
    if (diagMux == nullptr || (!diagScan.requested && !diagScan.running)) {
        return;
    }

    uint8_t service;
    uint32_t header;
    uint16_t cur, to;
    if (xSemaphoreTake(diagMux, pdMS_TO_TICKS(50)) != pdTRUE) {
        return;
    }
    if (diagScan.requested) {
        diagScan.requested = false;
        diagScan.running = true;
        diagScan.cur = diagScan.from;
    }
    service = diagScan.service;
    header = diagScan.header;
    cur = diagScan.cur;
    to = diagScan.to;
    xSemaphoreGive(diagMux);

    const uint32_t savedTimeout = elm327.getResponseTimeout();
    elm327.setResponseTimeout(250);

    char sh[16];
    if (header != 0) {
        snprintf(sh, sizeof(sh), "AT SH %lX", static_cast<unsigned long>(header));
        elm327.sendCommand_Blocking(sh);
    }

    uint16_t pid = cur;
    for (int batch = 0; pid <= to && batch < 4; ++pid, ++batch) {
        elm327.processPID(service, pid, 1, 8);
        const bool ok = elm327.nb_rx_state == ELM_SUCCESS && strlen(elm327.payload) > 0;
        const uint8_t nrc = elm327.lastNrc;
        // Skip the "definitely not there" answers (0x11 serviceNotSupported,
        // 0x31 requestOutOfRange). Keep positives and interesting NRCs like
        // 0x33 (security), 0x22 (conditions), 0x7E/0x7F (needs a session).
        if (ok || (nrc != 0 && nrc != 0x11 && nrc != 0x31)) {
            DiagScanResult r{};
            r.pid = pid;
            r.raw = ok ? elm327.payload : "";
            r.nrc = ok ? 0 : nrc;
            if (xSemaphoreTake(diagMux, pdMS_TO_TICKS(50)) == pdTRUE) {
                if (diagScan.results.size() < 512) {
                    diagScan.results.push_back(r);
                }
                xSemaphoreGive(diagMux);
            }
        }
    }

    if (header != 0) {
        elm327.sendCommand_Blocking("AT D");
    }
    elm327.setResponseTimeout(savedTimeout);

    if (xSemaphoreTake(diagMux, pdMS_TO_TICKS(50)) == pdTRUE) {
        diagScan.cur = pid;
        if (pid > to) {
            diagScan.running = false;
        }
        xSemaphoreGive(diagMux);
    }
#endif
}

std::string OBDClass::diagScanJSON() {
    JsonDocument doc;
    if (diagMux != nullptr && xSemaphoreTake(diagMux, pdMS_TO_TICKS(200)) == pdTRUE) {
        doc["running"] = diagScan.running || diagScan.requested;
        doc["service"] = diagScan.service;
        doc["header"] = diagScan.header;
        doc["from"] = diagScan.from;
        doc["to"] = diagScan.to;
        const int total = diagScan.to - diagScan.from + 1;
        doc["total"] = total;
        doc["done"] = (diagScan.running || diagScan.requested)
                          ? diagScan.cur - diagScan.from
                          : total;
        JsonArray a = doc["results"].to<JsonArray>();
        for (auto &r: diagScan.results) {
            JsonObject o = a.add<JsonObject>();
            o["pid"] = r.pid;
            o["raw"] = r.raw;
            o["nrc"] = r.nrc;
        }
        xSemaphoreGive(diagMux);
    } else {
        doc["running"] = false;
    }
    std::string out;
    serializeJson(doc, out);
    return out;
}

void OBDClass::onConnected(const std::function<void()> &callback) {
    connectedCallback = callback;
}

void OBDClass::onConnectError(const std::function<void()> &callback) {
    connectErrorCallback = callback;
}

DTCs *OBDClass::getDTCs() {
    return dtcsRead ? &dtcs : nullptr;
}

bool OBDClass::resetDTCs() {
    return elm327.resetDTC();
}

#ifndef USE_CAN
#ifdef USE_BLE
void OBDClass::onDevicesDiscovered(const std::function<void(BLEScanResultsSet * scanResult)> &callable) {
    devDiscoveredCallback = callable;
}
#else
void OBDClass::onDevicesDiscovered(const std::function<void(BTScanResults *scanResult)> &callable) {
    devDiscoveredCallback = callable;
}
#endif
#endif

std::string OBDClass::getConnectedBTAddress() const {
    return connectedBTAddress;
}

uint16_t OBDClass::getPayloadLength() const {
    return elm327.PAYLOAD_LEN;
}

OBDClass OBD;
