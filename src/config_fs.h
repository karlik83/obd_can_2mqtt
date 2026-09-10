/*
 * config_fs.h
 *
 * The device configuration (/settings.json, /states.json) lives on a dedicated
 * "cfg" filesystem partition instead of the main "spiffs" partition that also
 * holds the web UI. `pio run -t uploadfs` only rewrites "spiffs", so a UI update
 * no longer wipes the configuration.
 *
 * Boards whose partition table has no "cfg" partition (or an older firmware)
 * transparently fall back to LittleFS, exactly like before.
 */
#pragma once

#include <FS.h>

// Mount the "cfg" partition. On the first boot after it was added, copy an
// existing /settings.json and /states.json over from LittleFS so the upgrade
// does not lose anything. Call once, right after LittleFS.begin().
void initConfigFs();

// The filesystem that holds /settings.json and /states.json - the "cfg"
// partition when available, otherwise LittleFS.
fs::FS &configFs();
