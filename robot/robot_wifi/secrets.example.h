// Copy this file to secrets.h (git-ignored) and fill it in. Required to build.
#pragma once

// Router the robots join (2.4 GHz, WPA2). Leave WIFI_SSID as "" to always use
// the robot's own network instead.
#define WIFI_SSID "your-router-name"
#define WIFI_PASSWORD "your-router-password"

// Password for the robot's own network "RobotDuel-<id>" (used when it can't join
// the router). At least 8 characters.
#define AP_PASSWORD "choose-a-password"

// Password for wireless firmware updates (laptop/upload_wifi.sh reads it from here).
// Every robot built with this file accepts updates with this password.
#define OTA_PASSWORD "choose-another-password"
