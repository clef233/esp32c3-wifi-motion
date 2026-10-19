#pragma once

// Copy this file to wifi_config.h and fill in your local 2.4 GHz Wi-Fi credentials.
// wifi_config.h is intentionally excluded from Git.
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

// Empty means ping the gateway assigned by DHCP. Override for a different peer.
#define CSI_PING_TARGET ""
#define CSI_PING_INTERVAL_MS 100
