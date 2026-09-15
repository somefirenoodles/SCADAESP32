#pragma once

// Local SoftAP served by the ESP32. Leave the password empty for an open
// commissioning network, or use at least eight characters for WPA2.
#define SCADA_AP_SSID "SCADA-EV"
#define SCADA_AP_PASSWORD ""

// Copy this file as secrets.h and replace only the values.
#define SCADA_WIFI_SSID "CHANGE_ME"
#define SCADA_WIFI_PASSWORD "CHANGE_ME"

// Private IPv4 or DNS name of the Ubuntu Mosquitto server.
#define SCADA_MQTT_HOST "192.168.91.215"
#define SCADA_MQTT_PORT 1883
#define SCADA_MQTT_USERNAME "ev"
#define SCADA_MQTT_PASSWORD "CHANGE_ME"

// Use 0 for the first LAN test on port 1883. Before production on a shared
// network, enable a TLS listener in Mosquitto, set this to 1 and paste the CA.
#define SCADA_MQTT_USE_TLS 0
#define SCADA_MQTT_CA_CERT ""

#define SCADA_DEVICE_ID "esp32"
#define SCADA_MQTT_TOPIC "ev"
