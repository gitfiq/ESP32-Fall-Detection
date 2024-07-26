#ifndef BLUETOOTH_HANDLER_H
#define BLUETOOTH_HANDLER_H

#include <BluetoothSerial.h>
#include <Preferences.h>

// Extern declarations for variables defined in BluetoothHandler.cpp
extern String wifiSSID;
extern String wifiPassword;
extern bool updateCredentials;

// Function declarations
void initBluetooth();
void stopBluetooth();
void obtainWiFiCredentials(const int ledPin, const char* API_KEY, const char* USER_EMAIL, const char* USER_PASSWORD);
void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);
void writeCredentialsToPreferences(String ssid, String password);
void readCredentialsFromPreferences(String &ssid, String &password);
void setUpdateCredentialsFlag(bool value);

#endif // BLUETOOTH_HANDLER_H
