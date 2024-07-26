#ifndef WIFI_UTILS_H
#define WIFI_UTILS_H

#include <WiFi.h>

extern bool cantConnectFlag;

void setupWifi(String WIFI_SSID, String WIFI_PASSWORD, const int ledPin, const char* API_KEY, const char* USER_EMAIL, const char* USER_PASSWORD);  // Set up the Wifi
void disconnectWifi();  // Disconnect from Wifi

#endif