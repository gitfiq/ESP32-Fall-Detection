#include "WiFiUtils.h"
#include "BluetoothHandler.h"
#include "FirestoreUtils.h"
#include <WiFi.h>
#include <Wire.h>

bool cantConnectFlag = false;

void setupWifi(String WIFI_SSID, String WIFI_PASSWORD, const int ledPin, const char* API_KEY, const char* USER_EMAIL, const char* USER_PASSWORD) {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");

  unsigned long startAttemptTime = millis();

  // Keep trying to connect until successful or timeout (e.g., 20 seconds)
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 20000) {
    Serial.print(".");
    digitalWrite(ledPin, !digitalRead(ledPin));
    delay(300);
  } 

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Failed to connect to Wi-Fi");
    cantConnectFlag = true;
    obtainWiFiCredentials(ledPin, API_KEY, USER_EMAIL, USER_PASSWORD);
  } else {
    // Set the LED to HIGH only when the device is connected
    digitalWrite(ledPin, HIGH);
    Serial.println();
    Serial.print("Connected with IP: ");
    Serial.println(WiFi.localIP());
    
    // Debugging: delay to ensure serial output is visible
    delay(1000);

    // Attempt to reconnect to Firebase
    Serial.println("Attempting to reconnect to Firebase...");
    setFirebase(API_KEY, USER_EMAIL, USER_PASSWORD);  // Reconnect to the Cloud Firestore
    stopBluetooth();  //Stop the bluetooth after wifi credential received.
  }
}

void disconnectWifi() {
  WiFi.disconnect(true);  // Disconnect and delete saved WiFi credentials
  Serial.println("Disconnected from Wi-Fi");
}
