#include "BluetoothHandler.h"
#include "WiFiUtils.h"
#include <Wire.h>
#include <Preferences.h>
#include <BluetoothSerial.h>

BluetoothSerial SerialBT;
Preferences preferences;

String wifiSSID = "";
String wifiPassword = "";

unsigned long previousMillis = 0;
const long interval = 300;
bool updateCredentials = false; // Flag to indicate if credentials should be updated

void initBluetooth() {
  SerialBT.begin("ESP32_Fall_Detection"); // Bluetooth device name
  Serial.println("Begin Bluetooth");

  SerialBT.register_callback(callback);
}

void stopBluetooth() {
  SerialBT.end(); // Stop Bluetooth
  Serial.println("Bluetooth stopped");
}

void obtainWiFiCredentials(const int ledPin, const char* API_KEY, const char* USER_EMAIL, const char* USER_PASSWORD) {
  Serial.println("Getting Wifi Credential");
  //  Check if the Wifi crdentials are empty, the user wants to chnage the Wifi crdentials (updateCredentials) or if the ESP32 is unable to connect to the Wifi (cantConnectFlag)
  while (wifiSSID == "" || wifiPassword == "" || updateCredentials || cantConnectFlag) {

    // Check if it's time to toggle the LED
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      // Toggle LED
      digitalWrite(ledPin, !digitalRead(ledPin));
    }

    // Check for available Bluetooth data
    if (SerialBT.available()) {
      String received = SerialBT.readStringUntil('\n'); // Read until newline character

      if (received.startsWith("SSID:") && received.indexOf(";PASS:") != -1) {
        // Extract SSID and Password
        int passStartIndex = received.indexOf(";PASS:") + 6;
        int passEndIndex = received.lastIndexOf(";");
        wifiSSID = received.substring(5, received.indexOf(";PASS:"));
        wifiPassword = received.substring(passStartIndex, passEndIndex);

        Serial.println("SSID: " + wifiSSID);
        Serial.println("Password: " + wifiPassword);

        // Save the new credentials to preferences
        writeCredentialsToPreferences(wifiSSID, wifiPassword);

        // Clear the received data for the next iteration
        received = "";

        if (cantConnectFlag || updateCredentials) {
          setupWifi(wifiSSID, wifiPassword, ledPin, API_KEY, USER_EMAIL, USER_PASSWORD);  // Receonnect to the new Wifi
          cantConnectFlag = false;  // Clear the cannot update flag
          setUpdateCredentialsFlag(false);  // Clear the update credentials flag
        }
      }
    }
  }
}

void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  if (event == ESP_SPP_SRV_OPEN_EVT) {
    Serial.println("Client Connected");
  }
  if (event == ESP_SPP_CLOSE_EVT) {
    Serial.println("Client Disconnected");
  }
}

// Saves the data from the bluetooth onto the ESP32
void writeCredentialsToPreferences(String ssid, String password) {
  preferences.begin("wifiCreds", false);
  preferences.putString("ssid", ssid);
  preferences.putString("password", password);
  preferences.end();
}

// Read the data from the preferance file
void readCredentialsFromPreferences(String &ssid, String &password) {
  preferences.begin("wifiCreds", true);
  ssid = preferences.getString("ssid", "");
  password = preferences.getString("password", "");
  preferences.end();
}

// Checks if the wifi credentials needs to be updated
void setUpdateCredentialsFlag(bool value) {
  updateCredentials = value;
}
