#include "FirestoreUtils.h"
#include <Firebase_ESP_Client.h>
#include <addons/TokenHelper.h>


// Define Firebase Data object, Firebase authentication, configuration and timestampData
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
FirebaseData timestampData;

double sensitiveLevel = 0.65; // Define the sensitiveLevel variable

void setFirebase(const char* API_KEY, const char* USER_EMAIL, const char* USER_PASSWORD) {

  // Assign the API key
  config.api_key = API_KEY;

  // Assign the user sign-in credentials
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;

  // Assign the callback function for the long-running token generation task
  config.token_status_callback = tokenStatusCallback; 

  // Begin Firebase with configuration and authentication
  Firebase.begin(&config, &auth);

  // Reconnect to Wi-Fi if necessary
  Firebase.reconnectWiFi(true);
}

void readFirestore(const char* FIREBASE_PROJECT_ID, const char* userId, bool initalStatus) {

  char documentPath[100];
  snprintf(documentPath, sizeof(documentPath), "users/%s", userId);

  if (Firebase.Firestore.getDocument(&fbdo, FIREBASE_PROJECT_ID, "", documentPath)) {
    if (fbdo.httpCode() == FIREBASE_ERROR_HTTP_CODE_OK) {

      // Create a FirebaseJson object and set content with received payload
      FirebaseJson readjson;
      readjson.setJsonData(fbdo.payload().c_str());
      FirebaseJsonData readjsonData;

      if (initalStatus) {
        // Check for the username field
        bool initUsername = true;
        if (readjson.get(readjsonData, "fields/username/stringValue", true)) {
          String username = readjsonData.stringValue;
          if (username.length() != 0) {
            initUsername = false;
          }
        } else {
          Serial.println("Username field does not exist.");
        }

        // Check for the sensitivity level field
        bool initSensitivityLevel = true;
        if (readjson.get(readjsonData, "fields/sensitivity/doubleValue", true)) {
          initSensitivityLevel = false;
        } else {
          Serial.println("Sensitivity level field does not exist.");
        }

        // Check for the longitude and latitude field
        bool initLocation = true;
        if (readjson.get(readjsonData, "fields/longitude/doubleValue", true)) {
          float dLongitude = readjsonData.doubleValue;
          if (readjson.get(readjsonData, "fields/latitude/doubleValue", true)) {
            float dLatitude = readjsonData.doubleValue;
              if (dLatitude != 0.0 && dLongitude != 0.0) {
                initLocation = false;
              }
          } else {
          Serial.println("Sensitivity level field does not exist.");
          }
        } else {
          Serial.println("Sensitivity level field does not exist.");
        }

        //To check the initlaization of the fields in Cloud Firestore
        // Serial.print(initUsername ? "true" : "false");
        // Serial.print(" ");
        // Serial.print(initSensitivityLevel ? "true" : "false");
        // Serial.print(" ");
        // Serial.println(initLocation ? "true" : "false");

        // Initialize Firestore fields accordingly
        initializeFirestoreFields(initUsername, initSensitivityLevel, initLocation, FIREBASE_PROJECT_ID, userId);

      } else {
        // Check for the sensitivity level field
        if (readjson.get(readjsonData, "fields/sensitivity/doubleValue", true)) {
          sensitiveLevel = readjsonData.doubleValue;
        } else {
          Serial.println("Sensitivity level field does not exist.");
        }
      }

      //Clear the Json and the Json data after reading sensitive and username value
      readjson.clear();
      readjsonData.clear();
      
    } else {
      Serial.println("Failed to get document. HTTP error code: " + fbdo.errorReason());
      initializeFirestoreFields(true, true, true, FIREBASE_PROJECT_ID, userId);
    }
  } else {
    Serial.print("Failed to get document");
    initializeFirestoreFields(true, true, true, FIREBASE_PROJECT_ID, userId);
  }
}

void initializeFirestoreFields(bool initUsername, bool initSensitivityLevel, bool initLocation, const char* FIREBASE_PROJECT_ID, const char* userId) {

  // Create a FirebaseJson object for storing data
  FirebaseJson content;

  // Set default values
  float defaultAccelValue = 0.0;
  float defaultLongitude = 0.0;
  float defaultLatitude = 0.0;
  bool defaultFallStatus = false;
  bool defaultEmergency = false;
  float defaultSensitive = 0.65;
  bool defaultGPSStatus = false;

  // Use epoch time 0 as the default timestamp
  time_t defaultTime = 0;
  FirebaseJson timeJson;
  timeJson.set("seconds", defaultTime);
  timeJson.set("nanos", 0);

  // Update Firestore fields with default values
  char documentPath[100];
  snprintf(documentPath, sizeof(documentPath), "users/%s", userId);

  content.set("fields/accelerometer/doubleValue", String(defaultAccelValue, 2));
  content.set("fields/time/timestampValue", timeJson);
  content.set("fields/fall_status/booleanValue", String(defaultFallStatus).c_str());
  content.set("fields/emergency/booleanValue", String(defaultEmergency).c_str());
  content.set("fields/gps_status/booleanValue", String(defaultGPSStatus).c_str());

  // Only set the username field if it's empty
  String fieldMask = "accelerometer, time, fall_status, emergency, gps_status";
  if (initUsername) {
    content.set("fields/username/stringValue", "");
    fieldMask += ", username";
  }

  // Only set the sensitivity level field if it's empty
  if (initSensitivityLevel) {
    content.set("fields/sensitivity/doubleValue", String(defaultSensitive, 2));
    fieldMask += ", sensitivity";
  }

  // Only set the longitude and latitude field if it's empty
  if (initLocation) {
    content.set("fields/longitude/doubleValue", String(defaultLongitude, 6));
    content.set("fields/latitude/doubleValue", String(defaultLatitude, 6));
    fieldMask += ", longitude, latitude";
  }

  // Set the document with default values
  if (Firebase.Firestore.patchDocument(&fbdo, FIREBASE_PROJECT_ID, "", documentPath, content.raw(),fieldMask.c_str())) {
      Serial.println("Firestore fields initialized with default values");
  } else {
    Serial.println("Failed to initialize Firestore fields: " + fbdo.errorReason());
  }

  content.clear();
}

void updateFirestore(bool isFallUpdate, bool fallStatus, bool emergency, float accleration, float longitude, float latitude, time_t currentTime, String stringTime, bool gpsStatus, const char* FIREBASE_PROJECT_ID, const char* userId) {
  // Create a FirebaseJson object for storing data
  FirebaseJson content;

  FirebaseJson timeJson;
  timeJson.set("seconds", currentTime);
  timeJson.set("nanos", 0);

  // Determine the document path and fields based on update type
  char documentPath[150];
  String updateFields;
  if (isFallUpdate) {
    snprintf(documentPath, sizeof(documentPath), "%s/%s", userId, stringTime);
    content.set("fields/time/timestampValue", timeJson);
    updateFields = "time";
  } else {
    snprintf(documentPath, sizeof(documentPath), "users/%s", userId);
    content.set("fields/accelerometer/doubleValue", String(accleration, 2));
    content.set("fields/time/timestampValue", timeJson);
    content.set("fields/fall_status/booleanValue", String(fallStatus).c_str());
    content.set("fields/emergency/booleanValue", String(emergency).c_str());
    content.set("fields/gps_status/booleanValue", String(gpsStatus).c_str());
    updateFields = "accelerometer,time,fall_status,emergency,gps_status";
  }

  if (longitude != 0.0 && latitude != 0.0) {
    content.set("fields/longitude/doubleValue", String(longitude, 6));
    content.set("fields/latitude/doubleValue", String(latitude, 6));
    updateFields += ", longitude, latitude";
  }

  // Set the document with sensor data updates
  //const char* updateFields = isFallUpdate ? "time, longitude, latitude" : "accelerometer, time, fall_status, longitude, latitude, emergency, gps_status";  //testing string concatonation
  if (Firebase.Firestore.patchDocument(&fbdo, FIREBASE_PROJECT_ID, "", documentPath, content.raw(), updateFields)) {
  } else {
    Serial.println("FAILED: " + fbdo.errorReason());
  }

  content.clear();
}

