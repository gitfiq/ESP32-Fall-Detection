#ifndef FIRESTORE_UTILS_H
#define FIRESTORE_UTILS_H

#include <Firebase_ESP_Client.h>

extern double sensitiveLevel; // Declare the sensitiveLevel variable

void setFirebase(const char* API_KEY, const char* USER_EMAIL, const char* USER_PASSWORD);  // Set up the Firebase
void readFirestore(const char* FIREBASE_PROJECT_ID, const char* userId, bool initalSatus);  // Reads the initial values of the fields in Cloud Firestore to decide to initialize
void initializeFirestoreFields(bool initUsername, bool initSensitivityLevel, bool initLocation, const char* FIREBASE_PROJECT_ID, const char* userId);  // Initialize the Firestore Fields
void updateFirestore(bool isFallUpdate, bool fallStatus, bool emergency, float accleration, float longitude, float latitude, time_t currentTime, String stringTime, bool gpsStatus, const char* FIREBASE_PROJECT_ID, const char* userId);  // Function to update Cloud Firestore with sensor data

#endif
