#include <Wire.h>
#if defined(ESP32)
#include <WiFi.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#endif
#include <Firebase_ESP_Client.h>
#include <addons/TokenHelper.h>
#include <NTPClient.h>
#include <Time.h>

// Define the Wifi credentials
#define WIFI_SSID "iPhone(madfiq)"
#define WIFI_PASSWORD "afiq12345"

// Define Firebase API Key, Project ID, and user credentials
#define API_KEY "AIzaSyDS5VzBPaOL327YkiJSE2ZFcdPstCOBnSg"
#define FIREBASE_PROJECT_ID "falldetection-f3724"
#define USER_EMAIL "ahmadafiqirfanz75@gmail.com"
#define USER_PASSWORD "Madfiq59"

// Define the unique id for the user
const char* userId = "userabcd";
String location = "Home";
String username = "";
double sensitiveLevel = 0.6;
bool fallStatus = false;
int ID,fallflag = 0;

volatile int buttonState = 0;

const int buttonPin = 18;
const int buzzerPin = 19;

// Define Firebase Data object, Firebase authentication, and configuration
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
FirebaseData timestampData;

// NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "sg.pool.ntp.org");

float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
float AccX, AccY, AccZ;
float value;
float AngleRoll, AnglePitch;
uint32_t LoopTimer;
float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
float Kalman1DOutput[]={0,0};

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState=KalmanState+0.004*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
}

void gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();     
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  RateRoll=(float)GyroX/65.5;
  RatePitch=(float)GyroY/65.5;
  RateYaw=(float)GyroZ/65.5;
  AccX=(float)AccXLSB/4096;
  AccY=(float)AccYLSB/4096-0.02;
  AccZ=(float)AccZLSB/4096-0.02;
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);
  value=sqrt(AccX*AccX+AccZ*AccZ+AccY*AccY);
}

// Checks if the user press the button if a fall is detected
void checkUserInput(){
  buttonState = digitalRead(buttonPin);
  digitalWrite(buzzerPin,HIGH);
  fallflag = 1;
  ID += 1;
  for(int i = 0; i < 3000; i++){
    if(buttonState == 0){
      fallStatus = false;
      ID -= 1;
      fallflag = 0;
      digitalWrite(buzzerPin,LOW);
      break;
    }else{
      delay(1);
    }
  }
}

// Set up the Wifi
void setupWifi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }

  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
}

// Set up the Firebase
void setFirebase() {
  // Print Firebase client version
  Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);

  // Assign the API key
  config.api_key = API_KEY;

  // Assign the user sign-in credentials
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;

  // Assign the callback function for the long-running token generation task
  config.token_status_callback = tokenStatusCallback;  // see addons/TokenHelper.h

  // Begin Firebase with configuration and authentication
  Firebase.begin(&config, &auth);

  // Reconnect to Wi-Fi if necessary
  Firebase.reconnectWiFi(true);
}

// Reads the initial values of the fields to decide to initialize
void readUsernameAndInitializeFields() {
  Serial.println("Starting to read username and initialize Firestore fields...");

  String documentPath = "users/" + String(userId);
  if (Firebase.Firestore.getDocument(&fbdo, FIREBASE_PROJECT_ID, "", documentPath.c_str())) {
    if (fbdo.httpCode() == FIREBASE_ERROR_HTTP_CODE_OK) {
      // Print the entire JSON response for debugging
      Serial.println("Document retrieved successfully. JSON data:");
      Serial.println(fbdo.payload());

      // Create a FirebaseJson object and set content with received payload
      FirebaseJson readjson;
      readjson.setJsonData(fbdo.payload().c_str());
      FirebaseJsonData readjsonData;

      // Check for the username field
      bool initUsername = true;
      if (readjson.get(readjsonData, "fields/username/stringValue", true)) {
        String username = readjsonData.stringValue;
        Serial.println("Username retrieved: " + username);
        if (username.length() != 0) {
          initUsername = false;
        }
      } else {
        Serial.println("Username field does not exist.");
      }


      // Check for the sensitivity level field
      bool initSensitivityLevel = true;
      if (readjson.get(readjsonData, "fields/sensitivity/doubleValue", true)) {
        Serial.println("Sensitivity level retrieved: " + String(readjsonData.doubleValue));
        initSensitivityLevel = false;
      } else {
        Serial.println("Sensitivity level field does not exist.");
      }

      // Initialize Firestore fields accordingly
      initializeFirestoreFields(initUsername, initSensitivityLevel);

      //Clear the Json and the Json data after reading sensitive and username value
      readjson.clear();
      readjsonData.clear();
      
    } else {
      Serial.print("Failed to get document. HTTP error code: ");
      Serial.println(fbdo.httpCode());
      Serial.println("Reason: " + fbdo.errorReason());
      initializeFirestoreFields(true, true);
    }
  } else {
    Serial.print("Failed to get document: ");
    Serial.println(fbdo.errorReason());
    initializeFirestoreFields(true, true);
  }
}

// Initialize the Firestore Fields
void initializeFirestoreFields(bool initUsername, bool initSensitivityLevel) {
  Serial.println("Initializing Firestore fields with default values");

  // Create a FirebaseJson object for storing data
  FirebaseJson content;

  // Set default values
  float defaultAccelValue = 0.0;
  bool defaultFallStatus = false;
  String defaultLocation = "unknown";

  // Use epoch time 0 as the default timestamp
  time_t defaultTime = 0;
  FirebaseJson timeJson;
  timeJson.set("seconds", defaultTime);
  timeJson.set("nanos", 0);

  // Update Firestore fields with default values
  String documentPath = "users/" + String(userId);
  content.set("fields/accelerometer/doubleValue", String(defaultAccelValue, 2));
  content.set("fields/time/timestampValue", timeJson);
  content.set("fields/fall_status/booleanValue", String(defaultFallStatus).c_str());
  content.set("fields/location/stringValue", defaultLocation);

  // Only set the username field if it's empty
  String fieldMask = "accelerometer, time, fall_status, location";
  if (initUsername) {
    content.set("fields/username/stringValue", "");
    fieldMask += ", username";
  }

  // Only set the sensitivity level field if it's empty
  if (initSensitivityLevel) {
    content.set("fields/sensitivity/doubleValue", String(sensitiveLevel, 2));
    fieldMask += ", sensitivity";
  }

  // Set the document with default values
  if (Firebase.Firestore.patchDocument(&fbdo, FIREBASE_PROJECT_ID, "", documentPath.c_str(), content.raw(),fieldMask.c_str())) {
      Serial.println("Firestore fields initialized with default values");
  } else {
    Serial.println("Failed to initialize Firestore fields");
    Serial.println("REASON: " + fbdo.errorReason());
  }

  Serial.println(fieldMask);
  // Clear the content
  content.clear();
}

// Updating the Firestore
void updateFirestore(bool fallStatus, float accelValue, String location, time_t currentTime) {
  // Create a FirebaseJson object for storing data
  FirebaseJson content;

  FirebaseJson timeJson;
  timeJson.set("seconds", currentTime);
  timeJson.set("nanos", 0);

  // Update sensor data & fall indication
  String documentPath = "users/" + String(userId);
  content.set("fields/accelerometer/doubleValue", String(accelValue, 2));
  content.set("fields/time/timestampValue", timeJson);
  content.set("fields/fall_status/booleanValue", String(fallStatus).c_str());
  content.set("fields/location/stringValue", String(location).c_str());

  // Set the document with sensor data updates
  if (Firebase.Firestore.patchDocument(&fbdo, FIREBASE_PROJECT_ID, "", documentPath.c_str(), content.raw(), "accelerometer, time, fall_status, location")) {
      Serial.println("PASSED");
      Serial.println("JSON DATA: ");
      Serial.println("------------------------------------");
      Serial.println();
      Serial.printf("ok\n%s\n\n", fbdo.payload().c_str());
  } else {
    Serial.println("FAILED");
    Serial.println("REASON: " + fbdo.errorReason());
    Serial.println("------------------------------------");
    Serial.println();
  }

  // Content clear 
  content.clear();
}

// Send Fall Data
void updateFallFirestore(String location, time_t currentTime) {

  // Create a FirebaseJson object for storing data
  FirebaseJson fallcontent;

  FirebaseJson timeJson;
  timeJson.set("seconds", currentTime);
  timeJson.set("nanos", 0);

  // // Update fall history if a fall is detected
  String documentPath = String(userId) + "/" + String(ID);
  fallcontent.set("fields/time/timestampValue", timeJson);
  fallcontent.set("fields/location/stringValue", String(location).c_str());

  // Set the document with sensor data updates
  if (Firebase.Firestore.patchDocument(&fbdo, FIREBASE_PROJECT_ID, "", documentPath.c_str(), fallcontent.raw(), "time, location")) {
      Serial.println("PASSED");
      Serial.println("JSON DATA: ");
      Serial.println("------------------------------------");
      Serial.println();
      Serial.printf("ok\n%s\n\n", fbdo.payload().c_str());
  } else {
    Serial.println("FAILED");
    Serial.println("REASON: " + fbdo.errorReason());
    Serial.println("------------------------------------");
    Serial.println();
  }

  // fallcontent clear 
  fallcontent.clear();
}

// Read the sensitive levels that are set by the user. This will change the threshold value to detect falls
void readSensitiveValue() {
  Serial.println("Reading sesnsitive value");

  String documentPath = "users/" + String(userId);
  if (Firebase.Firestore.getDocument(&fbdo, FIREBASE_PROJECT_ID, "", documentPath.c_str())) {
    if (fbdo.httpCode() == FIREBASE_ERROR_HTTP_CODE_OK) {
      // Print the entire JSON response for debugging
      Serial.println("Document retrieved successfully. JSON data:");
      Serial.println(fbdo.payload());

      // Create a FirebaseJson object and set content with received payload
      FirebaseJson sensitivejson;
      sensitivejson.setJsonData(fbdo.payload().c_str());
      FirebaseJsonData sensitivejsonData;


      // Check for the sensitivity level field
      if (sensitivejson.get(sensitivejsonData, "fields/sensitivity/doubleValue", true)) {
        Serial.println("Sensitivity level retrieved: " + String(sensitivejsonData.doubleValue));
        //Update the sensitive level
        sensitiveLevel = sensitivejsonData.doubleValue;
      } else {
        Serial.println("Sensitivity level field does not exist.");
      }

      //Clear the Json and the Json data after reading sensitive value
      sensitivejson.clear();
      sensitivejsonData.clear();
      
    } else {
      Serial.print("Failed to get document. HTTP error code: ");
      Serial.println(fbdo.httpCode());
      Serial.println("Reason: " + fbdo.errorReason());
    }
  } else {
    Serial.print("Failed to get document: ");
    Serial.println(fbdo.errorReason());
  }
}

// Set up the initials of the device
void setup() {
  Serial.begin(57600);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  for (RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber ++) {
    gyro_signals();
    RateCalibrationRoll+=RateRoll;
    RateCalibrationPitch+=RatePitch;
    RateCalibrationYaw+=RateYaw;
    delay(1);
  }

  RateCalibrationRoll/=2000;
  RateCalibrationPitch/=2000;
  RateCalibrationYaw/=2000;
  LoopTimer=micros();

  pinMode(buttonPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
  //Set the buzzer to be low intially
  digitalWrite(buzzerPin,LOW);

  setupWifi();
  setFirebase();

 // Connection to the NTP Server and gets the time
  timeClient.begin();
  //timeClient.setTimeOffset(28800);

  // Attach an interrupt to the button pin
  attachInterrupt(digitalPinToInterrupt(buttonPin), buttonPressed, CHANGE);

  // Read username and initialize Firestore fields
  readUsernameAndInitializeFields();
}


void loop() {
  // Update the synchronisation with the NTP Server
  timeClient.update();
  // Get the timestamp in seconds since epoch
  time_t currentTime = timeClient.getEpochTime();

  readSensitiveValue();

  gyro_signals();
  RateRoll-=RateCalibrationRoll;
  RatePitch-=RateCalibrationPitch;
  RateYaw-=RateCalibrationYaw;
  // kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  // KalmanAngleRoll=Kalman1DOutput[0]; 
  // KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
  // kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  // KalmanAnglePitch=Kalman1DOutput[0]; 
  // KalmanUncertaintyAnglePitch=Kalman1DOutput[1];
  // Serial.print("Acceleration X [g]= ");
  // Serial.print(AccX);
  // Serial.print(" Acceleration Y [g]= ");
  // Serial.print(AccY);
  // Serial.print(" Acceleration Z [g]= ");
  // Serial.print(AccZ);
  
  Serial.print(" Value = ");
  Serial.print(value);
  Serial.print(" Status = ");

  // Read accelerationX and accelerationY from the DHT sensor
  float accelerationX = AccX;
  float accelerationY = AccY;
  float accelerationZ = AccZ;

  Serial.println(sensitiveLevel);

  while(value<0.6){
    time_t currentTime = timeClient.getEpochTime();
    gyro_signals();
    RateRoll-=RateCalibrationRoll;
    RatePitch-=RateCalibrationPitch;
    RateYaw-=RateCalibrationYaw;
    if (!isnan(accelerationX) && !isnan(accelerationY) && !isnan(accelerationZ)) {
      //Update values to Firestore
      updateFirestore(fallStatus, value, location, currentTime);
    }
    if(value>2.58){
      fallStatus = true;
      checkUserInput();
      if(fallflag==1){
        Serial.println(" Falling ");
        updateFallFirestore(location, currentTime);
        fallflag=0;
        }
      }
      delay(70);
    }
  
  // Check if the values are valid (not NaN)
  if (!isnan(accelerationX) && !isnan(accelerationY) && !isnan(accelerationZ)) {
    //Update values to Firestore
    updateFirestore(fallStatus, value, location, currentTime);
    }
      
  // Delay before the next reading
  delay(70);
}

// Interrupt service routine for the button
void buttonPressed() {
  // Read the state of the button
  buttonState = digitalRead(buttonPin);

  // Check if the button is pressed
  if (buttonState == 0) {
    digitalWrite(buzzerPin,LOW);
    fallStatus = false;
  }
}