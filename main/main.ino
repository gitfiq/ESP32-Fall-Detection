#include <Wire.h>
#include <NTPClient.h>
#include <TinyGPS++.h>

#include "WiFiUtils.h"
#include "FirestoreUtils.h"
#include "BluetoothHandler.h"

// Define Firebase API Key, Project ID, and user credentials
#define API_KEY "AIzaSyDS5VzBPaOL327YkiJSE2ZFcdPstCOBnSg"
#define FIREBASE_PROJECT_ID "falldetection-f3724"
#define USER_EMAIL "123456abcdef@gmail.com"
#define USER_PASSWORD "fedcba654321"

// The default baudrate of NEO-6M is 9600
#define GPS_BAUDRATE 9600  

// the TinyGPS++ object
TinyGPSPlus gps;  

const char* userId = "userabc";  // Define the unique id for the user

// Initialize the pins used for button and buzzer
const int buttonPin = 18;
const int buzzerPin = 19;
const int ledPin = 2;

// Declare the cloud Firestore field values
float longitude = 0.0;  // Global variable that stores the field longitude in Cloud Firestore
float latitude = 0.0;  // Global variable that stores the field latitude in Cloud Firestore
float acceleration = 1.0;  // Global variable that stores the field acceleration in Cloud Firestore
bool fallStatus = false;  // Global variable that stores the field fallStatus in Cloud Firestore
bool emergency = false;  // Global variable that stores the field emergency in Cloud Firestore
bool gpsStatus = false;  // Global variable to keep track of wheter the NEO-6M GPS is working 

// Declare flags to detect falls
bool fallflag = false;  // Variable to store the flag that checks if the user has pressed the button to show a false detected before sending the data to the Cloud Firestore
bool fallIssue = false;  // Variable to store the flag that checks if a fall is detected 
float dataPoints[5] = {1.0, 1.0, 1.0, 1.0, 1.0};  // Variable to store the acceleration values and will be use to check for fall detection

// Declare variables to track the button usage
unsigned long monitoringStartTime = 0;  // Variable to store the current time when data are stored in Cloud Firestore and is used to check the time interval for 'sendingInterval'
unsigned long sendingInterval = 2000;  // Variable to store the interval to store the values into Cloud Firestore
volatile unsigned long pressStartTime = 0;  // Variable to store the time when button is pressed
volatile int buttonState = 0; // Variable to store the button state (1 or 0)
volatile bool buttonPressedFlag  = false;  // Variable to check if the button was pressed
bool buttonStopFallDetect = false;

// NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "sg.pool.ntp.org");

// Initialize the variables for getting the device's acceleration
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
uint32_t LoopTimer;

// Kalman Filter (Used to decrease the noise in the calcuulating the rate of change in the device's orientation)
float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
float Kalman1DOutput[]={0,0};

//Roll and Pitch Threshold
float previousRollAngle = 0.0; // Variable to store the previous roll angle
float rollThreshold = 14.5; // Threshold for the roll angle change
float accumalatedRateOfChangeRoll = 0.0;
float previousPitchAngle = 0.0; // Variable to store the previous pitch angle
float pitchThreshold = 8.5; // Threshold for the pitch angle change
float accumalatedRateOfChangePitch = 0.0;
unsigned long monitoringRateOfChangeTime = 0;  // Variable to store the current time when rate of change resets
unsigned long resettingRateOfChangeInterval = 300;  // Variable to store the interval to store the values into Cloud Firestore
bool resetFlag = true;   // To check if the the device is ready to reset the accumalatedRateOfChangeRoll and accumalatedRateOfChangePitch


// A filter to reduce noise of the MPU 6050 sensor
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState=KalmanState+0.004*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
}

// Function to get the total accelration and the rate of change in the orientation of the device
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
  AccX=(float)AccXLSB/4096-0.02;
  AccY=(float)AccYLSB/4096;
  AccZ=(float)AccZLSB/4096+0.02;
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);
  acceleration=sqrt(AccX*AccX+AccZ*AccZ+AccY*AccY);
}

// Update the device's longitude and latitude
void updateLocation() {
  if (gps.location.isUpdated()) {
    latitude = gps.location.lat();
    longitude = gps.location.lng();

    Serial.print("Latitude: ");
    Serial.println(latitude, 6);

    Serial.print("Longitude: ");
    Serial.println(longitude, 6);

    gpsStatus = true;
  } else {
    gpsStatus = false;
  }
}

// Interrupt service routine for the button press
void buttonPressed() {
  pressStartTime = millis();  // Record the time when the button is pressed
  detachInterrupt(digitalPinToInterrupt(buttonPin));  // Reset the interrupt
  attachInterrupt(digitalPinToInterrupt(buttonPin), buttonReleased, RISING);  // Sets the Interupt to when button is released
}

// Interrupt service routine for the button release (0 to 1)
void buttonReleased() {
  buttonPressedFlag = true;  // Set the flag indicating the button was released
  detachInterrupt(digitalPinToInterrupt(buttonPin));  // Reset the interrupt
  attachInterrupt(digitalPinToInterrupt(buttonPin), buttonPressed, FALLING);  // Sets the Interupt to when button is pressed
}

// Reset the Fall_Status and Emergency fields in Firestore and stop the buzzer
void clearFallStatusAndEmergency() {
  digitalWrite(buzzerPin,LOW); // Stop the buzzer
  fallStatus = false;
  emergency = false;
}

// Set the emergency state to true and sound the buzzer with a different sound
void setEmergency() {
  emergency = true;
  for(int i = 0; i < 3; i++) {
    digitalWrite(buzzerPin,HIGH);
    delay(500);
    digitalWrite(buzzerPin,LOW);
    delay(500);
  }
  digitalWrite(buzzerPin,HIGH);
}

void restartESPWifi() {
  setUpdateCredentialsFlag(true);
}

// Interrupt service routine for the button to stop the the fall set
void buttonStopFall() {
  buttonStopFallDetect = true;
  detachInterrupt(digitalPinToInterrupt(buttonPin));  // Reset the interrupt
  attachInterrupt(digitalPinToInterrupt(buttonPin), buttonPressed, FALLING);  // Attach an interrupt to the button pin
}

// Checks if the user press the button if a fall is detected
void checkUserInput(){
  buttonState = digitalRead(buttonPin);
  digitalWrite(buzzerPin,HIGH);
  fallflag = true;
  for(int i = 0; i < 15000; i++){
    if(buttonStopFallDetect == true){
      fallStatus = false;
      fallflag = false;
      digitalWrite(buzzerPin,LOW);
      buttonStopFallDetect = false;
      break;
    }else{
      delay(1);
    }
  }
}

// Set up the initials of the device
void setup(){
  Serial.begin(9600);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Begin the operation to get the acceleration
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
  LoopTimer = micros();

  // Set the pins
  pinMode(buttonPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(ledPin, OUTPUT);

  digitalWrite(buzzerPin, LOW);  // Set the buzzer to be LOW intially

  digitalWrite(ledPin, LOW);  // Set the LED to be LOW intially

  Serial2.begin(GPS_BAUDRATE, SERIAL_8N1, 16, 17);   // Using hardware serial 2 for GPS

  initBluetooth();  //Initialize Bluetooth

  readCredentialsFromPreferences(wifiSSID, wifiPassword);  // Get the wifi credential

  // Checks if the wifi credentials are present
  if (wifiSSID == "" || wifiPassword == "") {
    obtainWiFiCredentials(ledPin, API_KEY, USER_EMAIL, USER_PASSWORD);
  }

  Serial.println("SSID: " + wifiSSID);
  Serial.println("Password: " + wifiPassword);

  setupWifi(wifiSSID, wifiPassword, ledPin, API_KEY, USER_EMAIL, USER_PASSWORD);  // Set up and begin connection with Wifi

  timeClient.begin();  // Connection to the NTP Server and gets the time

  attachInterrupt(digitalPinToInterrupt(buttonPin), buttonPressed, FALLING);  // Attach an interrupt to the button pin

  readFirestore(FIREBASE_PROJECT_ID, userId, true);  // Read username and sesitive fields initialize Firestore fields

  readFirestore(FIREBASE_PROJECT_ID, userId, false);  // Read the initial sensitive value from Cloud Firestore
}

// Run the loop
void loop(){
  //Serial.println(accumalatedRateOfChangePitch);

  timeClient.update();  // Update the synchronisation with the NTP Server

  time_t currentTime = timeClient.getEpochTime();   // Get the timestamp in seconds since epoch

  String stringTime = timeClient.getFormattedTime();  // Convert the time to string to pass into Cloud Firestore

  // Update the acceleration value
  gyro_signals();
  RateRoll-=RateCalibrationRoll;
  RatePitch-=RateCalibrationPitch;
  RateYaw-=RateCalibrationYaw;
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll=Kalman1DOutput[0]; 
  KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch=Kalman1DOutput[0]; 
  KalmanUncertaintyAnglePitch=Kalman1DOutput[1];

  // Calculate the rate of change of the roll angle
  float rollRateOfChange = abs(KalmanAngleRoll - previousRollAngle);
  accumalatedRateOfChangeRoll += rollRateOfChange;
  previousRollAngle = KalmanAngleRoll; // Update the previous roll angle

  // Calculate the rate of change of the pitch angle
  float pitchRateOfChange = abs(KalmanAnglePitch - previousPitchAngle);
  accumalatedRateOfChangePitch += pitchRateOfChange;
  previousPitchAngle = KalmanAnglePitch; // Update the previous pitch angle

  // Execute the operation to update the device's longitude and latitude
  char c = Serial2.read();
  gps.encode(c);
  updateLocation();

  // Shift the elements with the new acceleration value
  dataPoints[0] = dataPoints[1];  // Shift element at index 1 to 0
  dataPoints[1] = dataPoints[2];  // Shift element at index 2 to 1
  dataPoints[2] = dataPoints[3];  // Shift element at index 3 to 2
  dataPoints[3] = dataPoints[4];  // Shift element at index 4 to 3
  dataPoints[4] = acceleration;   // Replace element at index 4 with the new acceleration value

  //Checks the impact acceleration experienced
  if (acceleration > 1.4) {
    //Checks if free fall/ dropping is experienced
    resetFlag = false;
    for (int i = 0; i < 5; i++) {
      if(dataPoints[i]<sensitiveLevel){
        if (accumalatedRateOfChangeRoll >= rollThreshold || accumalatedRateOfChangePitch >= pitchThreshold) {
          fallIssue = true;  // A fall is detected
          detachInterrupt(digitalPinToInterrupt(buttonPin));  // Reset the interrupt
          attachInterrupt(digitalPinToInterrupt(buttonPin), buttonStopFall, FALLING);  // Attach an interrupt to the button pin
        }
      }
    } 
    resetFlag = true;  //Reset the flag after checking the fall
  }

  // To check if the fall is false positive 
  if(fallIssue){
    fallIssue = false;
    fallStatus = true;
    checkUserInput();  // Checks if the user pressed the button(Indication that is is a false positive)
    if(fallflag == true){
      updateFirestore(true, fallStatus, emergency, acceleration, longitude, latitude, currentTime, stringTime, gpsStatus, FIREBASE_PROJECT_ID, userId);  // Real fall detected. Updates to Fall History 
      fallflag = false;
      }
    }

      //Checks if it has been 300 ms since last reset of the accumated rate of change
  if (millis() - monitoringRateOfChangeTime >= resettingRateOfChangeInterval && resetFlag) {
    accumalatedRateOfChangeRoll = 0;  //Reset the total change of orientation (roll)
    accumalatedRateOfChangePitch = 0;
    monitoringRateOfChangeTime = millis();
  }

    //Checks if it has been 2 second since last sent data to Cloud Firestore
  if (millis() - monitoringStartTime >= sendingInterval) {
    // Check if the values are valid (not NaN)
    if (!isnan(AccX) && !isnan(AccY) && !isnan(AccZ)) {
    //Update values to Firestore
    updateFirestore(false, fallStatus, emergency, acceleration, longitude, latitude, currentTime, stringTime, gpsStatus, FIREBASE_PROJECT_ID, userId);  //Updates to user status
    }

    readFirestore(FIREBASE_PROJECT_ID, userId, false);  // Update the sensitive value from Cloud Firestore
    monitoringStartTime = millis();
  }

  // Checks if the user intends to activate the emergency function    
  if (buttonPressedFlag) {
    buttonPressedFlag = false;  // Reset the flag
    unsigned long pressDuration = millis() - pressStartTime;
    if (pressDuration >= 15000) {
      restartESPWifi();
    } else if (pressDuration < 15000 && pressDuration >= 5000) {
      // Long press action (less than 15 seconds and more than 5 seconds)
      setEmergency();
    } else if (pressDuration < 5000 && pressDuration >= 2000) {
      // Short press action (less than 5 seconds and more than 2 seconds). To prevent user's from accidentally stopping resetting the fall status
      clearFallStatusAndEmergency();
    }
  }

  // Checks if user want to send another Wifi Credential
  if (updateCredentials) {
    initBluetooth();
    disconnectWifi();
    delay(500);
    obtainWiFiCredentials(ledPin, API_KEY, USER_EMAIL, USER_PASSWORD);
  }

  // Delay before the next reading
  delay(10);

}