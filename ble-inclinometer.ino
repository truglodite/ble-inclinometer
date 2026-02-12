// Truglodite 2/10/26
// ble-inclinometer.ino
// The perfect tool for setting rc plane control throw, and many other angle measurement needs.
// This code is writed for the "Xiao NRF52840 Sense" board (use the "mbed" board definition).
// It's intended for use with the phone app "NRF Connect Mobile" (iOS and Android).

// Usage:
// Flash the board, power it on, and connect it to the app.
// Subscribe to the roll (1001) and pitch (1002) sensors ("down/line arrow" buttons).
// Configure the data as "UTF-8" ("quote" buttons).
// To zero both axis, send a true boolean (or 1) to the tare service (1003) (up arrow on sensor w/ long uuid).
// If using a battery for power via the battery pads, subscribe to the battery service (1004) to read battery volts.

// Arduino requires the seed gyro lib (not the 'duino download): https://github.com/Seeed-Studio/Seeed_Arduino_LSM6DS3

#include "ArduinoBLE.h"
#include "LSM6DS3.h"
#include "Wire.h"
#include <nrf52840.h>
#include <nrfx_saadc.h>
#include <AnalogIn.h>
#include <pinDefinitions.h>
#include <avr/dtostrf.h>

#define sampleCount 100 // # of samples between readings
#define longFlash 2000   // msec to flash when tare is received
#define shortFlash 50   // msec to flash when data is sent
#define chargeCurrent LOW // Built in battery charger: HIGH = 50mA, LOW = 100mA
#define batteryMaxV 4200 // voltage*1000 at 100% charge
#define batteryMinV 3200 // voltage*1000 at 0% charge

#define chargePin P0_13
#define batteryReadPin P0_14
#define batteryAnalogPin P0_31
LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A

// Characteristic UUID's
#define BLE_UUID_ANGLE_MONITOR_SERVICE "8acafa20-26e9-4d16-a792-cf7de147c01c"  // v4 random uuid
#define BLE_UUID_ROLL_DEGREES  "1001"
#define BLE_UUID_PITCH_DEGREES  "1002"
#define BLE_UUID_TARE_SWITCH  "1003"
#define BLE_UUID_BATTERY_VOLTS  "1004"
//#define BLE_UUID_BATTERY_VOLTS  "5726c19a-8a75-5d7a-845d-aadf6734d7e7"  // V5 uuid's
//#define BLE_UUID_ROLL_DEGREES  "a68e1ad6-8c88-56f4-b9d5-792af19cfb19"
//#define BLE_UUID_PITCH_DEGREES  "d9bc177b-1fbe-5724-867a-558e397f2401"
//#define BLE_UUID_TARE_SWITCH  "f7a73029-a679-5de1-8448-ba3d23450f75"

// Bluetooth® Low Energy Battery Service
BLEService angleMonitorService(BLE_UUID_ANGLE_MONITOR_SERVICE);

// Bluetooth® Low Energy Characteristics & Descriptors
BLEStringCharacteristic batteryVolts(BLE_UUID_BATTERY_VOLTS, BLERead | BLENotify, 20);
BLEStringCharacteristic rollDegrees(BLE_UUID_ROLL_DEGREES, BLERead | BLENotify, 20);
BLEStringCharacteristic pitchDegrees(BLE_UUID_PITCH_DEGREES, BLERead | BLENotify, 20);
BLEByteCharacteristic tareChar(BLE_UUID_TARE_SWITCH, BLERead | BLEWrite);

// BLE Descriptors
BLEDescriptor pitchDegreesDescriptor("2901", "Pitch Degrees");
BLEDescriptor rollDegreesDescriptor("2901", "Roll Degrees");
BLEDescriptor batteryVoltsDescriptor("2901", "Batt Volts");
BLEDescriptor tareCharDescriptor("2901", "Tare");

float battery = 0.0;  // battery voltage
char batteryBuffer[20];
float roll = 0;  // roll angle
char rollBuffer[20];
float pitch = 0;  // pitch angle
char pitchBuffer[20];
float rollRaw = 0.0;  // raw calculated roll
float pitchRaw = 0.0;  // raw calculated pitch
float tareRoll = 0.0;  // raw roll value when tared
float tarePitch = 0.0;  // raw pitch value when tared
long previousData = 0;  // msec since data was sent
long previousTare = 0;  // msec timer for data led flash
bool dataLedFlag = 0;  // flag for data led flash
bool tareFlag = 0;  // flag for tare routine
bool tareLedFlag = 0; // flag for tare led timer
u_int8_t samples = 0;  // sample count storage

void setup()
{
  Serial.begin(9600);    // initialize serial communication

  pinMode(LED_RED, OUTPUT); // initialize the built-in LED pin to indicate when a central is connected
  pinMode (chargePin, OUTPUT);  // init charge current setting pin
  pinMode (batteryReadPin, OUTPUT);  // init charge current setting pin

  // begin initialization
  if (!BLE.begin()) 
  {
    Serial.println("starting BLE failed!");

    while (1);
  }

  if (myIMU.begin() != 0) {
      Serial.println("IMU error");
  } else {
      Serial.println("IMU OK!");
  }

  // Set advertised local name and service
  BLE.setDeviceName( "Angle Monitor" );
  BLE.setLocalName( "Angle Monitor" );
  BLE.setAdvertisedService( angleMonitorService );

  // Add Characteristic descriptors
  batteryVolts.addDescriptor(batteryVoltsDescriptor);
  rollDegrees.addDescriptor(rollDegreesDescriptor);
  pitchDegrees.addDescriptor(pitchDegreesDescriptor);
  tareChar.addDescriptor(tareCharDescriptor);

  // Add BLE characteristics
  angleMonitorService.addCharacteristic( batteryVolts );
  angleMonitorService.addCharacteristic( rollDegrees );
  angleMonitorService.addCharacteristic( pitchDegrees );
  angleMonitorService.addCharacteristic( tareChar );

  // Add Service
  BLE.addService( angleMonitorService );

  // Write initial values
  dtostrf(battery, 4, 2, batteryBuffer);
  batteryVolts.writeValue(batteryBuffer);
  dtostrf(roll, 5, 2, rollBuffer);
  rollDegrees.writeValue(rollBuffer);
  dtostrf(pitch, 5, 2, pitchBuffer);
  pitchDegrees.writeValue(pitchBuffer);
  tareChar.writeValue(0);

  // start advertising
  BLE.advertise();

  // Configure IMU
  myIMU.settings.gyroEnabled = 0;  //Can be 0 or 1
  myIMU.settings.accelEnabled = 1;
  myIMU.settings.accelRange = 2;      //Max G force readable.  Can be: 2, 4, 8, 16
  myIMU.settings.accelSampleRate = 104;  //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664, 13330
  myIMU.settings.accelBandWidth = 50;  //Hz.  Can be: 50, 100, 200, 400;

  Serial.println("Bluetooth® device active, waiting for connections...");
}

void loop()
{
  digitalWrite(chargePin, chargeCurrent); // configure usb charger
  digitalWrite(batteryReadPin, LOW);  // configure battery measurement
  
  digitalWrite(LED_RED, LOW);  // Turn on red led while not connected
  digitalWrite(LED_BLUE, HIGH);  // Turn off blue led while not connected
  // wait for a Bluetooth® Low Energy central
  BLEDevice central = BLE.central();

  // if a central is connected to the peripheral:
  if (central)  {
    Serial.print("Connected to central: ");
    // print the central's BT address:
    Serial.println(central.address());
    
    digitalWrite(LED_RED, HIGH); // turn off the red LED once connected
    digitalWrite(LED_BLUE, LOW);  // Turn on the blue led while connected

    // check the battery level every 200ms
    // while the central is connected:
    while (central.connected())  {
      long currentMillis = millis();

      // count more samples
      if (samples < sampleCount)  {
        readData();
        samples++;
      }
      // enough samples, send data
      else  {
        previousData = currentMillis;
        sendData();
        digitalWrite(LED_RED, LOW); // turn on data led flash
        dataLedFlag = 1;
      }
      // data led is on, and time to turn it off
      if (dataLedFlag && currentMillis - previousData >= shortFlash)  {
        if(!tareLedFlag) { // turn led off only if not taring
          digitalWrite(LED_RED, HIGH);
        }
        dataLedFlag = 0;
      }

      // Tare recieved, turn on LED and set tare flag
      if (tareChar.written()) {
        if (tareChar.value()) {    // received a HIGH value
          previousTare = currentMillis; //start led timer
          digitalWrite(LED_RED, LOW); // turn on tare led flash
          tareFlag = 1;
          tareLedFlag = 1;
          Serial.println("Tare Axis");
        }
      }
      // We have a tare flag, and enough samples to tare
      if (tareFlag && samples >= sampleCount) {
        tareAxis();
        tareFlag = 0;
      }
      // tare led is on, and time to turn it off
      if (tareLedFlag && currentMillis - previousTare >= longFlash)  {
        digitalWrite(LED_RED, HIGH);
        tareLedFlag = 0;
      }

    }
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}


void readData()  {
  int batteryADC = analogRead(batteryAnalogPin); // read battery adc
  battery += (float(batteryADC) * 3.3) / 1024 * 1510.0 / 510.0; // calc actual battery volts w/ 3v3 reg and 10bit adc

  //Read angle between X and Z accelerometer axis
  float accX = myIMU.readFloatAccelX();
  float accY = myIMU.readFloatAccelY();
  float accZ = myIMU.readFloatAccelZ();
  rollRaw += atan2(accY, accZ) * 57.2958;
  pitchRaw += atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 57.2958;
}

void sendData() {
  // Average angles, stringify, print, and send
  roll = ( rollRaw / samples ) - tareRoll;
  pitch = ( pitchRaw / samples ) - tarePitch;
  dtostrf(roll, 5, 2, rollBuffer);
  dtostrf(pitch, 5, 2, pitchBuffer);
  Serial.print("Roll: ");
    Serial.println(rollBuffer);
  rollDegrees.writeValue(rollBuffer);
  Serial.print("Pitch: ");
    Serial.println(pitchBuffer);
  pitchDegrees.writeValue(pitchBuffer);

  battery = battery / samples;
  dtostrf(battery, 4, 2, batteryBuffer);
  Serial.print("Battery Volts: "); // print actual battery volts
  Serial.println(batteryBuffer);
  batteryVolts.writeValue(batteryBuffer);  // send the battery volts

  // rezero values
  samples = 0;
  rollRaw = 0;
  pitchRaw = 0;
  battery = 0;
}

void tareAxis() {
  tareRoll = rollRaw / samples;
  tarePitch = pitchRaw / samples;
}
