// Truglodite 2/10/26
// ble-inclinometer.ino
// The perfect tool for setting rc plane control throw, and many other angle measurement needs.
// This code is writed for the "Xiao NRF52840 Sense" board (use the "mbed" board definition).
// It's intended for use with the phone app "NRF Connect Mobile" (iOS and Android).

// Usage:
// Flash the board, power it on, and connect it to the app.
// Subscribe to the roll (2c08) and pitch (2c09) sensors ("down/line arrow" buttons).
// Configure the data as "signed int" ("quote" buttons).
// To zero both axis, send a true boolean to the tare service  (up arrow on sensor w/ long uuid).
// If using a battery for power via the battery pads, subscribe to the battery service to read battery %.

// Arduino requires the seed gyro lib (not the 'duino download): https://github.com/Seeed-Studio/Seeed_Arduino_LSM6DS3

#include "ArduinoBLE.h"
#include "LSM6DS3.h"
#include "Wire.h"
#include <nrf52840.h>
#include <nrfx_saadc.h>
#include <AnalogIn.h>
#include <pinDefinitions.h>

#define updateDelay 1000  // msec between data updates
#define longFlash 2000   // msec to flash when tare is received
#define shortFlash 50   // msec to flash when data is sent
#define chargeCurrent LOW // Built in battery charger: HIGH = 50mA, LOW = 100mA
#define batteryMaxV 4200 // voltage*1000 at 100% charge
#define batteryMinV 3200 // voltage*1000 at 0% charge

#define chargePin P0_13
#define batteryReadPin P0_14
#define batteryAnalogPin P0_31
LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A
// Bluetooth® Low Energy Battery Service
BLEService batteryService("180F");
BLEService rollservice("185A");
BLEService pitchservice("185B");
BLEService tareService("19B10000-E8F2-537E-4F6C-D104768A1214");

// Bluetooth® Low Energy Battery Level Characteristic
BLEUnsignedCharCharacteristic batteryLevelChar("2A19", BLERead | BLENotify); // standard 16-bit characteristic UUID
BLEUnsignedCharCharacteristic rollchar("2C08", BLERead | BLENotify); // standard 16-bit characteristic UUID
BLEUnsignedCharCharacteristic pitchchar("2C09", BLERead | BLENotify); // standard 16-bit characteristic UUID
BLEByteCharacteristic tareChar("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
// remote clients will be able to get notifications if this characteristic changes

double oldBatteryV = 0.0;  // last battery level reading from analog input
int roll = 0.0;  // roll to be sent
int pitch = 0.0;  // pitch to be sent
int rollRaw = 0.0;  // raw calculated roll
int pitchRaw = 0.0;  // raw calculated pitch
int tareRoll = 0.0;  // raw roll value when tared
int tarePitch = 0.0;  // raw pitch value when tared
long previousData = 0;  // msec since data was sent
long previousFlash = 0;  // msec timer for data led flash
bool dataFlag = 0;  // flag for data led flash
bool tareFlag = 0;  // flag for tare led flash

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

  BLE.setLocalName("AngleMonitor");

// Configure Battery Monitor Service
  BLE.setAdvertisedService(batteryService); // add the service UUID
  batteryService.addCharacteristic(batteryLevelChar); // add the battery level characteristic
  BLE.addService(batteryService); // Add the battery service
  batteryLevelChar.writeValue(oldBatteryV); // set initial value for this characteristic
// Configure roll Monitor Service
  BLE.setAdvertisedService(rollservice); // add the service UUID
  rollservice.addCharacteristic(rollchar); // add the battery level characteristic
  BLE.addService(rollservice); // Add the battery service
  rollchar.writeValue(roll); // set initial value for this characteristic
// Configure pitch Monitor Service
  BLE.setAdvertisedService(pitchservice); // add the service UUID
  pitchservice.addCharacteristic(pitchchar); // add the battery level characteristic
  BLE.addService(pitchservice); // Add the battery service
  pitchchar.writeValue(pitch); // set initial value for this characteristic
// Configure Tare Service
  BLE.setAdvertisedService(tareService);
  tareService.addCharacteristic(tareChar);
  BLE.addService(tareService);
  tareChar.writeValue(0);

  // start advertising
  BLE.advertise();

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
      // enough time has passed, update the battery and angles:
      if (currentMillis - previousData >= updateDelay)  {
        previousData = currentMillis;
        updateBatteryLevel();
        updateAngles();
        digitalWrite(LED_RED, LOW); // turn on data led flash
        dataFlag = 1;
      }
      // data led is on, and time to turn it off
      if (dataFlag && currentMillis - previousData >= shortFlash)  {
        if(!tareFlag) { // turn led off only if not taring
          digitalWrite(LED_RED, HIGH);
        }
        dataFlag = 0;
      }
      if (tareChar.written()) {
        if (tareChar.value()) {    // received a HIGH value
          previousFlash = currentMillis;
          digitalWrite(LED_RED, LOW); // turn on tare led flash
          tareFlag = 1;
          Serial.println("Tare Axis");
          tareAxis();    
        }
      }
      // tare led is on, and time to turn it off
      if (tareFlag && currentMillis - previousFlash >= longFlash)  {
        digitalWrite(LED_RED, HIGH);
        tareFlag = 0;
      }

    }
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}


void updateBatteryLevel()
{
  int battery = analogRead(batteryAnalogPin); // read battery adc
  double batteryV = (battery * 3.3) / 1024 * 1510.0 / 510.0; // calc actual battery volts w/ 3v3 reg and 10bit adc
  battery = batteryV * 1000; // use 4 digit precision for int mapping to %
  battery = map(battery, batteryMinV, batteryMaxV, 0, 100); // map volts to %
  if (batteryV != oldBatteryV)    // if the battery level has changed
    { 
      Serial.print("Battery Volts: "); // print actual battery volts
      Serial.println(batteryV);
      batteryLevelChar.writeValue(battery);  // send the battery %
      oldBatteryV = batteryV;           // save the volts for next comparison
    } 
}

void updateAngles()
{
  //Read angle between X and Z accelerometer axis
  float accX = myIMU.readFloatAccelX();
  float accY = myIMU.readFloatAccelY();
  float accZ = myIMU.readFloatAccelZ();
  rollRaw = atan2(accY, accZ) * 57.2958;
  pitchRaw = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 57.2958;
  roll = rollRaw - tareRoll;
  pitch = pitchRaw - tarePitch;
  Serial.print("Roll: "); // print roll angle
    Serial.println(roll);
  rollchar.writeValue(roll);  // send the roll value
  Serial.print("Pitch: "); // print pitch angle
    Serial.println(pitch);
  pitchchar.writeValue(pitch);  // send the pitch value
}

void tareAxis()
{
  tareRoll = rollRaw;
  tarePitch = pitchRaw;
}
