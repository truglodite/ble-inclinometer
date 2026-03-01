#include <Arduino.h>

// ble-inclinometer
// written by: Truglodite 2/26/26
// The perfect tool for setting rc aircraft control throw, and other angle measurement needs.
// This code is written for the "Xiao NRF52840 Sense" board (use the "mbed" board definition).
// Standalone use with just a battery and a phone app (NRF Connect Mobile, for iOS and Android).
// Optional OLED display can be used for operation without a phone (SSD1306 i2c).
// Optional tare button can be connected for operation without a phone.

// Usage:
// Flash the board, power it on, and connect it to the app.
// Subscribe to the roll (1001) and pitch (1002) sensors ("down/line arrow" buttons).
// Configure the data as "UTF-8" ("quote" buttons).
// To zero both axis, send a true boolean (or 1) to the tare service (1003) (up arrow on sensor w/ long uuid), or push the tare button.
// If using a battery for power via the battery pads, subscribe to the battery service (1004) to read battery volts.
// Roll axis goes into the usb, pitch is across the usb.
// LEDs indicate status: Blue = BLE connected, Green flash = Data updated, Red flash = Taring

//  Wiring:
//  Device Pin | Xiao Pin
//  ------------ | ------------
//  battery +   | Bat+
//  battery -   | Bat-
//  OLED VCC    | 3v3
//  OLED GND    | GND
//  OLED SDA    | 4
//  OLED SCL    | 5
//  button 1a   | 10
//  button 1b   | gnd

// Arduino requires the seed gyro lib (not the 'duino download): https://github.com/Seeed-Studio/Seeed_Arduino_LSM6DS3

#include "ArduinoBLE.h"
#include "LSM6DS3.h"
#include "Wire.h"
#include <nrf52840.h>
#include <nrfx_saadc.h>
#include <AnalogIn.h>
#include <pinDefinitions.h>
#include <avr/dtostrf.h>
#include <Adafruit_SSD1306.h>

// User configuration
#define sampleCount 100 // # of samples between readings
#define tareLEDtime 2000   // msec wait while taring angles (longer than time required to collect sampleCount data points)
#define dataFlash 50   // msec to flash when data is sent
#define chargeCurrent LOW // Built in battery charger: HIGH = 50mA, LOW = 100mA
#define ledColorData LED_GREEN  // LED indicator colors, choose one each (LED_GREEN, LED_RED, LED_BLUE)
#define ledColorBLE LED_BLUE
#define ledColorTare LED_RED
#define tareButtonPin 11  // Pin connected to tare button (11 is IO, 10 is MOSI :P)
//#define oledFormatBig // uncomment for a larger degree display on the OLED (nice for single color screens, not so great with Y/B screens)
#define displayAlternatePeriod 2500 // msec to alternate between info when using oledFormatBig

// END User configuration

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

// BLE Descriptors (not read by NRF connect app unfortunately, but here in case some app does)
BLEDescriptor pitchDegreesDescriptor("2901", "Pitch Degrees");
BLEDescriptor rollDegreesDescriptor("2901", "Roll Degrees");
BLEDescriptor batteryVoltsDescriptor("2901", "Batt Volts");
BLEDescriptor tareCharDescriptor("2901", "Tare");

// SSD1306 OLED display parameters
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1

// I²C pins used by the SSD1306
// Xiao nRF52840 Sense: pin 4 and pin 5
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

float battery = 0.0;  // battery voltage
char batteryBuffer[20]; // printable byte array
float roll = 0;  // roll angle
char rollBuffer[20]; // printable byte array
float pitch = 0;  // pitch angle
char pitchBuffer[20]; // printable byte array
float rollRaw = 0.0;  // raw calculated roll
float pitchRaw = 0.0;  // raw calculated pitch
float accX = 0.0; // accelerator sums
float accY = 0.0;
float accZ = 0.0;
float tareRoll = 0.0;  // raw roll value when tared
float tarePitch = 0.0;  // raw pitch value when tared
long currentMillis = 0; // global timer
long previousData = 0;  // msec since data was sent
long previousTare = 0;  // msec timer for data led flash
long previousDisplay = 0;  // msec timer for data led flash
u_int8_t displayIndex = 0; // display index for alternating displays
bool dataLedFlag = 0;  // flag for data led flash
bool tareFlag = 0;  // flag for tare routine
bool tareLedFlag = 0; // flag for tare led timer
bool centralFlag = 0; // flag if BLE is connected
String centralAddress = "0"; // array to store MAC address of connected BLE device
u_int8_t samples = 0;  // sample count storage

#define SPLASH_HEIGHT   64
#define SPLAST_WIDTH    128

static const unsigned char splashScreen[] = { // Size 128x64 pixels
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x13, 0xf8, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe7, 0x93, 0xf1, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe7, 0x93, 0xf3, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe4, 0x13, 0xf2, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe4, 0x13, 0xf2, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe7, 0x93, 0xf1, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x10, 0x38, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x30, 0x1c, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff,
    0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xfc, 0xe0, 0xf8, 0x27, 0xce, 0x0f, 0x03, 0xc0, 0x0e, 0x04, 0x03, 0x02, 0x01, 0xff, 0xff,
    0xff, 0xfc, 0xc4, 0x71, 0xe7, 0xcc, 0x46, 0x31, 0x98, 0xcc, 0x7f, 0x1e, 0x3e, 0x31, 0xff, 0xff,
    0xff, 0xfc, 0x9f, 0x27, 0xe7, 0xc9, 0xf2, 0x79, 0x98, 0xcc, 0xff, 0x9e, 0x7e, 0x78, 0xff, 0xff,
    0xff, 0xfc, 0x9f, 0x27, 0xe7, 0xc9, 0xf2, 0x79, 0xbd, 0xed, 0x8f, 0x9e, 0x42, 0x61, 0xff, 0xff,
    0xff, 0xfc, 0x9f, 0x27, 0xe7, 0xc9, 0xf2, 0x79, 0x99, 0xec, 0x8f, 0x9e, 0x46, 0x41, 0xff, 0xff,
    0xff, 0xfc, 0x9f, 0x23, 0xe7, 0xc9, 0xf2, 0x39, 0x99, 0xec, 0xff, 0x9e, 0x3e, 0x71, 0xff, 0xff,
    0xff, 0xfc, 0x9f, 0x30, 0x20, 0x49, 0xf3, 0x03, 0x18, 0xc6, 0x07, 0x9f, 0x02, 0x78, 0xff, 0xff,
    0xff, 0xfc, 0x9f, 0x38, 0x20, 0x49, 0xf3, 0x87, 0xbd, 0xef, 0x07, 0x9f, 0x82, 0x7c, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff,
    0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xc0, 0x60, 0x1b, 0xfe, 0x01, 0xbf, 0xe0, 0x18, 0x07, 0xc0, 0x30, 0x1f, 0xff,
    0xff, 0xef, 0xff, 0x80, 0x20, 0x1b, 0xf6, 0x01, 0xbf, 0xc0, 0x18, 0x02, 0x00, 0x20, 0x1f, 0xff,
    0xff, 0xef, 0xff, 0xf3, 0xe7, 0x9b, 0xf6, 0x7f, 0xbf, 0xcf, 0xd9, 0xf2, 0x79, 0xe7, 0xff, 0xff,
    0xff, 0xe3, 0x67, 0xfb, 0xe0, 0x1b, 0xf6, 0x7f, 0xbf, 0xcf, 0xdb, 0xfa, 0x7b, 0xe7, 0xff, 0xff,
    0xff, 0xe9, 0x6f, 0xfb, 0xe0, 0x1b, 0xf6, 0xc1, 0xbf, 0xcf, 0xdb, 0xfa, 0x7b, 0xe0, 0x1f, 0xff,
    0xff, 0xed, 0xaf, 0xfb, 0xe1, 0xfb, 0xf6, 0xc1, 0xbf, 0xcf, 0xdb, 0xf2, 0x7b, 0xe0, 0x1f, 0xff,
    0xff, 0xed, 0x9f, 0xfb, 0xe0, 0x78, 0x06, 0x01, 0x80, 0x40, 0x18, 0x06, 0x7b, 0xe0, 0x1f, 0xff,
    0xff, 0xe1, 0x9f, 0xfb, 0xef, 0x18, 0x06, 0x01, 0x80, 0x40, 0x18, 0x0e, 0x7b, 0xe0, 0x1f, 0xff,
    0xff, 0xff, 0xdf, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xbf, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

void readData()  {
  // Reads samples from the IMU and analog sensor, calculates the voltage and angles, and adds values to averaging sums
  int batteryADC = analogRead(batteryAnalogPin); // read battery adc
  battery += float(batteryADC); // calc actual battery volts w/ 3v3 reg and 10bit adc

  // Read angle between X and Z accelerometer axis
  accX += myIMU.readFloatAccelX();
  accY += myIMU.readFloatAccelY();
  accZ += myIMU.readFloatAccelZ();
}

void updateDataBuffers() {
  // Prints and updates data buffers
  // Calculate averaged and tared angles
  accX = accX / samples;
  accY = accY / samples;
  accZ = accZ / samples;
  rollRaw = atan2(accY, accZ) * 57.2958;
  pitchRaw = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 57.2958;
  roll = ( rollRaw ) - tareRoll;
  pitch = ( pitchRaw ) - tarePitch;
  // Calculate averaged battery voltage
  battery = battery / samples;
  battery = (battery * 3.3) / 1024 * 1510.0 / 510.0; // calc actual battery volts w/ 3v3 reg and 10bit adc

  // Stringify float angles to 1 decimal place
  dtostrf(roll, 5, 1, rollBuffer);
  dtostrf(pitch, 5, 1, pitchBuffer);
  // Stringify float voltage to 2 decimal places
  dtostrf(battery, 4, 2, batteryBuffer);

  // Print debug
  Serial.print("Roll/Pitch/Battery: ");
  Serial.print(rollBuffer);
  Serial.print(", ");
  Serial.print(pitchBuffer);
  Serial.print(", ");
  Serial.println(batteryBuffer);

  // rezero values
  samples = 0.0;
  accX = 0.0;
  accY = 0.0;
  accZ = 0.0;
  battery = 0.0;
}

void sendBLE() {
  // Sends data buffers to BLE
  rollDegrees.writeValue(rollBuffer);
  pitchDegrees.writeValue(pitchBuffer);
  batteryVolts.writeValue(batteryBuffer);
}

void sendOLED() {
  // Update the OLED
  display.clearDisplay();
  #ifdef oledFormatBig
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print("R:");
    display.setTextSize(3);
    // right justify values
    // strf() takes care of most of the work since values range +-180.0
    if(roll > -100) { // add a space only if we have a value > -100
      display.print(" ");
    }
    display.println(rollBuffer);
    display.setTextSize(1);
    display.print("P:");
    display.setTextSize(3);
    if(pitch > -100) { // add a space only if we have a value > -100
      display.print(" ");
    }
    display.println(pitchBuffer);
    display.setTextSize(1);
    display.setCursor(0, 52);
    // update alternating display index when enough time has passed
    if ( currentMillis - previousDisplay > displayAlternatePeriod) {
      previousDisplay = currentMillis;
      if (displayIndex == 0)  {
        displayIndex = 1;
      }
      else {
        displayIndex = 0;
      }
    }
    // show alternating display based on the current index
    if (displayIndex == 0)
    {
      display.print("Battery:      ");
      display.print(batteryBuffer);
      display.println(" V");
    }
    else {
      display.print("BT: ");
      if (centralFlag) {   // show BLE connections
        display.println(centralAddress);
      }
      else {
        display.println("    disconnected");
      }
    }
  #endif
  #ifndef oledFormatBig
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print("R: ");
    // right justify values
    // strf() takes care of most of the work since values range +-180.0
    if(roll > -100) { // add a space only if we have a value > -100
      display.print(" ");
    }
    display.print(rollBuffer);
    display.println((char)247); // degree symbol
    display.print("P: ");
    if(pitch > -100) { // add a space only if we have a value > -100
      display.print(" ");
    }
    display.print(pitchBuffer);
    display.println((char)247);
    
    display.setCursor(0, 38);
    display.setTextSize(1);
    display.print("BT: ");
    if (centralFlag) {   // show BLE connections
      display.println(centralAddress);
    }
    else {
      display.println("    disconnected");
    }
    display.setCursor(0, 52);
    display.print("Battery:      ");
    display.print(batteryBuffer);
    display.println(" V");
  #endif
  display.display();
}

void tareAxis() {
  // Sets axis zeros at the current position
  tareRoll = rollRaw;
  tarePitch = pitchRaw;
}

void setup()
{
  Serial.begin(115200);    // initialize serial communication
  delay(1000);    // give time for serial to startup

  pinMode(tareButtonPin, INPUT_PULLUP); // init tare button

  pinMode(LED_RED, OUTPUT); // initialize the built-in LEDs
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

  pinMode (chargePin, OUTPUT);  // init charge current setting pin
  pinMode (batteryReadPin, OUTPUT);  // init charge current setting pin

  Serial.println("BLE Inclinometer");
  Serial.println("by: Truglodite");
  // prepare I²C for the OLED
  Wire.begin();

  // Initialize SSD1306 OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // 0x3C is common I2C address
    Serial.println("OLED failed!");
  } else {
    Serial.println("OLED - OK");
  }

  //Display Splashscreen
  display.clearDisplay();
  display.drawBitmap( 0, 0, splashScreen, SPLAST_WIDTH, SPLASH_HEIGHT, WHITE);
  display.display();
  delay(3000);

// …rest of setup()
  // begin initialization
  if (!BLE.begin()) 
  {
    Serial.println("BLE failed!");
  }
  else  {
    Serial.println("BLE - OK");
  }

  if (myIMU.begin() != 0) {
      Serial.println("IMU error!");
  } else {
      Serial.println("IMU - OK");
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

  // Configure IMU for slow-precise angle measurement
  myIMU.settings.gyroEnabled = 0;  //Can be 0 or 1
  myIMU.settings.accelEnabled = 1;
  myIMU.settings.accelRange = 2;      //Max G force readable.  Can be: 2, 4, 8, 16
  myIMU.settings.accelSampleRate = 208;  //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664, 13330
  myIMU.settings.accelBandWidth = 50;  //Hz.  Can be: 50, 100, 200, 400;

  digitalWrite(ledColorBLE, HIGH);  // Ensure LEDs are off before looping
  digitalWrite(ledColorData, HIGH);
  digitalWrite(ledColorTare, HIGH);

  Serial.println("Bluetooth® waiting for connections to 'Angle Monitor'");
}

void loop()
{
  digitalWrite(chargePin, chargeCurrent); // configure usb charger
  digitalWrite(batteryReadPin, LOW);  // configure battery measurement
  
  currentMillis = millis();

  BLEDevice central = BLE.central();

  // Check BLE central if not connected
  if (!central) {
    if(centralFlag)  {
      Serial.print("Disconnected from central: ");
      Serial.println(central.address());
      digitalWrite(ledColorBLE, HIGH);  // Turn off led while not connected
      centralFlag = 0;
    }
  }
  // Central is connected
  else  {
    if (!centralFlag)  {
      digitalWrite(ledColorBLE, LOW); // Turn on led while connected
      centralAddress = central.address();
      Serial.print("Connected to central: ");
      Serial.println(centralAddress);
      centralFlag = 1;
    }
  }

  // count more samples if needed
  if (samples < sampleCount)  {
    readData();
    samples++;
  }
  // enough samples, send data
  else  {
    previousData = currentMillis;
    updateDataBuffers();
    if (central.connected())  {
      sendBLE();
    }
    sendOLED();
    digitalWrite(ledColorData, LOW); // turn on data led flash
    dataLedFlag = 1;
  }
  // data led is on, and time to turn it off
  if (dataLedFlag && currentMillis - previousData >= dataFlash)  {
    digitalWrite(ledColorData, HIGH);
    dataLedFlag = 0;
  }

  // Handle tare button, no debounce needed since taring takes a while
  if (!digitalRead(tareButtonPin) && !tareFlag) {
    // Button is pressed and tare hasn't started
    previousTare = currentMillis; //start led timer
    digitalWrite(ledColorTare, LOW); // turn on tare led flash
    tareFlag = 1;
    tareLedFlag = 1;
    Serial.println("Tare axis via button");
  }

  // Tare recieved, turn on LED and set tare flag
  if (tareChar.written() && !tareFlag) {
    if (tareChar.value()) {    // received a HIGH value
      previousTare = currentMillis; //start led timer
      digitalWrite(ledColorTare, LOW); // turn on tare led flash
      tareFlag = 1;
      tareLedFlag = 1;
      Serial.println("Tare axis via BLE");
    }
  }
  // We have a tare flag, and enough samples to tare
  if (tareFlag && samples >= sampleCount) {
    tareAxis();
    tareFlag = 0;
  }
  // tare led is on, and time to turn it off
  if (tareLedFlag && currentMillis - previousTare >= tareLEDtime)  {
    digitalWrite(ledColorTare, HIGH);
    tareLedFlag = 0;
  }

}

