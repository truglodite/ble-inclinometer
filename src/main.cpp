#include <Arduino.h>

// Truglodite 2/10/26
// ble-inclinometer
// The perfect tool for setting rc plane control throw, and other angle measurement needs.
// This code is writed for the "Xiao NRF52840 Sense" board (use the "mbed" board definition).
// Standalone use with just a battery and a phone app (NRF Connect Mobile, for iOS and Android).
// Optional OLED display can be used for operation without a phone (SSD1306 i2c).
// Optional tare button can be connected for operation without a phone.

// Usage:
// Flash the board, power it on, and connect it to the app.
// Subscribe to the roll (1001) and pitch (1002) sensors ("down/line arrow" buttons).
// Configure the data as "UTF-8" ("quote" buttons).
// Alternatively, read the optional OLED screen
// To zero both axis, send a true boolean (or 1) to the tare service (1003) (up arrow on sensor w/ long uuid).
// Alternatively, if a tare button is installed simply press it to zero both axis.
// If using a battery for power via the battery pads, subscribe to the battery service (1004) to read battery volts.
// Roll axis is into the usb, pitch is across the usb
// LEDs indicate status: Blue = BLE connected, Green flash = Data updated, Red flash = Taring

//  Wiring:
//  Device Pin | Xiao Pin
//  ------------------------
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
// #define oledFormatBig // uncomment for a larger degree display on the OLED (nice for single color screens, not so great with Y/B screens)

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
long previousData = 0;  // msec since data was sent
long previousTare = 0;  // msec timer for data led flash
bool dataLedFlag = 0;  // flag for data led flash
bool tareFlag = 0;  // flag for tare routine
bool tareLedFlag = 0; // flag for tare led timer
bool centralFlag = 0; // flag if BLE is connected
u_int8_t samples = 0;  // sample count storage

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
  // Sends data buffers to OLED
  display.clearDisplay();
  #ifdef oledFormatBig
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print("R:");
    display.setTextSize(3);
    display.println(rollBuffer);
    display.setTextSize(1);
    display.print("P:");
    display.setTextSize(3);
    display.println(pitchBuffer);
    display.setTextSize(1);
    display.setCursor(0, 52);
    display.print("   Battery: ");
    display.print(batteryBuffer);
    display.println(" V");
  #endif
  #ifndef oledFormatBig
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print("R:");
    display.println(rollBuffer);
    display.print("P:");
    display.println(pitchBuffer);
    display.setCursor(0, 52);
    display.setTextSize(1);
    display.print("   Battery: ");
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
  
  long currentMillis = millis();

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
      Serial.print("Connected to central: ");
      Serial.println(central.address());
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

