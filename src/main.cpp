#include <Arduino.h>
#include "ArduinoBLE.h"
#include "LSM6DS3.h"
#include "Wire.h"
#include <Adafruit_SSD1306.h>

// ================= USER CONFIG =================
#define sampleCount 100
#define tareLEDtime 2000
#define dataFlash 50
#define displayUpdateInterval 150
#define tareButtonPin 11

#define ledColorData LED_GREEN
#define ledColorBLE LED_BLUE
#define ledColorTare LED_RED

#define chargeCurrent LOW

// Battery calculation constants
#define ADC_REF 3.3
#define ADC_RES 1024.0
#define R1 1510.0
#define R2 510.0

#define RAD_TO_DEG 57.2958

// ================= HARDWARE =================
#define chargePin P0_13
#define batteryReadPin P0_14
#define batteryAnalogPin P0_31

LSM6DS3 myIMU(I2C_MODE, 0x6A);

// ================= BLE =================
BLEService angleMonitorService("8acafa20-26e9-4d16-a792-cf7de147c01c");

BLEFloatCharacteristic rollChar("1001", BLERead | BLENotify);
BLEFloatCharacteristic pitchChar("1002", BLERead | BLENotify);
BLEFloatCharacteristic batteryChar("1004", BLERead | BLENotify);
BLEByteCharacteristic tareChar("1003", BLERead | BLEWrite);

// ================= OLED =================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ================= GLOBALS =================
float accX = 0, accY = 0, accZ = 0;
float roll = 0, pitch = 0;
float rollRaw = 0, pitchRaw = 0;
float tareRoll = 0, tarePitch = 0;
float battery = 0;

uint8_t samples = 0;

unsigned long currentMillis = 0;
unsigned long previousDisplay = 0;
unsigned long previousDataFlash = 0;
unsigned long previousTare = 0;

bool dataLedFlag = false;
bool tareLedFlag = false;
bool tareFlag = false;
bool centralFlag = false;

char centralAddress[20] = "0";

// ================= FUNCTIONS =================

void readData() {
  battery += analogRead(batteryAnalogPin);

  accX += myIMU.readFloatAccelX();
  accY += myIMU.readFloatAccelY();
  accZ += myIMU.readFloatAccelZ();
}

void updateData() {
  if (samples == 0) return;

  accX /= samples;
  accY /= samples;
  accZ /= samples;

  float yz = sqrt(accY * accY + accZ * accZ);

  rollRaw = atan2(accY, accZ) * RAD_TO_DEG;
  pitchRaw = atan2(-accX, yz) * RAD_TO_DEG;

  roll = rollRaw - tareRoll;
  pitch = pitchRaw - tarePitch;

  battery /= samples;
  battery = (battery * ADC_REF / ADC_RES) * ((R1 + R2) / R2);

  Serial.print("Roll/Pitch/Battery: ");
  Serial.print(roll);
  Serial.print(", ");
  Serial.print(pitch);
  Serial.print(", ");
  Serial.println(battery);

  accX = accY = accZ = 0;
  battery = 0;
  samples = 0;
}

void sendBLE() {
  rollChar.writeValue(roll);
  pitchChar.writeValue(pitch);
  batteryChar.writeValue(battery);
}

void sendOLED() {
  if (currentMillis - previousDisplay < displayUpdateInterval) return;
  previousDisplay = currentMillis;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print("R:");
  display.print(roll, 1);
  display.print((char)247);

  display.setCursor(0, 20);
  display.print("P:");
  display.print(pitch, 1);
  display.print((char)247);

  display.setTextSize(1);

  display.setCursor(0, 45);
  display.print("BT: ");
  if (centralFlag) display.println(centralAddress);
  else display.println("disconnected");

  display.setCursor(0, 55);
  display.print("Bat:");
  display.print(battery, 2);
  display.print("V");

  display.display();
}

void tareAxis() {
  tareRoll = rollRaw;
  tarePitch = pitchRaw;
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(tareButtonPin, INPUT_PULLUP);

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

  pinMode(chargePin, OUTPUT);
  pinMode(batteryReadPin, OUTPUT);

  Wire.begin();

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED failed!");
  }

  if (!BLE.begin()) {
    Serial.println("BLE failed!");
  }

  if (myIMU.begin() != 0) {
    Serial.println("IMU error!");
  }

  BLE.setDeviceName("Angle Monitor");
  BLE.setLocalName("Angle Monitor");
  BLE.setAdvertisedService(angleMonitorService);

  angleMonitorService.addCharacteristic(rollChar);
  angleMonitorService.addCharacteristic(pitchChar);
  angleMonitorService.addCharacteristic(batteryChar);
  angleMonitorService.addCharacteristic(tareChar);

  BLE.addService(angleMonitorService);
  BLE.advertise();

  // IMU settings
  myIMU.settings.gyroEnabled = 0;
  myIMU.settings.accelEnabled = 1;
  myIMU.settings.accelRange = 2;
  myIMU.settings.accelSampleRate = 208;
  myIMU.settings.accelBandWidth = 50;

  digitalWrite(ledColorBLE, HIGH);
  digitalWrite(ledColorData, HIGH);
  digitalWrite(ledColorTare, HIGH);

  Serial.println("Bluetooth® device active");
}

// ================= LOOP =================
void loop() {
  currentMillis = millis();

  digitalWrite(chargePin, chargeCurrent);
  digitalWrite(batteryReadPin, LOW);

  BLEDevice central = BLE.central();

  if (central) {
    if (!centralFlag) {
      strcpy(centralAddress, central.address().c_str());
      digitalWrite(ledColorBLE, LOW);
      Serial.print("Connected: ");
      Serial.println(centralAddress);
      centralFlag = true;
    }
  } else {
    if (centralFlag) {
      digitalWrite(ledColorBLE, HIGH);
      Serial.println("Disconnected");
      centralFlag = false;
    }
  }

  // Continuous sampling
  readData();
  samples++;

  if (samples >= sampleCount) {
    updateData();

    if (central.connected()) {
      sendBLE();
    }

    sendOLED();

    digitalWrite(ledColorData, LOW);
    previousDataFlash = currentMillis;
    dataLedFlag = true;
  }

  // Data LED off timer
  if (dataLedFlag && currentMillis - previousDataFlash >= dataFlash) {
    digitalWrite(ledColorData, HIGH);
    dataLedFlag = false;
  }

  // Button debounce
  if (!digitalRead(tareButtonPin)) {
    delay(20);
    if (!digitalRead(tareButtonPin)) {
      tareFlag = true;
      previousTare = currentMillis;
      digitalWrite(ledColorTare, LOW);
      tareLedFlag = true;
      Serial.println("Tare button");
    }
  }

  // BLE tare
  if (tareChar.written() && tareChar.value()) {
    tareFlag = true;
    previousTare = currentMillis;
    digitalWrite(ledColorTare, LOW);
    tareLedFlag = true;
    Serial.println("Tare BLE");
  }

  if (tareFlag && samples >= sampleCount) {
    tareAxis();
    tareFlag = false;
  }

  if (tareLedFlag && currentMillis - previousTare >= tareLEDtime) {
    digitalWrite(ledColorTare, HIGH);
    tareLedFlag = false;
  }
}