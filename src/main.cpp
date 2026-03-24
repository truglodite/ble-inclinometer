#include <Arduino.h>
#include "ArduinoBLE.h"
#include "LSM6DS3.h"
#include "Wire.h"
#include <Adafruit_SSD1306.h>
#include <avr/dtostrf.h>

// ================= USER CONFIG =================
#define tareLEDtime 2000
#define dataFlash 50
#define displayUpdateInterval 150
#define displayAlternatePeriod 2500
#define tareButtonPin 11

#define ledColorData LED_GREEN
#define ledColorBLE LED_BLUE
#define ledColorTare LED_RED

#define chargeCurrent LOW
#define oledFormatBig

// Battery constants
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

// ================= SPLASH =================
static const unsigned char splashScreen[] PROGMEM = {
  // 👉 KEEP your original bitmap here
};

// ================= KALMAN FILTER =================
class Kalman {
public:
  float Q_angle = 0.001;
  float Q_bias = 0.003;
  float R_measure = 0.03;

  float angle = 0, bias = 0, rate = 0;
  float P[2][2] = {{0,0},{0,0}};

  float getAngle(float newAngle, float newRate, float dt) {
    rate = newRate - bias;
    angle += dt * rate;

    P[0][0] += dt * (dt*P[1][1] - P[1][0] - P[0][1] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    float S = P[0][0] + R_measure;
    float K[2] = {P[0][0]/S, P[1][0]/S};

    float y = newAngle - angle;
    angle += K[0] * y;
    bias += K[1] * y;

    float P00 = P[0][0], P01 = P[0][1];

    P[0][0] -= K[0] * P00;
    P[0][1] -= K[0] * P01;
    P[1][0] -= K[1] * P00;
    P[1][1] -= K[1] * P01;

    return angle;
  }
};

Kalman kalmanRoll, kalmanPitch;

// ================= GLOBALS =================
float accX, accY, accZ;
float gyroX, gyroY;

float roll = 0, pitch = 0;
float rollRaw = 0, pitchRaw = 0;
float tareRoll = 0, tarePitch = 0;
float battery = 0;

char rollBuffer[10];
char pitchBuffer[10];

unsigned long currentMillis = 0;
unsigned long previousDisplay = 0;
unsigned long previousAlternate = 0;
unsigned long previousTare = 0;
unsigned long lastTime = 0;

bool tareFlag = false;
bool tareLedFlag = false;
bool centralFlag = false;

uint8_t displayIndex = 0;
char centralAddress[20] = "0";

// ================= FUNCTIONS =================

void readData() {
  accX = myIMU.readFloatAccelX();
  accY = myIMU.readFloatAccelY();
  accZ = myIMU.readFloatAccelZ();

  gyroX = myIMU.readFloatGyroX();
  gyroY = myIMU.readFloatGyroY();
}

void updateData() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;
  if (dt <= 0) return;

  float rollAcc = atan2(accY, accZ) * RAD_TO_DEG;
  float pitchAcc = atan2(-accX, sqrt(accY*accY + accZ*accZ)) * RAD_TO_DEG;

  rollRaw = kalmanRoll.getAngle(rollAcc, gyroX, dt);
  pitchRaw = kalmanPitch.getAngle(pitchAcc, gyroY, dt);

  roll = rollRaw - tareRoll;
  pitch = pitchRaw - tarePitch;

  // RIGHT-JUSTIFIED FORMAT
  dtostrf(roll, 6, 1, rollBuffer);
  dtostrf(pitch, 6, 1, pitchBuffer);

  float raw = analogRead(batteryAnalogPin);
  battery = (raw * ADC_REF / ADC_RES) * ((R1 + R2) / R2);
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

#ifdef oledFormatBig

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("R:");

  display.setTextSize(3);
  display.println(rollBuffer);

  display.setTextSize(1);
  display.print("P:");

  display.setTextSize(3);
  display.println(pitchBuffer);

  if (currentMillis - previousAlternate > displayAlternatePeriod) {
    previousAlternate = currentMillis;
    displayIndex = !displayIndex;
  }

  display.setTextSize(1);
  display.setCursor(0, 52);

  if (displayIndex == 0) {
    display.print("Battery: ");
    display.print(battery, 2);
    display.print(" V");
  } else {
    display.print("BT: ");
    if (centralFlag) display.print(centralAddress);
    else display.print("disconnected");
  }

#else

  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print("R:");
  display.println(rollBuffer);

  display.setCursor(0, 20);
  display.print("P:");
  display.println(pitchBuffer);

#endif

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
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  display.clearDisplay();
  display.drawBitmap(0, 0, splashScreen, 128, 64, SSD1306_WHITE);
  display.display();
  delay(2000);

  BLE.begin();
  myIMU.begin();

  myIMU.settings.gyroEnabled = 1;
  myIMU.settings.accelEnabled = 1;
  myIMU.settings.accelRange = 2;
  myIMU.settings.accelSampleRate = 208;

  BLE.setDeviceName("Angle Monitor");
  BLE.setLocalName("Angle Monitor");
  BLE.setAdvertisedService(angleMonitorService);

  angleMonitorService.addCharacteristic(rollChar);
  angleMonitorService.addCharacteristic(pitchChar);
  angleMonitorService.addCharacteristic(batteryChar);
  angleMonitorService.addCharacteristic(tareChar);

  BLE.addService(angleMonitorService);
  BLE.advertise();

  lastTime = millis();

  digitalWrite(ledColorBLE, HIGH);
  digitalWrite(ledColorData, HIGH);
  digitalWrite(ledColorTare, HIGH);
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
      centralFlag = true;
    }
  } else {
    if (centralFlag) {
      digitalWrite(ledColorBLE, HIGH);
      centralFlag = false;
    }
  }

  readData();
  updateData();

  if (central.connected()) sendBLE();
  sendOLED();

  if (!digitalRead(tareButtonPin)) {
    delay(20);
    if (!digitalRead(tareButtonPin)) {
      tareFlag = true;
      previousTare = currentMillis;
      digitalWrite(ledColorTare, LOW);
      tareLedFlag = true;
    }
  }

  if (tareChar.written() && tareChar.value()) {
    tareFlag = true;
    previousTare = currentMillis;
    digitalWrite(ledColorTare, LOW);
    tareLedFlag = true;
  }

  if (tareFlag) {
    tareAxis();
    tareFlag = false;
  }

  if (tareLedFlag && currentMillis - previousTare >= tareLEDtime) {
    digitalWrite(ledColorTare, HIGH);
    tareLedFlag = false;
  }
}