#include <Arduino.h>
#include "ArduinoBLE.h"
#include "LSM6DS3.h"
#include "Wire.h"
#include <Adafruit_SSD1306.h>
#include <math.h>
#include <avr/dtostrf.h>

// User configuration
#define sampleCount 100
#define tareLEDtime 2000
#define dataFlash 50
#define chargeCurrent LOW
#define ledColorData LED_GREEN
#define ledColorBLE LED_BLUE
#define ledColorTare LED_RED
#define tareButtonPin 11
#define displayAlternatePeriod 2500

// Pins
#define chargePin P0_13
#define batteryReadPin P0_14
#define batteryAnalogPin P0_31

// IMU
LSM6DS3 myIMU(I2C_MODE, 0x6A);

// BLE UUIDs
#define BLE_UUID_ANGLE_MONITOR_SERVICE "8acafa20-26e9-4d16-a792-cf7de147c01c"
#define BLE_UUID_ROLL_DEGREES  "1001"
#define BLE_UUID_PITCH_DEGREES "1002"
#define BLE_UUID_TARE_SWITCH   "1003"
#define BLE_UUID_BATTERY_VOLTS "1004"

// BLE service & characteristics
BLEService angleMonitorService(BLE_UUID_ANGLE_MONITOR_SERVICE);
BLEStringCharacteristic batteryVolts(BLE_UUID_BATTERY_VOLTS, BLERead | BLENotify, 20);
BLEStringCharacteristic rollDegrees(BLE_UUID_ROLL_DEGREES, BLERead | BLENotify, 20);
BLEStringCharacteristic pitchDegrees(BLE_UUID_PITCH_DEGREES, BLERead | BLENotify, 20);
BLEByteCharacteristic tareChar(BLE_UUID_TARE_SWITCH, BLERead | BLEWrite);

// OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Splash screen
#define SPLASH_WIDTH 128
#define SPLASH_HEIGHT 64
static const unsigned char splashScreen[] = {
  /* ... (use your existing 128x64 splashScreen array here) ... */
};

// Globals
float battery = 0.0;
char batteryBuffer[20];
float roll = 0, pitch = 0;
char rollBuffer[20], pitchBuffer[20];
float rollRaw = 0.0, pitchRaw = 0.0;
float accX = 0.0, accY = 0.0, accZ = 0.0;
float tareRoll = 0.0, tarePitch = 0.0;
unsigned long currentMillis = 0, previousData = 0, previousTare = 0, previousDisplay = 0;
uint8_t displayIndex = 0, samples = 0;
bool dataLedFlag = false, tareFlag = false, tareLedFlag = false, centralFlag = false;
String centralAddress = "0";

// Kalman filter class
class Kalman {
public:
    float Q_angle = 0.0005;
    float Q_bias = 0.0005;
    float R_measure = 0.08;
    float angle = 0.0;
    float bias = 0.0;
    float P[2][2] = {{0,0},{0,0}};

    float getAngle(float newAngle, float newRate, float dt) {
        // Predict
        float rate = newRate - bias;
        angle += dt * rate;
        P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += Q_bias * dt;

        // Update
        float S = P[0][0] + R_measure;
        float K[2];
        K[0] = P[0][0]/S;
        K[1] = P[1][0]/S;
        float y = newAngle - angle;
        angle += K[0]*y;
        bias += K[1]*y;
        float P00_temp = P[0][0];
        float P01_temp = P[0][1];
        P[0][0] -= K[0]*P00_temp;
        P[0][1] -= K[0]*P01_temp;
        P[1][0] -= K[1]*P00_temp;
        P[1][1] -= K[1]*P01_temp;

        return angle;
    }
};

Kalman kalmanRoll;
Kalman kalmanPitch;

// --- Functions ---
void readData() {
    int batteryADC = analogRead(batteryAnalogPin);
    battery += float(batteryADC);
    accX += myIMU.readFloatAccelX();
    accY += myIMU.readFloatAccelY();
    accZ += myIMU.readFloatAccelZ();
}

void updateDataBuffers() {
    accX /= samples;
    accY /= samples;
    accZ /= samples;

    rollRaw = atan2(accY, accZ) * 57.2958;
    pitchRaw = atan2(-accX, sqrt(accY*accY + accZ*accZ)) * 57.2958;

    // Apply Kalman filter
    float dt = 0.01; // approximate sample interval
    roll = kalmanRoll.getAngle(rollRaw - tareRoll, 0, dt);
    pitch = kalmanPitch.getAngle(pitchRaw - tarePitch, 0, dt);

    battery = battery / samples;
    battery = (battery*3.3)/1024 * 1510.0 / 510.0;

    // Right-justify formatting
    dtostrf(roll, 6, 1, rollBuffer);
    dtostrf(pitch, 6, 1, pitchBuffer);
    dtostrf(battery, 4, 2, batteryBuffer);

    samples = 0;
    accX = accY = accZ = battery = 0.0;
}

void sendBLE() {
    rollDegrees.writeValue(rollBuffer);
    pitchDegrees.writeValue(pitchBuffer);
    batteryVolts.writeValue(batteryBuffer);
}

void sendOLED() {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(2);
    display.setCursor(0,0);
    display.print("R: "); if(roll > -100) display.print(" "); display.print(rollBuffer); display.println((char)247);
    display.print("P: "); if(pitch > -100) display.print(" "); display.print(pitchBuffer); display.println((char)247);
    display.setTextSize(1);
    display.setCursor(0,38);
    display.print("BT: "); display.println(centralFlag ? centralAddress : "disconnected");
    display.setCursor(0,52);
    display.print("Battery: "); display.print(batteryBuffer); display.println(" V");
    display.display();
}

void tareAxis() {
    tareRoll = rollRaw;
    tarePitch = pitchRaw;
}

// --- Setup ---
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
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) Serial.println("OLED failed!");
    else {
        display.clearDisplay();
        display.drawBitmap(0,0,splashScreen,SPLASH_WIDTH,SPLASH_HEIGHT,WHITE);
        display.display();
        delay(3000);
    }

    BLE.begin();
    BLE.setDeviceName("Angle Monitor");
    BLE.setLocalName("Angle Monitor");
    BLE.setAdvertisedService(angleMonitorService);

    angleMonitorService.addCharacteristic(batteryVolts);
    angleMonitorService.addCharacteristic(rollDegrees);
    angleMonitorService.addCharacteristic(pitchDegrees);
    angleMonitorService.addCharacteristic(tareChar);
    BLE.addService(angleMonitorService);

    dtostrf(battery,4,2,batteryBuffer); batteryVolts.writeValue(batteryBuffer);
    dtostrf(roll,5,2,rollBuffer); rollDegrees.writeValue(rollBuffer);
    dtostrf(pitch,5,2,pitchBuffer); pitchDegrees.writeValue(pitchBuffer);
    tareChar.writeValue(0);
    BLE.advertise();

    if (myIMU.begin() != 0) Serial.println("IMU error!");

    // Kalman tuning already defaulted in class for ultra-smooth
    digitalWrite(ledColorBLE, HIGH);
    digitalWrite(ledColorData, HIGH);
    digitalWrite(ledColorTare, HIGH);
}

// --- Loop ---
void loop() {
    digitalWrite(chargePin, chargeCurrent);
    digitalWrite(batteryReadPin, LOW);
    currentMillis = millis();

    BLEDevice central = BLE.central();

    if (!central) {
        if(centralFlag) {
            Serial.print("Disconnected: "); Serial.println(central.address());
            digitalWrite(ledColorBLE,HIGH); centralFlag=false;
        }
    } else {
        if(!centralFlag) {
            centralAddress = central.address();
            Serial.print("Connected: "); Serial.println(centralAddress);
            digitalWrite(ledColorBLE,LOW); centralFlag=true;
        }
    }

    if(samples<sampleCount){ readData(); samples++; }
    else {
        previousData=currentMillis;
        updateDataBuffers();
        if(central.connected()) sendBLE();
        sendOLED();
        digitalWrite(ledColorData,LOW); dataLedFlag=true;
    }

    if(dataLedFlag && currentMillis-previousData>=dataFlash){ digitalWrite(ledColorData,HIGH); dataLedFlag=false; }

    if(!digitalRead(tareButtonPin) && !tareFlag){
        previousTare=currentMillis; digitalWrite(ledColorTare,LOW); tareFlag=true; tareLedFlag=true;
        Serial.println("Tare axis via button");
    }

    if(tareChar.written() && !tareFlag && tareChar.value()){
        previousTare=currentMillis; digitalWrite(ledColorTare,LOW); tareFlag=true; tareLedFlag=true;
        Serial.println("Tare axis via BLE");
    }

    if(tareFlag && samples>=sampleCount){ tareAxis(); tareFlag=false; }

    if(tareLedFlag && currentMillis-previousTare>=tareLEDtime){ digitalWrite(ledColorTare,HIGH); tareLedFlag=false; }
}