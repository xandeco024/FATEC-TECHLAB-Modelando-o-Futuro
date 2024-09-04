#include <Arduino.h>
#include <LiquidCrystal.h>

//pin definition

//temp pins
#define plusTempBtnPin A2
#define scaleTempBtnPin A1
#define minusTempBtnPin A0
#define tempSensorPin A3
#define relePin 7

//stepper motor pins
//#define stepSpeedPin 6 (directly connected to 5V)
#define stepIn1Pin 2
#define stepIn2Pin 3
#define stepIn3Pin 4
#define stepIn4Pin 5
#define stepControlPin 6
#define plusStepSpeedPin A5
#define minusStepSpeedPin A4

int plusButtonState = 0;
int scaleButtonState = 0;
int minusButtonState = 0;

LiquidCrystal lcd(8, 9, 10, 11, 12, 13);

float scale = 1;
double currentTemp = 0;
float targetTemp = 0;

int tempReadDelay = 1000;
int tempReadTime = 0;

void setup() {
    pinMode(tempSensorPin, INPUT);
    pinMode(relePin, OUTPUT);

    pinMode(plusTempBtnPin, INPUT);
    pinMode(scaleTempBtnPin, INPUT);
    pinMode(minusTempBtnPin, INPUT);

    //pinMode(stepSpeedPin, OUTPUT);
    pinMode(stepIn1Pin, OUTPUT);
    pinMode(stepIn2Pin, OUTPUT);
    pinMode(stepIn3Pin, OUTPUT);
    pinMode(stepIn4Pin, OUTPUT);

    lcd.begin(16, 2);
    Serial.begin(9600);
}

void TempScreen() {
    //screen handling

    // lcd.setCursor(0, 0);
    // lcd.print("Cur:");
    // //print the current temp in the following format: 999.9C
    // lcd.print(currentTemp, 1);
    // lcd.print("C   ");

    // lcd.setCursor(0, 1);
    // lcd.print("Tar:");
    // lcd.print(targetTemp, 1);
    // lcd.print("C   ");

    // lcd.setCursor(12, 1);
    // //pint the scale with 1 decimal
    // lcd.print("S");
    // lcd.print(scale);

    //999 | 999ºC S100
    //DES 100 | 100 RPM
}

int ReadTemp() {
    //temperature reading with delay but not with the delay function
    if (millis() - tempReadTime > tempReadDelay) {
        tempReadTime = millis();
        currentTemp = analogRead(tempSensorPin);
    }
}

void loop() {
    //temperature reading
    ReadTemp();

    //button handling
    if (digitalRead(scaleTempBtnPin) == HIGH && scaleButtonState == 0) {
        scaleButtonState = 1;

        if (scale == 1) {
            scale = 10;
        } 
        else if (scale == 10) {
            scale = 100;
        } 
        else if (scale == 100) {
            scale = 1;
        }
    }
    else if (digitalRead(scaleTempBtnPin) == LOW) {
        scaleButtonState = 0;
    }

    if (digitalRead(plusTempBtnPin) == HIGH && plusButtonState == 0) {
        plusButtonState = 1;
        targetTemp += scale;
    }
    else if (digitalRead(plusTempBtnPin) == LOW) {
        plusButtonState = 0;
    }

    if (digitalRead(minusTempBtnPin) == HIGH && minusButtonState == 0) {
        minusButtonState = 1;
        targetTemp -= scale;
        if (targetTemp < 0) {
            targetTemp = 0;
        }
    }
    else if (digitalRead(minusTempBtnPin) == LOW) {
        minusButtonState = 0;
    }

    //temperature control
    if (currentTemp < targetTemp) {
        digitalWrite(relePin, HIGH);
    } 
    else 
    {
        digitalWrite(relePin, LOW);
    }

    TempScreen(); 

    //stepper motor control

    // digitalWrite(stepIn1Pin, HIGH);
    // digitalWrite(stepIn2Pin, LOW);
    // digitalWrite(stepIn3Pin, LOW);
    // digitalWrite(stepIn4Pin, LOW);
    // delay(10);

    // digitalWrite(stepIn1Pin, LOW);
    // digitalWrite(stepIn2Pin, HIGH);
    // digitalWrite(stepIn3Pin, LOW);
    // digitalWrite(stepIn4Pin, LOW);
    // delay(10);

    // digitalWrite(stepIn1Pin, LOW);
    // digitalWrite(stepIn2Pin, LOW);
    // digitalWrite(stepIn3Pin, HIGH);
    // digitalWrite(stepIn4Pin, LOW);
    // delay(10);

    // digitalWrite(stepIn1Pin, LOW);
    // digitalWrite(stepIn2Pin, LOW);
    // digitalWrite(stepIn3Pin, LOW);
    // digitalWrite(stepIn4Pin, HIGH);
    // delay(10);
}