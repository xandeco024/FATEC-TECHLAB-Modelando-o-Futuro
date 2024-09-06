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

int plusTempBtnState = 0;
int scaleTempBtnState = 0;
int minuTempBtnState = 0;

int stepState = 0;
int stepControl = 0;
int stepSpeed = 0;
long stepSpeedTime = 0;
int plusStepSpeedBtnState = 0;
int minusStepSpeedBtnState = 0;
int stepControlBtnState = 0;

LiquidCrystal lcd(8, 9, 10, 11, 12, 13);

int scale = 1;
int currentTemp = 0;
int targetTemp = 0;

int tempReadDelay = 1000;
long tempReadTime = 0;

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
    lcd.setCursor(6, 0);
    lcd.print("|");
    lcd.setCursor(6, 1);
    lcd.print("|");

    lcd.setCursor(0, 0);
    lcd.print("C:");
    char tempCurrent[4];
    sprintf(tempCurrent, "%3d", currentTemp);
    lcd.print(tempCurrent);
    lcd.print("C");

    lcd.setCursor(0, 1);
    lcd.print("T:");
    char tempTarget[4];
    sprintf(tempTarget, "%3d", targetTemp);
    lcd.print(tempTarget);
    lcd.print("C");

    lcd.setCursor(7, 0);
    lcd.print("S:");
    char tempScale[4];
    sprintf(tempScale, "%3d", scale);
    lcd.print(tempScale);

    lcd.setCursor(7, 1);
    lcd.print("S:");
    char tempStepSpeed[4];
    sprintf(tempStepSpeed, "%3d", stepSpeed);
    lcd.print(tempStepSpeed);
    lcd.print("RPM");
}

void StepperControl() {
    if (stepControl == 1) {
        //control without the delay function

        //translate rpm to step speed (thats for later)
        if (millis() - stepSpeedTime > stepSpeed) {
            stepSpeedTime = millis();
            if (stepState == 0) {
                digitalWrite(stepIn1Pin, HIGH);
                digitalWrite(stepIn2Pin, LOW);
                digitalWrite(stepIn3Pin, LOW);
                digitalWrite(stepIn4Pin, LOW);
                stepState = 1;
            } 
            else if (stepState == 1) {
                digitalWrite(stepIn1Pin, LOW);
                digitalWrite(stepIn2Pin, HIGH);
                digitalWrite(stepIn3Pin, LOW);
                digitalWrite(stepIn4Pin, LOW);
                stepState = 2;
            } 
            else if (stepState == 2) {
                digitalWrite(stepIn1Pin, LOW);
                digitalWrite(stepIn2Pin, LOW);
                digitalWrite(stepIn3Pin, HIGH);
                digitalWrite(stepIn4Pin, LOW);
                stepState = 3;
            } 
            else if (stepState == 3) {
                digitalWrite(stepIn1Pin, LOW);
                digitalWrite(stepIn2Pin, LOW);
                digitalWrite(stepIn3Pin, LOW);
                digitalWrite(stepIn4Pin, HIGH);
                stepState = 0;
            }
        }
    }
}

void ReadTemp() {
    //temperature reading with delay but not with the delay function
    if (millis() - tempReadTime > tempReadDelay) {
        tempReadTime = millis();
        currentTemp = analogRead(tempSensorPin);
    }
}

void ReadButtons()
{
    if (digitalRead(scaleTempBtnPin) == HIGH && scaleTempBtnState == 0) {
        scaleTempBtnState = 1;

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
        scaleTempBtnState = 0;
    }

    if (digitalRead(plusTempBtnPin) == HIGH && plusTempBtnState == 0) {
        plusTempBtnState = 1;
        targetTemp += scale;
        if (targetTemp > 999) {
            targetTemp = 999;
        }
    }
    else if (digitalRead(plusTempBtnPin) == LOW) {
        plusTempBtnState = 0;
    }

    if (digitalRead(minusTempBtnPin) == HIGH && minuTempBtnState == 0) {
        minuTempBtnState = 1;
        targetTemp -= scale;
        if (targetTemp < 0) {
            targetTemp = 0;
        }
    }
    else if (digitalRead(minusTempBtnPin) == LOW) {
        minuTempBtnState = 0;
    }

    if (digitalRead(plusStepSpeedPin) == HIGH && plusStepSpeedBtnState == 0) {
        plusStepSpeedBtnState = 1;
        stepSpeed += 10;
    }
    else if (digitalRead(plusStepSpeedPin) == LOW) {
        plusStepSpeedBtnState = 0;
    }

    if (digitalRead(minusStepSpeedPin) == HIGH && minusStepSpeedBtnState == 0) {
        minusStepSpeedBtnState = 1;
        stepSpeed -= 10;
        if (stepSpeed < 0) {
            stepSpeed = 0;
        }
    }
    else if (digitalRead(minusStepSpeedPin) == LOW) {
        minusStepSpeedBtnState = 0;
    }

    if (digitalRead(stepControlPin) == HIGH && stepControlBtnState == 0) {
        stepControlBtnState = 1;
        stepControl = stepControl == 0 ? 1 : 0;
    }
    else if (digitalRead(stepControlPin) == LOW) {
        stepControlBtnState = 0;
    }

    //temperature control
    if (currentTemp < targetTemp) {
        digitalWrite(relePin, HIGH);
    } 
    else 
    {
        digitalWrite(relePin, LOW);
    }

}

void loop() {
    ReadTemp();
    ReadButtons();
    StepperControl();
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