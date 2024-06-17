#include <Arduino.h>
#include <LiquidCrystal.h>

//pin definition
#define plusButtonPin 2
#define scaleButtonPin 3
#define minusButtonPin 4
#define tempSensorPin A5
#define relePin 5
#define buzzerPin 6

int plusButtonState = 0;
int scaleButtonState = 0;
int minusButtonState = 0;

LiquidCrystal lcd(8, 9, 10, 11, 12, 13);

float scale = 1;
float currentTemp = 0;
float targetTemp = 0;

void setup() {
    pinMode(tempSensorPin, INPUT);
    pinMode(relePin, OUTPUT);

    pinMode(plusButtonPin, INPUT);
    pinMode(scaleButtonPin, INPUT);
    pinMode(minusButtonPin, INPUT);

    lcd.begin(16, 2);
    Serial.begin(9600);
}

void TempScreen() {
    //screen handling

    lcd.setCursor(0, 0);
    lcd.print("Cur:");
    //print the current temp in the following format: 999.9C
    lcd.print(currentTemp, 1);
    lcd.print("C   ");

    lcd.setCursor(0, 1);
    lcd.print("Tar:");
    lcd.print(targetTemp, 1);
    lcd.print("C   ");

    lcd.setCursor(12, 1);
    //pint the scale with 1 decimal
    lcd.print("S");
    lcd.print(scale, 1);
}

void loop() {
    //temperature reading
    currentTemp = analogRead(tempSensorPin);

    //button handling
    if (digitalRead(scaleButtonPin) == HIGH && scaleButtonState == 0) {
        scaleButtonState = 1;

        if (scale == 1) {
            scale = 10;
        } 
        else if (scale == 10) {
            scale = 0.1;
        } 
        else if (scale == 0.1) {
            scale = 1;
        }
    }
    else if (digitalRead(scaleButtonPin) == LOW) {
        scaleButtonState = 0;
    }

    if (digitalRead(plusButtonPin) == HIGH && plusButtonState == 0) {
        plusButtonState = 1;
        targetTemp += scale;
    }
    else if (digitalRead(plusButtonPin) == LOW) {
        plusButtonState = 0;
    }

    if (digitalRead(minusButtonPin) == HIGH && minusButtonState == 0) {
        minusButtonState = 1;
        targetTemp -= scale;
        if (targetTemp < 0) {
            targetTemp = 0;
        }
    }
    else if (digitalRead(minusButtonPin) == LOW) {
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
}