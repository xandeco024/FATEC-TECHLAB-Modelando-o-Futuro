#include <Arduino.h>
#include <LiquidCrystal.h>

//pin definition

//input pins
#define plusButtonPin A2
#define scaleButtonPin A1
#define minusButtonPin A0
#define tempSensorPin A3

//output pins
#define relePin 7

//stepper motor pins
#define stepSpeedPin 6
#define stepIn1Pin 2
#define stepIn2Pin 3
#define stepIn3Pin 4
#define stepIn4Pin 5

int plusButtonState = 0;
int scaleButtonState = 0;
int minusButtonState = 0;

LiquidCrystal lcd(8, 9, 10, 11, 12, 13);

float scale = 1;
double currentTemp = 0;
float targetTemp = 0;

//temperature sensor parameters
#define R0 100000
#define B 3950

void setup() {
    pinMode(tempSensorPin, INPUT);
    pinMode(relePin, OUTPUT);

    pinMode(plusButtonPin, INPUT);
    pinMode(scaleButtonPin, INPUT);
    pinMode(minusButtonPin, INPUT);

    pinMode(stepSpeedPin, OUTPUT);
    pinMode(stepIn1Pin, OUTPUT);
    pinMode(stepIn2Pin, OUTPUT);
    pinMode(stepIn3Pin, OUTPUT);
    pinMode(stepIn4Pin, OUTPUT);

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
    currentTemp = analogRead(tempSensorPin); // Ler valor do sensor

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

    //stepper motor control
    digitalWrite(stepSpeedPin, HIGH);

    digitalWrite(stepIn1Pin, HIGH);
    digitalWrite(stepIn2Pin, LOW);
    digitalWrite(stepIn3Pin, LOW);
    digitalWrite(stepIn4Pin, LOW);
    delay(10);

    digitalWrite(stepIn1Pin, LOW);
    digitalWrite(stepIn2Pin, HIGH);
    digitalWrite(stepIn3Pin, LOW);
    digitalWrite(stepIn4Pin, LOW);
    delay(10);

    digitalWrite(stepIn1Pin, LOW);
    digitalWrite(stepIn2Pin, LOW);
    digitalWrite(stepIn3Pin, HIGH);
    digitalWrite(stepIn4Pin, LOW);
    delay(10);

    digitalWrite(stepIn1Pin, LOW);
    digitalWrite(stepIn2Pin, LOW);
    digitalWrite(stepIn3Pin, LOW);
    digitalWrite(stepIn4Pin, HIGH);
    delay(10);
}