#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

//pin definition

//temp pins
#define plusTempBtnPin A2
#define scaleTempBtnPin A1
#define minusTempBtnPin A0
#define tempSensorPin A3
#define relePin 6

//temp coisos
//model NTC 100k 3950
//datasheet https://fab.cba.mit.edu/classes/863.18/CBA/people/erik/reference/11_NTC-3950-100K.pdf
float T1 = 273.15;  // Temperatura 0º C em Kelvin
float T2 = 373.15;  // Temperatura 100º C em Kelvin
float RT1 = 32724.0;  // Resistência do termistor a 0ºC
float RT2 = 671.0;  // Resistência do termistor a 100ºC

// Tensão de alimentação e resolução do ADC
float voltage = 5.0;
float adcResolution = 1024.0;
float resistorResistance = 10000.0;  // Resistência do resistor em série (10k ohms)

// Função para calcular o valor Beta do termistor
float thermistorBeta() {
    return log(RT1 / RT2) / (1 / T1 - 1 / T2);
}

// Função para calcular a resistência do termistor com base na leitura do ADC
float thermistorResistance(float adcReading)
{
    return resistorResistance * (voltage / (adcReading * voltage / adcResolution) - 1);
}

// Função para calcular a temperatura em Celsius com base na resistência do termistor
float thermistorTemperatureC(float thermistorResistance)
{
    const double kelvin25 = 298.15;  // 25ºC em Kelvin
    const double resistance25 = 100000.0;  // Resistência do termistor a 25ºC

    // Cálculo da temperatura com base no valor Beta
    double t = 1 / (1 / kelvin25 + log(thermistorResistance / resistance25) / thermistorBeta());

    return t - 273.15;  // Converte de Kelvin para Celsius
}

//stepper motor pins
//#define stepSpeedPin 6 (directly connected to 5V)
#define stepIn1Pin 2
#define stepIn2Pin 3
#define stepIn3Pin 4
#define stepIn4Pin 5
#define stepControlPin 7
#define plusStepSpeedPin 8
#define minusStepSpeedPin 9

//stepper coisos
//model: Nema sla oq
int stepsPerRotation = 5;
int stepDelay;

int plusTempBtnState = 0;
int scaleTempBtnState = 0;
int minusTempBtnState = 0;

int stepState = 0;
int stepControl = 0;
int stepRPM = 0;
long stepSpeedTime = 0;
int plusStepSpeedBtnState = 0;
int minusStepSpeedBtnState = 0;
int stepControlBtnState = 0;

//LiquidCrystal lcd(8, 9, 10, 11, 12, 13);
LiquidCrystal_I2C lcd(0x27, 16, 2);

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

    lcd.init();
    lcd.backlight();
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
    sprintf(tempStepSpeed, "%3d", stepRPM);
    lcd.print(tempStepSpeed);
    lcd.print("RPM");
}

unsigned long calcStepDelay(int rpm) {
    float SPS = rpm * stepsPerRotation / 60.0;
    return 1/SPS*1000.0;
}


void StepperControl() {

    //stepDelay = calcStepDelay(stepRPM);

    if (stepControl == 1) {
        if (millis() - stepSpeedTime > stepDelay) {
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

void CurveMaker()
{
    //vai verificar a temperatura inicial do hotend, a temperatura máxima que chega, e o tempo que chega, armazenar em um dicionario

    
}

void ReadTemp() {
    //temperature reading with delay but not with the delay function
    if (millis() - tempReadTime > tempReadDelay) {
        tempReadTime = millis();
        currentTemp = analogRead(tempSensorPin);
        currentTemp = thermistorTemperatureC(thermistorResistance(currentTemp));
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

    if (digitalRead(minusTempBtnPin) == HIGH && minusTempBtnState == 0) {
        Serial.print("APERTO");
        minusTempBtnState = 1;
        targetTemp -= scale;
        if (targetTemp < 0) {
            targetTemp = 0;
        }
    }
    else if (digitalRead(minusTempBtnPin) == LOW) {
        minusTempBtnState = 0;
    }

    if (digitalRead(plusStepSpeedPin) == HIGH && plusStepSpeedBtnState == 0) {
        plusStepSpeedBtnState = 1;
        stepRPM += 10;
        if (stepRPM > 999) { //replae with stepper max rpm
            stepRPM = 999;
        }
        stepDelay = calcStepDelay(stepRPM);
    }
    else if (digitalRead(plusStepSpeedPin) == LOW) {
        plusStepSpeedBtnState = 0;
    }

    if (digitalRead(minusStepSpeedPin) == HIGH && minusStepSpeedBtnState == 0) {
        minusStepSpeedBtnState = 1;
        stepRPM -= 10;
        if (stepRPM < 0) {
            stepRPM = 0;
        }
        stepDelay = calcStepDelay(stepRPM);
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


}