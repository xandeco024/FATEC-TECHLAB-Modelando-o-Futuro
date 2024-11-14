#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

//pin definition

//temp pins
const int plusTempBtnPin = A2;
const int scalePin = A1;
const int minusTempBtnPin = A0;

//stepper motor pins
const int stepIn1Pin = 2;
const int stepIn2Pin = 3;
const int stepIn3Pin = 4;
const int stepIn4Pin = 5;
const int stepControlPin = 7;
const int plusStepSpeedPin = 8;
const int minusStepSpeedPin = 9;

//stepper variables
int stepsPerRotation = 5;
int stepDelay = 200; // Ajuste o valor para um delay adequado ao motor

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

LiquidCrystal_I2C lcd(0x27, 16, 2);

int scale = 1;
int targetTemp = 0;

// Classe Thermistor com construtor atualizado para receber pino
class Thermistor {
     //model NTC 100k 3950
    //datasheet https://fab.cba.mit.edu/classes/863.18/CBA/people/erik/reference/11_NTC-3950-100K.pdf

    private: 
        int pin;
        const float T1 = 273.15;  // Temperatura 0º C em Kelvin
        const float T2 = 373.15;  // Temperatura 100º C em Kelvin
        const float RT1 = 32724.0;  // Resistência do termistor a 0ºC
        const float RT2 = 671.0;  // Resistência do termistor a 100ºC
        const float voltage = 5.0;
        const float adcResolution = 1024.0;
        const float resistorResistance = 10000.0;  // Resistência do resistor em série (10k ohms)
        const float thermistorBeta = log(RT1 / RT2) / (1 / T1 - 1 / T2);
        const float kelvin25 = 298.15;  // 25ºC em Kelvin
        const float resistance25 = 100000.0;  // Resistência do termistor a 25ºC

        unsigned long readingInterval = 1000;
        unsigned long lastReadTime = 0;

        // Armazenamento de dados
        struct DataPoint {
            float time; // Tempo em segundos
            float temperature;  // Temperatura em graus Celsius
        };

        DataPoint dataLog[180]; // Array de armazenamento
        int logIndex = 0;       // Índice atual no array de dados
        bool logging = false;    // Estado de logging (iniciado ou pausado)
        int loggingInterval = 1000; // Intervalo de logging em ms
        unsigned long lastLogTime = 0;        // Tempo da última medição

    public:
        float currentTemp = 0;

        Thermistor(int inputPin) : pin(inputPin) {}

        double GetTempF() {
            float resistance = resistorResistance * (voltage / (analogRead(pin) * voltage / adcResolution) - 1);
            double t = 1 / (1 / kelvin25 + log(resistance / resistance25) / thermistorBeta);
            return t;
        }

        double GetTempC() {
            return GetTempF() - 273.15;
        }

        void Start() {
            pinMode(pin, INPUT);
        }

        void Update() {
            if (millis() - lastReadTime > readingInterval) {
                lastReadTime = millis();
                currentTemp = GetTempC();
            }

            UpdateLogging();
        }

        void UpdateLogging() {
            if (logging && millis() - lastLogTime > loggingInterval) {
                lastLogTime = millis();
                float temperature = GetTempC();

                // Armazena o ponto de dados no array, com tempo em segundos
                dataLog[logIndex].time = logIndex;
                dataLog[logIndex].temperature = temperature;

                Serial.print(dataLog[logIndex].time);
                Serial.print(" s | ");
                Serial.print(dataLog[logIndex].temperature);
                Serial.println(" C");

                logIndex++; // Avança para o próximo índice
                if (logIndex >= 180) 
                {
                    StopLogging();
                }
            }
        }

        void StartLogging() {
            logging = true;
            logIndex = 0; // Reinicia o índice
            lastLogTime = millis(); // Reinicia o contador de tempo
            Serial.println("I");
        }

        // Parar a medição
        void StopLogging() {
            logging = false;
            Serial.println("P");
        }
};

Thermistor thermistor(A3); // Inicializando com o pino adequado

void setup() {
    thermistor.Start();

    pinMode(plusTempBtnPin, INPUT);
    pinMode(scalePin, INPUT);
    pinMode(minusTempBtnPin, INPUT);

    pinMode(stepIn1Pin, OUTPUT);
    pinMode(stepIn2Pin, OUTPUT);
    pinMode(stepIn3Pin, OUTPUT);
    pinMode(stepIn4Pin, OUTPUT);

    lcd.init();
    lcd.backlight();
    Serial.begin(9600);
}

void TempScreen(float currentTemp, float targetTemp, int scale, int stepRPM) {
    lcd.setCursor(0, 0);
    lcd.print("C:");
    lcd.print((int)currentTemp);
    lcd.print("C");

    lcd.setCursor(0, 1);
    lcd.print("T:");
    lcd.print(int(targetTemp));
    lcd.print("C");

    lcd.setCursor(8, 0);
    lcd.print("S:");
    lcd.print(scale);

    lcd.setCursor(8, 1);
    lcd.print("RPM:");
    lcd.print(int(stepRPM));
}

unsigned long calcStepDelay(int rpm) {
    float SPS = rpm * stepsPerRotation / 60.0;
    return SPS > 0 ? 1 / SPS * 1000.0 : 1000;
}

void StepperControl() {
    if (stepControl == 1) {
        if (millis() - stepSpeedTime > stepDelay) {
            stepSpeedTime = millis();
            // Controle dos estados do motor de passo
            switch (stepState) {
                case 0: digitalWrite(stepIn1Pin, HIGH); digitalWrite(stepIn2Pin, LOW); digitalWrite(stepIn3Pin, LOW); digitalWrite(stepIn4Pin, LOW); stepState = 1; break;
                case 1: digitalWrite(stepIn1Pin, LOW); digitalWrite(stepIn2Pin, HIGH); digitalWrite(stepIn3Pin, LOW); digitalWrite(stepIn4Pin, LOW); stepState = 2; break;
                case 2: digitalWrite(stepIn1Pin, LOW); digitalWrite(stepIn2Pin, LOW); digitalWrite(stepIn3Pin, HIGH); digitalWrite(stepIn4Pin, LOW); stepState = 3; break;
                case 3: digitalWrite(stepIn1Pin, LOW); digitalWrite(stepIn2Pin, LOW); digitalWrite(stepIn3Pin, LOW); digitalWrite(stepIn4Pin, HIGH); stepState = 0; break;
            }
        }
    }
}

void ReadButtons() {
    if (digitalRead(scalePin) == HIGH && scaleTempBtnState == 0) {
        scaleTempBtnState = 1;
        scale = (scale == 1) ? 10 : (scale == 10) ? 100 : 1;
    } else if (digitalRead(scalePin) == LOW) {
        scaleTempBtnState = 0;
    }

    if (digitalRead(plusTempBtnPin) == HIGH && plusTempBtnState == 0) {
        plusTempBtnState = 1;
        targetTemp += scale;
        targetTemp = min(targetTemp, 999);
    } else if (digitalRead(plusTempBtnPin) == LOW) {
        plusTempBtnState = 0;
    }

    if (digitalRead(minusTempBtnPin) == HIGH && minusTempBtnState == 0) {
        minusTempBtnState = 1;
        targetTemp -= scale;
        targetTemp = max(targetTemp, 0);
    } else if (digitalRead(minusTempBtnPin) == LOW) {
        minusTempBtnState = 0;
    }

    if (digitalRead(plusStepSpeedPin) == HIGH && plusStepSpeedBtnState == 0) {
        plusStepSpeedBtnState = 1;
        stepRPM += scale;
        stepRPM = min(stepRPM, 999);
        stepDelay = calcStepDelay(stepRPM);
    } else if (digitalRead(plusStepSpeedPin) == LOW) {
        plusStepSpeedBtnState = 0;
    }

    if (digitalRead(minusStepSpeedPin) == HIGH && minusStepSpeedBtnState == 0) {
        minusStepSpeedBtnState = 1;
        stepRPM -= scale;
        stepRPM = max(stepRPM, 0);
        stepDelay = calcStepDelay(stepRPM);
    } else if (digitalRead(minusStepSpeedPin) == LOW) {
        minusStepSpeedBtnState = 0;
    }

    if (digitalRead(stepControlPin) == HIGH && stepControlBtnState == 0) {
        stepControlBtnState = 1;
        stepControl = !stepControl;

        if (stepControl) thermistor.StartLogging();
        else thermistor.StopLogging();

    } else if (digitalRead(stepControlPin) == LOW) {
        stepControlBtnState = 0;
    }
}

void loop() {
    ReadButtons();
    thermistor.Update();
    TempScreen(thermistor.currentTemp, targetTemp, scale, stepRPM);
    StepperControl();
}