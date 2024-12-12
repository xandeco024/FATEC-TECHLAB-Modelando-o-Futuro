#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

#define F_CPU 16000000UL 

//temp pins
const int plusTempBtnPin= A2;
const int scalePin = A1;
const int minusTempBtnPin = A0;

const int heaterPin = 9;

//stepper motor pins
const int stepIn1Pin = 2;
const int stepIn2Pin = 3;
const int stepIn3Pin = 4;
const int stepIn4Pin = 5;
const int plusStepSpeedPin = 6;
const int stepControlPin = 7;
const int minusStepSpeedPin = 8;

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

//regulação temperatura

volatile uint16_t sensorValue = 0;
float temp_sensor = 0;

// Valores do controlador
float Kp = 43.0949;           // Ganho proporcional (ajuste conforme necessário)
float Ki = 18.7504;           // Ganho integral (ajuste conforme necessário)
float Kd = 0.91976;           // Ganho derivativo (ajuste conforme necessário)

float Tensao = 5;
float cv = 0;
float cv1 = 0;
float erro = 0;
float erro1 = 0;
float erro2 = 0;

// Valores para ajuste do Arduino
const int frequenciaPWM = 2000;
const int frequenciaTimer = 1000;
const int prescaler = 8; // Ajustar para obter a frequência desejada
volatile uint16_t valorICR1 = 0;
volatile uint16_t valorOCR1A = 0;
float Tm = 1/frequenciaTimer;

float h1 = Tm/2;
float h2 = 1/Tm;

float b0 = Kp+Ki*h1+Kd*h2;
float b1 = -Kp+Ki*h1-2*Kd*h2;
float b2 = Kd*h2;



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

        //codigo pra juntar 10 leituras e tirar a media, pra reduzir variação
        float readings[50];
        int readingsIndex = 0;

        // bool logging = false;    // Estado de logging (iniciado ou pausado)
        // int loggingInterval = 1000; // Intervalo de logging em ms
        // unsigned long lastLogTime = 0;        // Tempo da última medição
        // unsigned long startTime = 0;          // Tempo de início do logging

    public:
        float currentTemp = 0;

        Thermistor(int inputPin) : pin(inputPin) {}

        double TensionToC(double tension) {
            float resistance = resistorResistance * (voltage / (tension * voltage / adcResolution) - 1);
            double t = 1 / (1 / kelvin25 + log(resistance / resistance25) / thermistorBeta);
            return t - 273.15;
        }

        void Start() {
            pinMode(pin, INPUT);
        }

        void Update() {
            readings[readingsIndex] = analogRead(pin);
            readingsIndex = (readingsIndex + 1) % 50;   

            if (readingsIndex == 0) {
                float sum = 0;
                for (int i = 0; i < 50; i++) {
                    sum += readings[i];
                }
                currentTemp = sum / 50;
            }

            // UpdateLogging();
        }

        // void UpdateLogging() {
        //     if (logging && millis() - lastLogTime > loggingInterval) {
        //         lastLogTime = millis();
        //         float temperature = GetTempC();

        //         // printa a temperatura no tempo (tempo desde que comecou a logar)

        //         Serial.print((millis() - startTime) / 1000);
        //         Serial.print("s | ");
        //         Serial.print(temperature);
        //         Serial.println(" C");
        //     }
        // }

        // void StartLogging() {
        //     logging = true;
        //     lastLogTime = millis(); // Reinicia o contador de tempo
        //     startTime = millis(); // Salva o tempo de início
        //     Serial.println("I");
        // }

        // // Parar a medição
        // void StopLogging() {
        //     logging = false;
        //     Serial.println("P");
        // }
};

Thermistor thermistor(A3); // Inicializando com o pino adequado

void setup() {
    Serial.begin(115200);
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

    pinMode(heaterPin, OUTPUT);

    TCCR1A = (1 << WGM11) | (1 << COM1A1); // Modo Fast PWM, 10-bit, OC1B (pino 9)
    TCCR1B = (1 << WGM12) | (1 << WGM13) | (1 << CS11); // Prescaler de 8

    //TCCR1A = 0b00100010; // COM1A1-COM1A0-COM1B1-COM1B0-X-X-WGM11-WGM10 // Modo Fast PWM, 10-bit, OC1B (pino 9)
    //TCCR1B = 0b00011010; // ICNC1-ICES1-X-WGM13-WGM12-CS12-CS11-CS10 // Prescaler de 8

    valorICR1 = int((F_CPU/prescaler)/frequenciaPWM-1);
    ICR1 = valorICR1;

    TIMSK1 |= (1 << OCIE1A); // Habilita interrupção por comparação A
}

void TempScreen(float currentTemp, float targetTemp, int scale, int stepRPM) {
    lcd.setCursor(0, 0);
    lcd.print("C:");
    lcd.print(currentTemp);
    // lcd.print("C");

    lcd.setCursor(0, 1);
    lcd.print("T:");
    lcd.print(targetTemp);
    // lcd.print("C");

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
        if (millis() - stepSpeedTime > 10) {
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

        // if (stepControl) thermistor.StartLogging();
        // else thermistor.StopLogging();

    } else if (digitalRead(stepControlPin) == LOW) {
        stepControlBtnState = 0;
    }
}

void loop() {
    ReadButtons();
    thermistor.Update();
    TempScreen(analogRead(A3), targetTemp, scale, stepRPM);
    StepperControl();

    Serial.print(targetTemp);
    Serial.print("      ");
    Serial.print(temp_sensor);
    Serial.print("      ");
    Serial.print(cv);
    Serial.print("      ");
    Serial.println(erro);
}

ISR(TIMER1_COMPA_vect){ //TIMER1_COMPA_vect) {
	//----- Cálculo do erro -----

    sensorValue = analogRead(A3);
    temp_sensor = sensorValue * Tensao / 1024.0;

	erro = targetTemp - temp_sensor;
	
	//----- Equação de diferenças ------
	//cv = cv1 + (Kp + Kd/Tm)*erro + (-Kp + Ki*Tm - 2*Kd/Tm)*erro1 + (Kd/Tm)*erro2;
    //cv = cv1 + b0*erro + b1*erro1 + b2*erro2;

    //cv = cv1 + 2034.2445 * erro - -4068.2414 * erro1 + 2034 * erro2; 
    // cv = cv1 + 2034.2445 * erro - -4068.2414 * erro1 + 0 * erro2; 
    cv = cv1 + 1.5 * erro - 1.44356 * erro1 + 0.01 * erro2;

    if (cv > Tensao) cv = Tensao;
    if (cv < 0) cv = 0;

    cv1 = cv;
	erro2 = erro1;
	erro1 = erro;

    valorOCR1A = cv*valorICR1/Tensao;
    OCR1A = valorOCR1A; //map(cv, 0, 250, 0, valorICR1); // Atualiza o duty cycle no OCR1A
}