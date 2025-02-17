#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

#define F_CPU 16000000UL 

//temp pins
const int plusTempBtnPin= A2;
const int scalePin = A1;
const int minusTempBtnPin = A0;

//const int heaterPin = 5; //antes 9

//stepper motor pins
const int stepIn1Pin = 2;
const int stepIn2Pin = 3;
const int stepIn3Pin = 4;
const int stepIn4Pin = 6;
const int plusStepSpeedPin = 7;
const int stepControlPin = 8;
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

//model NTC 100k 3950
//datasheet https://fab.cba.mit.edu/classes/863.18/CBA/people/erik/reference/11_NTC-3950-100K.pdf
// Constantes do termistor
const float T1 = 273.15;    // Temperatura 0º C em Kelvin
const float T2 = 373.15;    // Temperatura 100º C em Kelvin
const float RT1 = 32724.0;  // Resistência do termistor a 0ºC
const float RT2 = 671.0;    // Resistência do termistor a 100ºC
const float voltage = 5.0;  // Tensão de referência do ADC
const float adcResolution = 1023.0; // Resolução do ADC
const float resistorResistance = 10000.0; // Resistência do resistor em série (10k ohms)
const float kelvin25 = 298.15;  // 25ºC em Kelvin
const float resistance25 = 100000.0;    // Resistência do termistor a 25ºC

int PWM_pin = 5; //Pin for PWM signal to the MOSFET driver (the BJT npn with pullup)
int but1 = 7;
int EN = 2;
int STEP = 3;
int DIR = 4;
int LED = 13;

//Variables
float targetTemp = 0;            //Default temperature setpoint. Leave it 0 and control it with rotary encoder
float currentTemp = 0.0;
float PID_error = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;
float PID_value = 0;
float last_set_temperature = 0;
int max_PWM = 255;

//PID constants
//////////////////////////////////////////////////////////
int kp = 90;   int ki = 30;   int kd = 80;
//////////////////////////////////////////////////////////

int PID_p = 0;    int PID_i = 0;    int PID_d = 0;
float last_kp = 0;
float last_ki = 0;
float last_kd = 0;

int PID_values_fixed =0;

//prettify :D

byte celsiusChar[] = {
    B00001,
    B01100,
    B10010,
    B10000,
    B10000,
    B10010,
    B01100,
    B00000
};

void setup() {
    Serial.begin(115200);

    pinMode(A3, INPUT);

    pinMode(plusTempBtnPin, INPUT);
    pinMode(scalePin, INPUT);
    pinMode(minusTempBtnPin, INPUT);

    pinMode(stepIn1Pin, OUTPUT);
    pinMode(stepIn2Pin, OUTPUT);
    pinMode(stepIn3Pin, OUTPUT);
    pinMode(stepIn4Pin, OUTPUT);

    lcd.init();
    lcd.createChar(0, celsiusChar);
    lcd.backlight();

    pinMode(PWM_pin, OUTPUT);

    TCCR0B = TCCR0B & B11111000 | B00000010;    // D5 adn D6 PWM frequency of 7812.50 Hz
    Time = millis();

    TCCR1A = 0;             //Reset entire TCCR1A register
    TCCR1B = 0;             //Reset entire TCCR1B register
    TCCR1A |= B00000010;    //   /8
    TCNT1 = 0;              //Reset Timer 1 value to 0
}

void TempScreen() {
    char tempBuffer[6]; // Buffer to hold formatted temperature (including null terminator)
    sprintf(tempBuffer, "%3d", int(currentTemp)); // Format the integer with leading spaces

    lcd.setCursor(0, 0);
    lcd.print("C:");
    lcd.print(tempBuffer);
    lcd.write(byte(0));

    lcd.setCursor(0, 1);
    lcd.print("T:");
    sprintf(tempBuffer, "%3d", int(targetTemp)); // Format targetTemp the same way
    lcd.print(tempBuffer);
    lcd.write(byte(0));

    lcd.setCursor(8, 0);
    lcd.print("S:");
    sprintf(tempBuffer, "%3d", scale); // Format scale the same way
    lcd.print(tempBuffer);

    lcd.setCursor(8, 1);
    lcd.print("RPM:");
    sprintf(tempBuffer, "%3d", stepRPM); // Format stepRPM the same way
    lcd.print(tempBuffer);
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
        targetTemp = min(targetTemp, 400);
        Serial.println("+");
    } else if (digitalRead(plusTempBtnPin) == LOW) {
        plusTempBtnState = 0;
    }

    if (digitalRead(minusTempBtnPin) == HIGH && minusTempBtnState == 0) {
        minusTempBtnState = 1;
        targetTemp -= scale;
        targetTemp = max(targetTemp, 0);
        Serial.println("-");
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

void ReadTemp() {
    int numReadings = 50; // Number of readings to average
    float totalTemp = 0.0; // Variable to store the sum of all readings

    for (int i = 0; i < numReadings; i++) {
        int read = analogRead(A3);

        // Calculate the Beta value of the thermistor
        float beta = log(RT1 / RT2) / (1 / T1 - 1 / T2);

        // Calculate the thermistor resistance based on the ADC reading
        float thermistorResistance = resistorResistance * (voltage / (read * voltage / adcResolution) - 1);

        // Calculate the temperature in Kelvin based on the thermistor resistance
        double t = 1 / (1 / kelvin25 + log(thermistorResistance / resistance25) / beta);

        // Convert from Kelvin to Celsius and add to the total
        totalTemp += (t - 273.15);
    }

    // Calculate the average temperature
    currentTemp = totalTemp / numReadings;
}

void loop() {
    ReadButtons();
    StepperControl();
    ReadTemp();
    TempScreen();

    //Next we calculate the error between the setpoint and the real value
    PID_error = targetTemp - currentTemp + 6;
    //Calculate the P value
    PID_p = 0.01*kp * PID_error;
    //Calculate the I value in a range on +-6
    PID_i = 0.01*PID_i + (ki * PID_error);
    

    //For derivative we need real time to calculate speed change rate
    timePrev = Time;                            // the previous time is stored before the actual time read
    Time = millis();                            // actual time read
    elapsedTime = (Time - timePrev) / 1000; 
    //Now we can calculate the D calue
    PID_d = 0.01*kd*((PID_error - previous_error)/elapsedTime);
    //Final total PID value is the sum of P + I + D
    PID_value = PID_p + PID_i + PID_d;

    
    //We define PWM range between 0 and 255
    if(PID_value < 0){
        PID_value = 0;
    }
    if(PID_value > max_PWM){
        PID_value = max_PWM;
    }
    
    //Now we can write the PWM signal to the mosfet on digital pin D5
    analogWrite(PWM_pin,PID_value);
    previous_error = PID_error;     //Remember to store the previous error for next loop.
    
    Serial.print(targetTemp);
    Serial.print("      ");
    Serial.print(currentTemp);
    Serial.print("      ");
    Serial.print(PID_value);
    Serial.print("      ");
    Serial.println(PID_error);
}

ISR(TIMER1_COMPA_vect){
    TCNT1  = 0;                  //First, set the timer back to 0 so it resets for next interrupt
}