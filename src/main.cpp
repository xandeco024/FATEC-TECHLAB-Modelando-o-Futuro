#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

#define F_CPU 16000000UL 

// CONSTANTES DE SEGURANÇA PARA PET
const float TEMP_SAFETY_LIMIT = 280.0;      // PET derrete ~250°C, margem de segurança
const float TEMP_EMERGENCY_SHUTDOWN = 320.0; // Desligamento de emergência absoluto
const float TEMP_MIN_VALID = -10.0;         // Temperatura mínima válida do sensor
const float TEMP_MAX_VALID = 350.0;         // Temperatura máxima válida do sensor
const unsigned long TEMP_READ_TIMEOUT = 5000; // 5s sem leitura válida = erro

// Variáveis de segurança
bool systemError = false;
bool heatingEnabled = true;
unsigned long lastValidTempTime = 0;
float lastValidTemp = 25.0;

//new controls
const int potPin = A1;
int potValue = 0;
const int togglePotPin = 11;
int togglePotState = 0;
const int modifyBtnPin = 10;
int modifyBtnState = 0;
const int menuBtnPin = 9;
int menuBtnState = 0;
int menu = 0;
const int toggleBtnPin = 8;
int toggleBtnState = 0;

//stepper motor pins
const int stepDirPin = 2;
const int stepPin = 3;

//stepper variables
const int stepsPerRevolution = 200;
int stepDelay = 5000; // Ajuste o valor para um delay adequado ao motor

int stepState = 0;
int stepControl = 0;
int stepSpeed = 0;
long lastStepTime = 0;

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

    pinMode(potPin, INPUT);
    pinMode(togglePotPin, OUTPUT);
    pinMode(modifyBtnPin, INPUT);
    pinMode(menuBtnPin, INPUT);
    pinMode(toggleBtnPin, INPUT);

    pinMode(stepPin, OUTPUT);
    pinMode(stepDirPin, OUTPUT);
    digitalWrite(stepDirPin, HIGH); // Set the direction of the stepper motor
    digitalWrite(stepPin, LOW); // Ensure the step pin is low initially

    lcd.init();
    lcd.createChar(0, celsiusChar);
    lcd.backlight();

    pinMode(PWM_pin, OUTPUT);

    TCCR0B = TCCR0B & B11111000 | B00000010;    // D5 adn D6 PWM frequency of 7812.50 Hz
    Time = millis();

    digitalWrite(stepDirPin, HIGH); // Set the direction of the stepper motor

    // Set up the stepper motor timer
    cli();
    TCCR1A = 0; // Clear control register A
    TCCR1B = 0; // Clear control register B

    TCCR1B |= (1 << WGM12); // Set CTC mode (WGM12 = 1)

    TCCR1B |= (1 << CS11); // Set prescaler to 8

    OCR1A = F_CPU / (8 * 2 * (1000000 / stepDelay)) - 1;

    TIMSK1 |= (1 << OCIE1A); // Enable compare interrupt
    sei(); // Enable global interrupts
}



void UpdateStepperSpeed(int rpm) {
    if (rpm < 1) rpm = 1; // Evitar divisão por zero
    
    // Calcula o tempo em microssegundos para cada alternância do pino
    unsigned long microsecondsPerStep = (60L * 1000L * 1000L) / (stepsPerRevolution * rpm * 2);
    
    // Ajusta o prescaler e OCR1A com base na velocidade
    unsigned long timerTicks;
    
    cli(); // Desabilita interrupções durante a mudança
    
    if (rpm > 150) {
        // Para velocidades altas, use prescaler de 1
        TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10)); // Limpa bits prescaler
        TCCR1B |= (1 << CS10); // Prescaler 1
        timerTicks = (F_CPU / 1 / (1000000L / microsecondsPerStep)) - 1;
    } else {
        // Para velocidades normais, use prescaler de 8
        TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10)); // Limpa bits prescaler
        TCCR1B |= (1 << CS11); // Prescaler 8
        timerTicks = (F_CPU / 8 / (1000000L / microsecondsPerStep)) - 1;
    }
    
    // Garante valores válidos para OCR1A
    if (timerTicks > 65535) timerTicks = 65535;  // Máximo para timer de 16 bits
    if (timerTicks < 10) timerTicks = 10;        // Mínimo prático
    
    OCR1A = timerTicks;
    
    sei(); // Reabilita interrupções
}

void ReadTemp() {
    int numReadings = 10; 
    float totalTemp = 0.0; 
    int validReadings = 0;

    for (int i = 0; i < numReadings; i++) {
        int read = analogRead(A3);
        
        // Verificar se a leitura ADC está em range válido
        if (read < 10 || read > 1020) {
            continue; // Pula leitura inválida
        }

        // Calculate the Beta value of the thermistor
        float beta = log(RT1 / RT2) / (1 / T1 - 1 / T2);

        // Calculate the thermistor resistance based on the ADC reading
        float thermistorResistance = resistorResistance * (voltage / (read * voltage / adcResolution) - 1);

        // Calculate the temperature in Kelvin based on the thermistor resistance
        double t = 1 / (1 / kelvin25 + log(thermistorResistance / resistance25) / beta);

        // Convert from Kelvin to Celsius
        float tempReading = t - 273.15;
        
        // Verificar se a temperatura está em range válido
        if (tempReading >= TEMP_MIN_VALID && tempReading <= TEMP_MAX_VALID) {
            totalTemp += tempReading;
            validReadings++;
        }
    }

    // Se temos leituras válidas, calcular média
    if (validReadings > 5) { // Precisamos de pelo menos metade das leituras válidas
        currentTemp = totalTemp / validReadings;
        lastValidTemp = currentTemp;
        lastValidTempTime = millis();
    } else {
        // Usar última temperatura válida se não conseguimos leituras confiáveis
        currentTemp = lastValidTemp;
        
        // Se faz muito tempo sem leitura válida, marcar erro
        if (millis() - lastValidTempTime > TEMP_READ_TIMEOUT) {
            systemError = true;
            Serial.println("ERRO: Sensor de temperatura falhou!");
        }
    }
}

int ReadSmooth(int pin){
    int val = 0;
    for(int i = 0; i < 10; i++){
        val += analogRead(pin);
        // delay(10);
    }
    return val/10;
}

void loop() {
    ReadTemp();
    
    // ================ VERIFICAÇÕES DE SEGURANÇA CRÍTICAS ================
    // Verificação de temperatura de emergência
    if (currentTemp > TEMP_EMERGENCY_SHUTDOWN) {
        systemError = true;
        analogWrite(PWM_pin, 0);  // Desligar aquecimento imediatamente
        stepControl = 0;          // Parar motor
        lcd.clear();
        lcd.print("EMERGENCIA!");
        lcd.setCursor(0, 1);
        lcd.print("T>320C PARADO");
        Serial.println("EMERGENCIA: Temperatura muito alta!");
        while(1); // Para o sistema completamente
    }
    
    // Verificação de erro do sistema
    if (systemError) {
        analogWrite(PWM_pin, 0);  // Desligar aquecimento
        stepControl = 0;          // Parar motor
        lcd.clear();
        lcd.print("ERRO SISTEMA");
        lcd.setCursor(0, 1);
        lcd.print("Verificar sensor");
        Serial.println("ERRO: Sistema em modo de segurança");
        delay(1000);
        return; // Não processar mais nada
    }
    
    // Limitar temperatura alvo aos limites seguros para PET
    if (targetTemp > TEMP_SAFETY_LIMIT) {
        targetTemp = TEMP_SAFETY_LIMIT;
    }
    // ===================================================================

    if (digitalRead(modifyBtnPin) == HIGH && modifyBtnState == 0) {
        modifyBtnState = 1;
        Serial.println("Modify");
        if (togglePotState == 0) {
            togglePotState = 1;
        } else {
            togglePotState = 0;
        }
    } else if (digitalRead(modifyBtnPin) == LOW) {
        modifyBtnState = 0;
    }

    if (togglePotState == 1) {
        digitalWrite(togglePotPin, HIGH);
        potValue = constrain(ReadSmooth(potPin), 10, 1010);
        // Serial.println(potValue);
    } else {
        digitalWrite(togglePotPin, LOW);
    }

    if (digitalRead(menuBtnPin) == HIGH && menuBtnState == 0) {
        menuBtnState = 1;
        Serial.println("Menu");
        menu = (menu + 1) % 3;
        lcd.clear();
        togglePotState = 0;
    } else if (digitalRead(menuBtnPin) == LOW) {
        menuBtnState = 0;
    }

    if (digitalRead(toggleBtnPin) == HIGH && toggleBtnState == 0) {
        toggleBtnState = 1;
        Serial.println("Toggle");

        if (stepControl == 0) {
            stepControl = 1;
        } else {
            stepControl = 0;
        }

    } else if (digitalRead(toggleBtnPin) == LOW) {
        toggleBtnState = 0;
    }

    if (menu == 0) {
        lcd.setCursor(0, 0);
        lcd.print("Modelando o");
        lcd.setCursor(0, 1);
        lcd.print("Futuro");

    } else if (menu == 1) {
        if (togglePotState == 1) {
            // MUDANÇA CRÍTICA: Limitar temperatura máxima a 280°C para segurança do PET
            targetTemp = map(potValue, 10, 1010, 0, TEMP_SAFETY_LIMIT);
            targetTemp = constrain(targetTemp, 0, TEMP_SAFETY_LIMIT);
        }

        char tempBuffer[6];
        sprintf(tempBuffer, "%3d", int(currentTemp));

        lcd.setCursor(0, 0);
        lcd.print("T:");
        lcd.print(tempBuffer);
        lcd.write(byte(0));
        lcd.print("/");

        sprintf(tempBuffer, "%3d", int(targetTemp));
        lcd.print(tempBuffer);
        lcd.write(byte(0));
        
        // Mostrar status de aquecimento e avisos
        if (systemError) {
            lcd.print(" ERR");
        } else if (PID_value > 0 && targetTemp > 0) {
            lcd.print(" AQ"); // Aquecendo
        } else {
            lcd.print("   ");
        }

        lcd.setCursor(0, 1);
        if (!togglePotState) {
            if (targetTemp > 270) {
                lcd.print("ALTA TEMP!     ");
            } else {
                lcd.print("Press p/ ajustar");
            }
        } else {
            lcd.print("Press p/ confirm");
        }

    } else if (menu == 2) {
        if (togglePotState == 1) {
            int rpm = map(potValue, 10, 1010, 5, 200);
            rpm = constrain(rpm, 5, 80);
            stepSpeed = rpm;
            
            // Atualiza a velocidade do motor
            UpdateStepperSpeed(rpm);
        }    

        lcd.setCursor(0, 0);
        lcd.print("Motor:");
        lcd.print(stepSpeed);
        lcd.print("rpm ");
        lcd.print(stepControl ? "ON " : "OFF");

        lcd.setCursor(0, 1);
        if (!togglePotState) {
            lcd.print("Press p/ ajustar");
        } else {
            lcd.print("Press p/ confirm");
        }
    }

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
    
    // SEGURANÇA ADICIONAL: Só permitir aquecimento se não há erro e temp alvo > 0
    if (systemError || targetTemp <= 0) {
        PID_value = 0;
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
    if (stepControl) {
        PORTD ^= (1 << PORTD3); // Alterna o pino D3 (stepPin) diretamente
    }
}