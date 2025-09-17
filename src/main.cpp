#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

#define F_CPU 16000000UL 

// CONSTANTES DE SEGURANÇA PARA PET
const float TEMP_SAFETY_LIMIT = 300.0;      // PET derrete ~250°C, margem de segurança
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
int lastStablePotValue = 512; // Valor inicial central
const int togglePotPin = 11;
int togglePotState = 0;

// FILTROS PARA POTENCIÔMETRO COM MAL CONTATO
const int POT_NOISE_THRESHOLD = 15;    // Ignora variações menores que 15
const int POT_FILTER_SAMPLES = 8;      // Média móvel de 8 amostras
const int POT_CHANGE_DELAY = 100;      // 100ms entre mudanças válidas
int potFilterBuffer[POT_FILTER_SAMPLES];
int potFilterIndex = 0;
unsigned long lastPotChangeTime = 0;
bool potFilterInitialized = false;

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
int stepDelay = 10000; // Aumentar delay inicial para menos vibração

// MICROSTEPPING: Configurar baseado no driver
// 1 = full step, 2 = half step, 4 = quarter step, 8 = eighth step, 16 = sixteenth step
const int microstepMultiplier = 16; // 1/16 para máximo silêncio

// NOTA: Microstepping 1/16 - Prós e Contras:
// PRÓS: Muito mais silencioso, movimento mais suave
// CONTRAS: 
// - Torque reduzido (~70% do torque nominal)
// - 16x mais interrupções (mais carga no processador)
// - Precisão posicional pode ser menor devido a não-linearidades
// - Velocidade máxima efetiva menor
// Para extrusora de filamento: geralmente OK pois não precisa de muito torque

int stepState = 0;
int stepControl = 0;
int stepSpeed = 0;
long lastStepTime = 0;

// Novas variáveis para controle suave
volatile bool stepDirection = true;
volatile unsigned long stepCounter = 0;
volatile bool enableStepping = false;

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

    // PRIMEIRA COISA: Configurar pinos do motor para estado seguro
    pinMode(stepPin, OUTPUT);
    pinMode(stepDirPin, OUTPUT);
    digitalWrite(stepPin, LOW);     // Garantir LOW imediatamente
    digitalWrite(stepDirPin, LOW);  // Definir direção
    
    // Pequeno delay para estabilizar
    delay(50);

    pinMode(A3, INPUT);

    pinMode(potPin, INPUT);
    pinMode(togglePotPin, OUTPUT);
    pinMode(modifyBtnPin, INPUT_PULLUP);
    pinMode(menuBtnPin, INPUT_PULLUP);
    pinMode(toggleBtnPin, INPUT_PULLUP);

    lcd.init();
    lcd.createChar(0, celsiusChar);
    lcd.backlight();

    pinMode(PWM_pin, OUTPUT);
    
    // Garantir que PWM está desligado inicialmente
    analogWrite(PWM_pin, 0);

    TCCR0B = TCCR0B & B11111000 | B00000010;    // D5 adn D6 PWM frequency of 7812.50 Hz
    Time = millis();

    // Configuração melhorada do timer para motor
    cli();
    TCCR1A = 0; // Clear control register A
    TCCR1B = 0; // Clear control register B

    TCCR1B |= (1 << WGM12); // Set CTC mode (WGM12 = 1)
    TCCR1B |= (1 << CS11) | (1 << CS10); // Prescaler 64 (mais estável)

    OCR1A = 2000; // Valor inicial conservador

    // NÃO habilitar interrupção ainda - só após tudo configurado
    // TIMSK1 |= (1 << OCIE1A);
    sei(); // Enable global interrupts
    
    // Inicializar variáveis de controle
    stepControl = 0;
    stepSpeed = 0;
    targetTemp = 0;
    
    // Agora sim, habilitar interrupção do timer
    TIMSK1 |= (1 << OCIE1A);
    
    // Delay final para estabilização completa
    delay(100);
}

void UpdateStepperSpeed(int rpm) {
    if (rpm < 1) rpm = 1;
    // Com 1/16 microstepping, limitar velocidade para compensar perda de torque
    if (rpm > 150) rpm = 150; // Reduzido devido ao microstepping
    
    // Calcular com microstepping - 16x mais pulsos!
    unsigned long totalStepsPerRev = stepsPerRevolution * microstepMultiplier; // 200 * 16 = 3200 steps/rev
    unsigned long stepsPerSecond = (totalStepsPerRev * rpm) / 60;
    unsigned long timerFreq = stepsPerSecond * 2;
    
    unsigned long timerTicks;
    
    cli();
    
    // Com 1/16, sempre usar prescaler menor para alta frequência
    TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));
    TCCR1B |= (1 << CS11); // Prescaler 8 (obrigatório para 1/16)
    timerTicks = (F_CPU / 8 / timerFreq) - 1;
    
    if (timerTicks > 65535) timerTicks = 65535;
    if (timerTicks < 50) timerTicks = 50; // Mínimo maior para estabilidade
    
    OCR1A = timerTicks;
    
    sei();
    
    // Debug mostrando carga do sistema
    if (rpm % 20 == 0) {
        Serial.print("RPM: ");
        Serial.print(rpm);
        Serial.print(" | Freq ISR: ");
        Serial.print(timerFreq);
        Serial.print("Hz | Steps/rev: ");
        Serial.print(totalStepsPerRev);
        Serial.print(" | Ticks: ");
        Serial.println(timerTicks);
    }
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

// Função melhorada para ler potenciômetro com filtros
int ReadStablePot(int pin) {
    int rawValue = analogRead(pin);
    
    // Inicializar buffer na primeira leitura
    if (!potFilterInitialized) {
        for (int i = 0; i < POT_FILTER_SAMPLES; i++) {
            potFilterBuffer[i] = rawValue;
        }
        potFilterInitialized = true;
        return rawValue;
    }
    
    // Filtro 1: Rejeitar valores muito fora do esperado (mal contato severo)
    if (abs(rawValue - lastStablePotValue) > 400) {
        return lastStablePotValue; // Manter valor anterior se muito discrepante
    }
    
    // Filtro 2: Média móvel para suavizar ruído
    potFilterBuffer[potFilterIndex] = rawValue;
    potFilterIndex = (potFilterIndex + 1) % POT_FILTER_SAMPLES;
    
    long sum = 0;
    for (int i = 0; i < POT_FILTER_SAMPLES; i++) {
        sum += potFilterBuffer[i];
    }
    int filteredValue = sum / POT_FILTER_SAMPLES;
    
    // Filtro 3: Histerese - só aceita mudanças significativas
    if (abs(filteredValue - lastStablePotValue) < POT_NOISE_THRESHOLD) {
        return lastStablePotValue; // Manter valor se mudança é muito pequena
    }
    
    // Filtro 4: Rate limiting - evita mudanças muito rápidas
    if (millis() - lastPotChangeTime < POT_CHANGE_DELAY) {
        return lastStablePotValue;
    }
    
    // Mudança válida aceita
    lastPotChangeTime = millis();
    lastStablePotValue = filteredValue;
    return filteredValue;
}

int ReadSmooth(int pin){
    // Usar nova função filtrada para o potenciômetro
    if (pin == potPin) {
        return ReadStablePot(pin);
    }
    
    // Para outros pinos, usar método original
    int val = 0;
    for(int i = 0; i < 10; i++){
        val += analogRead(pin);
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

    if (digitalRead(modifyBtnPin) == LOW && modifyBtnState == 0) {
        modifyBtnState = 1;
        Serial.println("Modify");
        if (togglePotState == 0) {
            togglePotState = 1;
        } else {
            togglePotState = 0;
        }
    } else if (digitalRead(modifyBtnPin) == HIGH) {
        modifyBtnState = 0;
    }

    if (togglePotState == 1) {
        digitalWrite(togglePotPin, HIGH);
        
        // Usar função melhorada com filtros
        int rawPot = ReadStablePot(potPin);
        potValue = constrain(rawPot, 10, 1010);
        
        // Debug para monitorar estabilidade
        static unsigned long lastDebugTime = 0;
        if (millis() - lastDebugTime > 500) { // Debug a cada 500ms
            Serial.print("Pot Raw: ");
            Serial.print(analogRead(potPin));
            Serial.print(" | Filtered: ");
            Serial.print(potValue);
            Serial.print(" | Stable: ");
            Serial.println(lastStablePotValue);
            lastDebugTime = millis();
        }
    } else {
        digitalWrite(togglePotPin, LOW);
    }

    if (digitalRead(menuBtnPin) == LOW && menuBtnState == 0) {
        menuBtnState = 1;
        Serial.println("Menu");
        menu = (menu + 1) % 3;
        lcd.clear();
        togglePotState = 0;
    } else if (digitalRead(menuBtnPin) == HIGH) {
        menuBtnState = 0;
    }

    if (digitalRead(toggleBtnPin) == LOW && toggleBtnState == 0) {
        toggleBtnState = 1;
        Serial.println("Toggle");

        if (stepControl == 0) {
            stepControl = 1;
        } else {
            stepControl = 0;
        }

    } else if (digitalRead(toggleBtnPin) == HIGH) {
        toggleBtnState = 0;
    }

    if (menu == 0) {
        lcd.setCursor(0, 0);
        lcd.print("Modelando o");
        lcd.setCursor(0, 1);
        lcd.print("Futuro");

    } else if (menu == 1) {
        if (togglePotState == 1) {
            // Usar valor filtrado diretamente
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
                lcd.print("ALTA TEMP!      ");
            } else {
                lcd.print("Press p/ ajustar");
            }
        } else {
            // Mostrar indicador de estabilidade
            lcd.print("Ajustando... ");
            if (millis() - lastPotChangeTime > 1000) {
                lcd.print("OK");
            } else {
                lcd.print("  ");
            }
        }

    } else if (menu == 2) {
        if (togglePotState == 1) {
            // REMOVIDO LIMITE MÍNIMO: agora vai de 5 a 120 RPM
            int rpm = map(potValue, 10, 1010, 0, 180); // Era 40, agora é 5
            rpm = constrain(rpm, 0, 180);
            stepSpeed = rpm;
            
            UpdateStepperSpeed(rpm);
        }    

        lcd.setCursor(0, 0);
        lcd.print("Motor:");
        lcd.print(stepSpeed);
        lcd.print("rpm");
        
        if (stepSpeed < 100) {
            lcd.print(" ");
        }
        lcd.print(stepControl ? " ON" : "OFF");

        lcd.setCursor(0, 1);
        if (!togglePotState) {
            lcd.print("1/16 SILENT mode");
        } else {
            // Indicador de ajuste estável
            lcd.print("Ajustando... ");
            if (millis() - lastPotChangeTime > 1000) {
                lcd.print("OK");
            } else {
                lcd.print("  ");
            }
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

// ISR agora roda 16x mais vezes! (A 100 RPM = ~5300 Hz)
ISR(TIMER1_COMPA_vect){
    if (stepControl && stepSpeed > 0) {
        static bool stepPhase = false;
        
        // Código mais otimizado para reduzir overhead da ISR
        if (stepPhase) {
            PORTD |= B00001000;   // Mais rápido que (1 << PORTD3)
        } else {
            PORTD &= B11110111;   // Mais rápido que ~(1 << PORTD3)
        }
        
        stepPhase = !stepPhase;
        
        if (stepPhase) {
            stepCounter++;
        }
    } else {
        PORTD &= B11110111;
        stepCounter = 0;
    }
}