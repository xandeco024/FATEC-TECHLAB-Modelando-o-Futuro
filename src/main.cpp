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

// Controle do cooler (ventilador)
const int fanRelayPin = 7;
const float FAN_ON_THRESHOLD = -5.0;    // Liga quando 5°C acima do alvo
const float FAN_OFF_THRESHOLD = -2.0;   // Desliga quando chegar a 2°C acima do alvo (histerese)
const float FAN_EMERGENCY_TEMP = 280.0; // Liga em emergência se passar de 280°C
bool fanState = false;

// Controles antigos (potenciômetro) removidos
/*
const int potPin = A1;
int potValue = 0;
int lastStablePotValue = 512;
const int togglePotPin = 11;
int togglePotState = 0;
*/

// Filtros para potenciômetro removidos
/*
const int POT_NOISE_THRESHOLD = 15;
const int POT_FILTER_SAMPLES = 8;
const int POT_CHANGE_DELAY = 100;
int potFilterBuffer[POT_FILTER_SAMPLES];
int potFilterIndex = 0;
unsigned long lastPotChangeTime = 0;
bool potFilterInitialized = false;
*/

// Novos botões de controle
const int minusBtnPin = 12;
const int scaleBtnPin = 11;
const int plusBtnPin = 10; // Substitui o antigo modifyBtnPin

int minusBtnState = 0;
int scaleBtnState = 0;
int plusBtnState = 0;

int tempScale = 10;
int motorScale = 10;

const int menuBtnPin = 9;
int menuBtnState = 0;
int menu = 0;
const int toggleBtnPin = 8;
int toggleBtnState = 0;

// Automação
enum AutoState {
    AUTO_IDLE,
    AUTO_CONFIRM_PREHEAT,    // Aguardando confirmação para aquecer
    AUTO_PREHEATING,         // Aquecendo
    AUTO_READY_TO_EXTRUDE,   // Pronto, aguardando confirmação para extrusar
    AUTO_RUNNING,            // Extrusando
    AUTO_PAUSED_TEMP,        // Pausado por temperatura baixa
    AUTO_STOPPED             // Parado (transitório)
};
AutoState autoState = AUTO_IDLE;

// Constantes para temperatura
const float AUTO_TARGET_TEMP = 210.0;      // Temperatura alvo máxima
const float TEMP_MIN_EXTRUSION = 200.0;    // Abaixo disso, não extrudar
const float TEMP_IDEAL = 210.0;            // Temperatura ideal de operação

// Constantes para curva de velocidade dinâmica (baseada em temperatura)
const int RPM_AT_IDEAL = 55;               // RPM na temperatura ideal (210°C)
const int RPM_AT_MAX_TEMP = 60;            // RPM na temperatura máxima (   )

//stepper motor pins
const int stepDirPin = 2;
const int stepPin = 3;
const int stepEnablePin = 6;  // Pino ENABLE para economia de energia

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
volatile bool stepDirection = true; // false = CCW (LOW), true = CW (HIGH) - INVERTIDO!
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
// int but1 = 7; // REMOVIDO - Pino 7 agora é fanRelayPin
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
    pinMode(stepEnablePin, OUTPUT);

    digitalWrite(stepPin, LOW);     // Garantir LOW imediatamente
    digitalWrite(stepDirPin, stepDirection ? HIGH : LOW);  // Definir direção baseada em stepDirection
    digitalWrite(stepEnablePin, HIGH);  // Motor DESABILITADO inicialmente (economia de energia)

    // Pequeno delay para estabilizar
    delay(50);

    pinMode(A3, INPUT);

    // Pinos do potenciômetro removidos
    // pinMode(potPin, INPUT);
    // pinMode(togglePotPin, OUTPUT);

    // Configuração dos novos botões
    pinMode(plusBtnPin, INPUT_PULLUP);
    pinMode(scaleBtnPin, INPUT_PULLUP);
    pinMode(minusBtnPin, INPUT_PULLUP);
    
    pinMode(menuBtnPin, INPUT_PULLUP);
    pinMode(toggleBtnPin, INPUT_PULLUP);

    lcd.init();
    lcd.createChar(0, celsiusChar);
    lcd.backlight();

    pinMode(PWM_pin, OUTPUT);

    // Configurar pino do cooler (relé NO - Normally Open)
    pinMode(fanRelayPin, OUTPUT);
    digitalWrite(fanRelayPin, LOW);  // Cooler desligado inicialmente

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

    // NÃO habilitar ISR aqui - será habilitada apenas quando motor ligar
    // Isso economiza CPU quando motor está parado
    // TIMSK1 |= (1 << OCIE1A); // ← Removido! Agora controlado por SetMotorEnabled()

    // Delay final para estabilização completa
    delay(100);
}

void UpdateStepperDirection() {
    // Atualiza o pino de direção baseado na variável stepDirection
    digitalWrite(stepDirPin, stepDirection ? HIGH : LOW);
    Serial.print("Direção do motor: ");
    Serial.println(stepDirection ? "CW (HIGH)" : "CCW (LOW)");
}

void SetMotorEnabled(bool enable) {
    // Controle inteligente do pino ENABLE para economia de energia
    if (enable) {
        // Habilita motor: ENABLE = LOW (driver ativo)
        digitalWrite(stepEnablePin, LOW);
        // Habilita interrupção do timer para gerar pulsos
        TIMSK1 |= (1 << OCIE1A);
        Serial.println("Motor HABILITADO (bobinas energizadas)");
    } else {
        // Desabilita motor: ENABLE = HIGH (driver desligado)
        digitalWrite(stepEnablePin, HIGH);
        // Desabilita interrupção do timer para economizar CPU
        TIMSK1 &= ~(1 << OCIE1A);
        // Garante que o pino STEP está em LOW
        digitalWrite(stepPin, LOW);
        Serial.println("Motor DESABILITADO (economia ~1.5A)");
    }
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

void ControlCooler() {
    // SEGURANÇA: Modo de emergência - liga cooler se temperatura muito alta
    if (currentTemp >= FAN_EMERGENCY_TEMP) {
        if (!fanState) {
            digitalWrite(fanRelayPin, HIGH);
            fanState = true;
            Serial.println("FAN EMERGÊNCIA - Temperatura crítica!");
        }
        return; // Não processa lógica normal em emergência
    }

    // Se sistema em erro ou sem aquecimento (targetTemp = 0), desliga cooler
    if (systemError || targetTemp <= 0) {
        if (fanState) {
            digitalWrite(fanRelayPin, LOW);
            fanState = false;
            Serial.println("FAN DESLIGADO - Sistema em standby/erro");
        }
        return;
    }

    // LÓGICA SIMPLES E EFICAZ:
    // Liga cooler quando temperatura está ACIMA do alvo (resfriamento ativo)
    // Desliga quando está ABAIXO do alvo (permite aquecimento)

    // DEBUG detalhado (temporário)
    static unsigned long lastDebugTime = 0;
    if (millis() - lastDebugTime > 2000) {
        Serial.print("DEBUG FAN - Erro: ");
        Serial.print(PID_error);
        Serial.print(" | Thresh ON: ");
        Serial.print(FAN_ON_THRESHOLD);
        Serial.print(" | Thresh OFF: ");
        Serial.print(FAN_OFF_THRESHOLD);
        Serial.print(" | Estado: ");
        Serial.println(fanState ? "ON" : "OFF");
        lastDebugTime = millis();
    }

    // Liga o cooler se temperatura está 5°C+ acima do alvo
    if (PID_error < FAN_ON_THRESHOLD && !fanState) {
        digitalWrite(fanRelayPin, HIGH);
        fanState = true;
        Serial.print(">>> FAN LIGADO - Temp muito alta: ");
        Serial.print(currentTemp);
        Serial.print("°C (alvo: ");
        Serial.print(targetTemp);
        Serial.println("°C)");
    }
    // Desliga quando temperatura voltar próxima do alvo (histerese)
    // Isso permite que o aquecedor trabalhe normalmente
    else if (PID_error > FAN_OFF_THRESHOLD && fanState) {
        digitalWrite(fanRelayPin, LOW);
        fanState = false;
        Serial.print(">>> FAN DESLIGADO - Temperatura normalizada: ");
        Serial.print(currentTemp);
        Serial.println("°C");
    }
}

// Função ReadStablePot removida pois não é mais necessária
// Função ReadSmooth removida pois não é mais necessária

void loop() {
    ReadTemp();
    
    // ================ VERIFICAÇÕES DE SEGURANÇA CRÍTICAS ================
    // Verificação de temperatura de emergência
    if (currentTemp > TEMP_EMERGENCY_SHUTDOWN) {
        systemError = true;
        analogWrite(PWM_pin, 0);  // Desligar aquecimento imediatamente
        digitalWrite(fanRelayPin, HIGH);  // LIGAR cooler em emergência
        stepControl = 0;          // Parar motor
        lcd.clear();
        lcd.print("EMERGENCIA!");
        lcd.setCursor(0, 1);
        lcd.print("T>320C PARADO");
        Serial.println("EMERGENCIA: Temperatura muito alta! FAN LIGADO!");
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

    // Lógica antiga do potenciômetro e botão de modificação removida

    // Leitura dos novos botões de controle
    if (digitalRead(plusBtnPin) == LOW && plusBtnState == 0) {
        plusBtnState = 1;
        Serial.println("Plus");
        if (menu == 1) {
            targetTemp += tempScale;
        } else if (menu == 2) {
            stepSpeed += motorScale;
        }
    } else if (digitalRead(plusBtnPin) == HIGH) {
        plusBtnState = 0;
    }

    if (digitalRead(minusBtnPin) == LOW && minusBtnState == 0) {
        minusBtnState = 1;
        Serial.println("Minus");
        if (menu == 1) {
            targetTemp -= tempScale;
        } else if (menu == 2) {
            stepSpeed -= motorScale;
        }
    } else if (digitalRead(minusBtnPin) == HIGH) {
        minusBtnState = 0;
    }

    if (digitalRead(scaleBtnPin) == LOW && scaleBtnState == 0) {
        scaleBtnState = 1;
        Serial.println("Scale");
        if (menu == 1) {
            if (tempScale == 1) tempScale = 10;
            else if (tempScale == 10) tempScale = 100;
            else tempScale = 1;
        } else if (menu == 2) {
            if (motorScale == 1) motorScale = 10;
            else if (motorScale == 10) motorScale = 100;
            else motorScale = 1;
        }
    } else if (digitalRead(scaleBtnPin) == HIGH) {
        scaleBtnState = 0;
    }

    if (digitalRead(menuBtnPin) == LOW && menuBtnState == 0) {
        menuBtnState = 1;
        Serial.println("Menu");

        // Se estava no modo auto, apenas reseta o estado (mantém temp e motor)
        if (menu == 3) {
            autoState = AUTO_IDLE;
            Serial.println("Saiu do modo Auto - Estado resetado");
        }

        menu = (menu + 1) % 4; // Agora são 4 menus (0, 1, 2, 3)
        lcd.clear();
    } else if (digitalRead(menuBtnPin) == HIGH) {
        menuBtnState = 0;
    }

    if (digitalRead(toggleBtnPin) == LOW && toggleBtnState == 0) {
        toggleBtnState = 1;
        Serial.println("Toggle");

        if (menu == 2) { // Controle manual do motor
            if (stepControl == 0) {
                UpdateStepperDirection(); // Atualiza direção antes de ligar
                SetMotorEnabled(true);    // Habilita motor (ISR + ENABLE pin)
                stepControl = 1;
            } else {
                SetMotorEnabled(false);   // Desabilita motor (economia de energia)
                stepControl = 0;
            }
        } else if (menu == 3) { // Controle da automação
            if (autoState == AUTO_IDLE) {
                autoState = AUTO_CONFIRM_PREHEAT; // Pede confirmação primeiro
            } else if (autoState == AUTO_CONFIRM_PREHEAT) {
                autoState = AUTO_PREHEATING; // Confirmou, começa a aquecer
            } else if (autoState == AUTO_READY_TO_EXTRUDE) {
                autoState = AUTO_RUNNING; // Confirmou, começa a extrusar
                stepSpeed = 5; // Inicia o motor devagar
                UpdateStepperDirection(); // Atualiza direção antes de ligar
                SetMotorEnabled(true);    // Habilita motor (ISR + ENABLE pin)
                stepControl = 1;
            } else {
                autoState = AUTO_STOPPED; // Para o processo se estiver rodando
            }
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
        // A lógica de ajuste agora é feita pelos botões +, - e escala
        targetTemp = constrain(targetTemp, 0, TEMP_SAFETY_LIMIT);

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
        
        // Mostrar status de aquecimento, resfriamento e avisos
        if (systemError) {
            lcd.print(" ERR");
        } else if (fanState) {
            lcd.print(" FAN"); // Cooler ligado
        } else if (PID_value > 0 && targetTemp > 0) {
            lcd.print(" AQ"); // Aquecendo
        } else {
            lcd.print("   ");
        }

        lcd.setCursor(0, 1);
        lcd.print("Escala: ");
        char scaleBuffer[4];
        sprintf(scaleBuffer, "%3d", tempScale);
        lcd.print(scaleBuffer);
        lcd.print(" C ");

    } else if (menu == 2) {
        // A lógica de ajuste agora é feita pelos botões +, - e escala
        stepSpeed = constrain(stepSpeed, 0, 180);
        UpdateStepperSpeed(stepSpeed);

        lcd.setCursor(0, 0);
        lcd.print("Motor:");
        char speedBuffer[4];
        sprintf(speedBuffer, "%3d", stepSpeed);
        lcd.print(speedBuffer);
        lcd.print("rpm");
        
        lcd.print(stepControl ? " ON" : "OFF");

        lcd.setCursor(0, 1);
        lcd.print("Escala: ");
        char scaleBuffer[4];
        sprintf(scaleBuffer, "%3d", motorScale);
        lcd.print(scaleBuffer);
        lcd.print("   ");
    } else if (menu == 3) { // Novo menu de Automação
        switch (autoState) {
            case AUTO_IDLE:
                // Não mexe em nada, só mostra a tela
                lcd.setCursor(0, 0);
                lcd.print("Auto: Parado    "); // 16 chars
                lcd.setCursor(0, 1);
                lcd.print("Press p/Iniciar "); // 16 chars
                break;

            case AUTO_CONFIRM_PREHEAT:
                // Ainda não aquece
                lcd.setCursor(0, 0);
                lcd.print("Aquecer a ");      // 10 chars
                lcd.print((int)AUTO_TARGET_TEMP); // +3 = 13
                lcd.write(byte(0));           // +1 = 14
                lcd.print("? ");              // +2 = 16
                lcd.setCursor(0, 1);
                lcd.print("Press=S Menu=N  "); // 16 chars
                break;

            case AUTO_PREHEATING:
                targetTemp = AUTO_TARGET_TEMP;
                stepControl = 0;
                stepSpeed = 0;
                UpdateStepperSpeed(0);

                lcd.setCursor(0, 0);
                lcd.print("Aquecendo...    "); // 16 chars
                lcd.setCursor(0, 1);
                lcd.print("T:");              // 2 chars
                lcd.print((int)currentTemp);  // +3 = 5
                lcd.write(byte(0));           // +1 = 6
                lcd.print("/");               // +1 = 7
                lcd.print((int)AUTO_TARGET_TEMP); // +3 = 10
                lcd.write(byte(0));           // +1 = 11
                lcd.print("     ");           // +5 = 16

                if (currentTemp >= AUTO_TARGET_TEMP) {
                    autoState = AUTO_READY_TO_EXTRUDE;
                }
                break;

            case AUTO_READY_TO_EXTRUDE:
                targetTemp = AUTO_TARGET_TEMP;
                stepControl = 0;
                stepSpeed = 0;
                UpdateStepperSpeed(0);

                lcd.setCursor(0, 0);
                lcd.print("Pronto! T:");     // 10 chars
                lcd.print((int)currentTemp); // +3 = 13
                lcd.write(byte(0));          // +1 = 14
                lcd.print("  ");             // +2 = 16
                lcd.setCursor(0, 1);
                lcd.print("Press=Extrusar  "); // 16 chars
                break;

            case AUTO_RUNNING:
                targetTemp = AUTO_TARGET_TEMP;

                // Habilita motor apenas na primeira vez que entra neste estado
                static bool autoRunningMotorEnabled = false;
                if (!autoRunningMotorEnabled) {
                    UpdateStepperDirection(); // Garante que a direção está correta
                    SetMotorEnabled(true);    // Habilita motor
                    autoRunningMotorEnabled = true;
                }
                stepControl = 1;

                // ============ CURVA DE VELOCIDADE DINÂMICA ============
                if (currentTemp < TEMP_MIN_EXTRUSION) {
                    autoState = AUTO_PAUSED_TEMP;
                    stepSpeed = 0;
                    autoRunningMotorEnabled = false; // Reset para próxima vez
                } else if (currentTemp < TEMP_IDEAL) {
                    stepSpeed = map(currentTemp, TEMP_MIN_EXTRUSION, TEMP_IDEAL, 0, RPM_AT_IDEAL);
                } else if (currentTemp < AUTO_TARGET_TEMP) {
                    stepSpeed = map(currentTemp, TEMP_IDEAL, AUTO_TARGET_TEMP, RPM_AT_IDEAL, RPM_AT_MAX_TEMP);
                } else {
                    stepSpeed = RPM_AT_MAX_TEMP;
                }

                stepSpeed = constrain(stepSpeed, 0, RPM_AT_MAX_TEMP);
                UpdateStepperSpeed(stepSpeed);

                lcd.setCursor(0, 0);
                lcd.print("T:");             // 2 chars
                lcd.print((int)currentTemp); // +3 = 5
                lcd.write(byte(0));          // +1 = 6
                lcd.print(" RPM:");          // +5 = 11
                if (stepSpeed < 10) {
                    lcd.print(" ");          // Padding para 2 dígitos
                }
                lcd.print(stepSpeed);        // +2 = 13-15
                lcd.print("  ");             // Preenche até 16
                lcd.setCursor(0, 1);
                lcd.print("Extrudando...   "); // 16 chars
                break;

            case AUTO_PAUSED_TEMP:
                targetTemp = AUTO_TARGET_TEMP;

                // Desabilita motor apenas na primeira vez que entra neste estado
                static bool autoPausedMotorDisabled = false;
                if (!autoPausedMotorDisabled) {
                    SetMotorEnabled(false);  // Desabilita motor (economia)
                    autoPausedMotorDisabled = true;
                }
                stepControl = 0;
                stepSpeed = 0;
                UpdateStepperSpeed(0);

                lcd.setCursor(0, 0);
                lcd.print("PAUSA T<");       // 8 chars
                lcd.print((int)TEMP_MIN_EXTRUSION); // +3 = 11
                lcd.write(byte(0));          // +1 = 12
                lcd.print("    ");           // +4 = 16
                lcd.setCursor(0, 1);
                lcd.print("Atual:");         // 6 chars
                lcd.print((int)currentTemp); // +3 = 9
                lcd.write(byte(0));          // +1 = 10
                lcd.print("      ");         // +6 = 16

                if (currentTemp >= TEMP_IDEAL) {
                    autoState = AUTO_RUNNING;
                    autoPausedMotorDisabled = false; // Reset para próxima vez
                }
                break;

            case AUTO_STOPPED:
                autoState = AUTO_IDLE;
                lcd.clear();
                break;
        }
    }

    //Next we calculate the error between the setpoint and the real value
    PID_error = targetTemp - currentTemp; // Removido o offset de +6 para um PID mais limpo
    //Calculate the P value
    PID_p = 0.01*kp * PID_error;
    //Calculate the I value in a range on +-6
    PID_i = PID_i + (0.01 * ki * PID_error); // Lógica do integral corrigida
    
    // Anti-windup: Limita o termo integral para evitar que ele cresça demais
    if(PID_i > max_PWM) PID_i = max_PWM;
    if(PID_i < 0) PID_i = 0;

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

    // Controlar cooler baseado em temperatura e PID
    ControlCooler();

    // Debug serial - apenas a cada 500ms para não spammar (economia de energia)
    static unsigned long lastSerialPrint = 0;
    if (millis() - lastSerialPrint > 500) {
        Serial.print("Temp: ");
        Serial.print(targetTemp);
        Serial.print("/");
        Serial.print(currentTemp);
        Serial.print(" | PID: ");
        Serial.print(PID_value);
        Serial.print(" | Err: ");
        Serial.print(PID_error);
        Serial.print(" | FAN: ");
        Serial.println(fanState ? "ON" : "OFF");
        lastSerialPrint = millis();
    }
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