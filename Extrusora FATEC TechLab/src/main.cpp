#include <Arduino.h>
#include <LiquidCrystal.h>

LiquidCrystal lcdFofo(12, 11, 5, 4, 3, 2);



float temperatura = 50;

void setup() {
    lcdFofo.begin(16, 2);
    lcdFofo.setCursor(0,0);
    lcdFofo.print("FATEC TechLab");
    lcdFofo.setCursor(0,1);
    lcdFofo.print("Extrusora PET");
    delay(1000);
}

void loop() {
    lcdFofo.clear();
    lcdFofo.setCursor(0,0);
    lcdFofo.print("TEMP: ");
    lcdFofo.print(temperatura, 1);
    lcdFofo.setCursor(0,1);
    if(temperatura < 80)
    {
        lcdFofo.print("ESQUENTANDO...");
        temperatura += 1.5;
    }
    else
    {
        lcdFofo.print("PRONTO!");
    }
    delay(1000);
}
