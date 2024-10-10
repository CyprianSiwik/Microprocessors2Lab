#include "Keypad.h"

//Variables for LEDs and Buzzer
const int buzzerPin = 12;
const int redLED = 11;
const int yellowLED = 10;
const int greenLED = 9;

//Variables for Keypad
const byte ROWS = 4; // number of rows
const byte COLS = 4; // number of columns
char keys[ROWS][COLS] = {
{'1','2','3', 'A'},
{'4','5','6', 'B'},
{'7','8','9', 'C'},
{'*','0','#', 'D'}
};

byte rowPins[ROWS] = {8, 7, 6, 5};
byte colPins[COLS] = {4, 3, 2, 1}; 
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);


void setup() {

}

void loop() {
  // put your main code here, to run repeatedly:

}
