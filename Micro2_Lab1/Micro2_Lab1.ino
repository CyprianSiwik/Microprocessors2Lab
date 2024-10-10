#include "Keypad.h"

//Variables for LEDs and Buzzer
const int buzzerPin = 13;
const int redLED = 12;
const int yellowLED = 11;
const int greenLED = 10;

//Variables for Keypad
const byte ROWS = 4; // number of rows
const byte COLS = 4; // number of columns
char keys[ROWS][COLS] = {
{'1','2','3', 'A'},
{'4','5','6', 'B'},
{'7','8','9', 'C'},
{'*','0','#', 'D'}
};

byte rowPins[ROWS] = {9, 8, 7, 6};
byte colPins[COLS] = {5, 4, 3, 2}; 
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

//LED Controls and Other Variables
const int redOn = 10;
const int greenOn = 10;
const int yellowOn = 3;

bool durationSet = false;
bool lightRunning = false;

void setup() {

  pinMode(redLED, OUTPUT);
  pinMode(yellowLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  systemStartup();

}

void loop() {
  // put your main code here, to run repeatedly:

}

//Flashes red LED on startup
void systemStartup() {
  //TODO: Change function to non-blocking
  digitalWrite(redLED, HIGH);
  delay(1000);
  digitalWrite(redLED, LOW);
  delay(1000);
}

//Buzzer Control
void buzzerControl() {
  //TODO: Change function to non-blocking
  digitalWrite(buzzerPin, HIGH);
  delay(3000);
  digitalWrite(buzzerPin, LOW);
}

void trafficSignal() {
  //TODO: Implement this function
}

void keypadControl() {
  //TODO: Implement this function
}