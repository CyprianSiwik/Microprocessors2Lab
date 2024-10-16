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

enum{}

void setup() {

  pinMode(redLED, OUTPUT);
  pinMode(yellowLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  systemStartup();

  //set pins as outputs
  pinMode(13, OUTPUT);

  cli();//stop interrupts

  //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei();//allow interrupts

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
