/*

EECE.4520 Microprocessors II & Embedded Systems
Lab Group: Andrew Borin, Cyprian Siwik, Anna Schmidt

Project: Lab 1: Traffic Light Controller
Description: This code creates a basic traffic light system. The LEDs will stay on for a
  set amount of time depending on the input of the keypad. A buzzer is also implemented 
  for an audio signal when the light is about to switch colors. 

*/

#include <avr/pgmspace.h>

#include "Keypad.h"

//Variables for LEDs and Buzzer
const int buzzerPin = 13;
const int redLED = 12;
const int yellowLED = 11;
const int greenLED = 10;

boolean toggle1 = true;

//Variables for Keypad
const byte ROWS = 4; // number of rows
const byte COLS = 4; // number of columns
const char keys[ROWS][COLS] = {
  {'1','2','3', 'A'},
  {'4','5','6', 'B'},
  {'7','8','9', 'C'},
  {'*','0','#', 'D'}
};

byte rowPins[ROWS] = {9, 8, 7, 6};
byte colPins[COLS] = {5, 4, 3, 2}; 
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// Variables for Button
const int buttonPin = 2; // Pin where the button is connected
int buttonState = HIGH;  // Current state of the button
int lastButtonState = HIGH;  // Previous state of the button
bool ledState = false;   // Tracks the LED state
int count = 0;           // Counts stable states
bool manualControl = false; // Flag to check if manual control is active

//LED Controls and Other Variables
const int redOn = 10;
const int greenOn = 10;
const int yellowOn = 3;

enum{Initial, KeypadA, KeypadB, RedIN, GreenIN, EndKeypad, RedSolid, RedFlickBuzz, GreenSolid, GreenFlickBuzz, YellowBuzz};
unsigned char state = Initial; 
unsigned char key = "";

unsigned char redTimeString = "";
unsigned char greenTimeString = "";

unsigned int redTime;
unsigned int greenTime;
unsigned int TempTime;
unsigned int chosenPin;

bool FlickerFlag = false;

bool durationSet = false;
bool lightRunning = false;

void setup() {

  pinMode(redLED, OUTPUT);
  pinMode(yellowLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(buzzerPin, OUTPUT);

  Serial.begin(9600);

  //Setup Timers
  cli();//stop interrupts
  //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)

  OCR1B = 31248; //setting Timer1B to be 2 hz
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  TIMSK1 &= ~(1 << OCIE1B);

  sei();//allow interrupts
}


ISR(TIMER1_COMPA_vect) {
  //TODO: Implement this function
  char key = keypad.getKey();

  switch(state){
    case Initial:
      if (toggle1){
        digitalWrite(redLED,HIGH);
        toggle1 = 0;
        Serial.println("ON");
      }
      else{
        digitalWrite(redLED,LOW);
        toggle1 = 1;
        state = KeypadA;
        Serial.println("OFF");
      }
      Serial.println("completed Initial");
      
      break;

    case KeypadA:
  
      if (key == 'A'){
        state = RedIN;
        Serial.println("Completed KeypadA");
      }
      break;

    case KeypadB:
      if(key == 'B'){
        state = GreenIN;
        Serial.println("Completed KeypadB");
      }
      break;

    case RedIN:
      if (key != '#') {
        redTimeString += key;
        Serial.println("Added to RedIN String");
      }
      else if(key == '#'){
        redTime = atoi(redTimeString);
        state = KeypadB;
        Serial.println("Completed RedIN");
        break;
      }
      char redTimeString = "";
      break;

    case GreenIN:
      if (key != '#') {
        greenTimeString += key;
        Serial.println("Added to GreenIN String");

      }
      else if(key == '#'){
        greenTime = atoi(greenTimeString);
        state = EndKeypad;
        break;
      }
      char greenTimeString = "";
      break;

    case EndKeypad: 
      if(key == '*'){
        state = RedSolid;
      }
      break;

    case RedSolid:
      TempTime++;
      digitalWrite(redLED, HIGH);
      digitalWrite(buzzerPin, HIGH);
      if(TempTime == (redTime - 3)){
        TempTime = 0;
        chosenPin = redLED;

        TIMSK1 &= ~(1 << OCIE1A);  //Disables 1 sec timer so that it doesnt interrupt half second timer
        TIMSK1 |= (1 << OCIE1B);  //Enables 0.5 sec timer so that the 0.5 sec light blink can run
      }
      break;

    case GreenSolid:
      TempTime++;
      digitalWrite(greenLED, HIGH);
      digitalWrite(buzzerPin, HIGH);
      if(TempTime == (greenTime - 3)){
        TempTime = 0;
        chosenPin = greenLED;

        TIMSK1 &= ~(1 << OCIE1A);  //Disables 1 sec timer so that it doesnt interrupt half second timer
        TIMSK1 |= (1 << OCIE1B);  //Enables 0.5 sec timer so that the 0.5 sec light blink can run
      }
      break;

    case YellowBuzz:
      digitalWrite(yellowLED, HIGH);
      digitalWrite(buzzerPin, HIGH);
      chosenPin = yellowLED;

      TIMSK1 &= ~(1 << OCIE1A);  //Disables 1 sec timer so that it doesnt interrupt half second timer
      TIMSK1 |= (1 << OCIE1B);  //Enables 0.5 sec timer so that the 0.5 sec light blink can run

      break;
  }

}

ISR(TIMER1_COMPB_vect){
  if(TempTime != 6){
    if(!FlickerFlag){
      digitalWrite(chosenPin, LOW);
      
    }
    else if(FlickerFlag){
      digitalWrite(chosenPin, HIGH);
      
    }

    FlickerFlag = !FlickerFlag;
    TempTime++;
  }
  else{
    TempTime = 0;
    digitalWrite(buzzerPin, LOW);
    digitalWrite(chosenPin, LOW);

    if (chosenPin == redLED){
      state = GreenSolid;
    }
    else if(chosenPin == greenLED){
      state = YellowBuzz;
    }
    else{
      state = RedSolid;
    }

    TIMSK1 &= ~(1 << OCIE1B);  //Disables 0.5 sec timer so that it doesnt interrupt one second timer
    TIMSK1 |= (1 << OCIE1A);  //Enables 1 sec timer so that the main state machine returns
  }

}
void loop() {
  // put your main code here, to run repeatedly:
  char Tempkey = keypad.getKey();
  
  if (Tempkey != 0){
    Serial.println(Tempkey);
    key = Tempkey;
    if (key == 'A'){
      state = RedIN;
    }
    else if (key == 'B'){
      state = GreenIN;
    }
  }
}

