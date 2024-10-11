/*

EECE.4520 Microprocessors II & Embedded Systems
Lab Group: Andrew Borin, Cyprian Siwik, Anna Schmidt

Project: Lab 1: Traffic Light Controller
Description: This code creates a basic traffic light system. The LEDs will stay on for a
  set amount of time depending on the input of the keypad. A buzzer is also implemented 
  for an audio signal when the light is about to switch colors. 

*/

#include <String.h>

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

byte rowPins[ROWS] = {28, 30, 32, 34};
byte colPins[COLS] = {36, 38, 40, 42}; 
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
unsigned char key;

String redTimeString = "";
String greenTimeString = "";

unsigned int redTime;
unsigned int greenTime;
unsigned int TempTime;
unsigned int chosenPin;

bool FlickerFlag = false;

volatile bool timer1_compa_enable;
volatile bool timer1_compb_enable;
volatile bool redSet;
volatile bool greenSet;

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

  OCR1B = 7812; //setting Timer1B to be 2 hz
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
  timer1_compa_enable = true;

}

ISR(TIMER1_COMPB_vect){

  timer1_compb_enable = true;
  //Serial.println("Timer B");

}

int getUserInput(){
  String stringtemp;
  
  for (int i = 0; i <= 1; i++){
    while(true){
      char Tempkey = keypad.getKey();

      if (Tempkey <= '9' && Tempkey >= '0'){
        stringtemp += Tempkey;

        Tempkey = "";
        Serial.println("out");
        break;
      }
    }
  }
  return stringtemp.toInt();

}

void loop() {
  // put your main code here, to run repeatedly:
  char StateKey = keypad.getKey();
  if (StateKey == 'A'){
    state = RedIN;
    redSet = false;
  }
  if(StateKey == 'B'){
    state = GreenIN;
    greenSet = false;
  }
  
  if(timer1_compa_enable) {
    timer1_compa_enable = false;

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
          Serial.println("completed Initial");
        }
        
        
        break;

      case RedIN:

        TIMSK1 &= ~(1 << OCIE1A);  //turn off 1 second timer to not interrupt next bit of code
        
        Serial.println("Inside RedIN");
        
        redTime = getUserInput();

        Serial.print("This is the RedTime: ");
        Serial.println(redTime);

        while(true){
          char checkKey = keypad.getKey();
          if (checkKey == '#' && !redSet){
            state = GreenIN;
            redSet = true;
            break;
          } 
          else if(checkKey == 'A'){
            state = RedIN;
            break;
          }
        }

        TIMSK1 |= (1 << OCIE1A);
        break;

      case GreenIN:
        TIMSK1 &= ~(1 << OCIE1A);  //turn off 1 second timer to not interrupt next bit of code
        
        Serial.println("Inside GreenIN");
        
        greenTime = getUserInput();

        Serial.print("This is the GreenTime: ");
        Serial.println(greenTime);

        while(true){
          char checkKey = keypad.getKey();
          if (checkKey == '#' && !greenSet){
            state = RedSolid;
            greenSet = true;
            break;
          } 
          else if(checkKey == 'B'){
            state = GreenIN;
            break;
          }
        }

        TIMSK1 |= (1 << OCIE1A);
        break;

      case RedSolid:
        
        digitalWrite(redLED, HIGH);
        TempTime++;
        Serial.println("Counting Red");
        
        if(TempTime == (redTime - 3)){
          Serial.println("Transitioning to blinking");
          digitalWrite(buzzerPin, HIGH);
          TempTime = 0;
          chosenPin = redLED;

          timer1_compa_enable = false;
          TIMSK1 &= ~(1 << OCIE1A);  //Disables 1 sec timer so that it doesnt interrupt half second timer
          TIMSK1 |= (1 << OCIE1B);  //Enables 0.5 sec timer so that the 0.5 sec light blink can run
          Serial.println("disabled OCIEA enabled OCIEB");
          Serial.println(OCR1B);
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

  if(timer1_compb_enable){
    timer1_compb_enable = false; 
    Serial.println("Entered second timer");
    if(TempTime != 6){
      if(!FlickerFlag){
        digitalWrite(chosenPin, LOW);
      }
      else if(FlickerFlag){
        digitalWrite(chosenPin, HIGH);
      }

      FlickerFlag = !FlickerFlag;
      TempTime++;
      Serial.println("Second Counter");
    }
    else{
      TempTime = 0;
      Serial.println("Switching Colors");

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
}

