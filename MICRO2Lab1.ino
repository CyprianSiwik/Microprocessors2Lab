//WRITTEN 9/10/24 FOR MICRO II - TRAFFIC LIGHT LAB 1

//Global variable for button pin number
const int buttonPin = 8;
int buttonState = 0;

//assign pin numbers for color of led
const int greenPin = 10;
const int yellowPin = 11;
const int redPin = 12;

//pin number for buzzer
const int buzzerPin = 7;

//debounce settings
const unsigned long debounceDelay = 50;

//manage debounce state
unsigned long lastButtonPressTime = 0;
bool lastButtonState = HIGH;




// blink function - void does not return anything
// parameters for pin number - to specify red, green, yellow
// duration long parameter - allows us to speciify whether we want 15 or 3 seconds
// boolean flashing - if true - simply flashes until a button is pressed to go into our void loop
// if flashing false - skips and goes into duration of solid color code

/*
void again(int x){
  while(true){

  }
}
*/

void blink(int pin, unsigned long duration, bool flashing) {
  //FLASHING STATE STATIC
  unsigned long endTime = millis() + duration;

  if (flashing) {   //Flashes 1 sec on 1 sec off
    while (true) {
      unsigned long startTime = millis(); //initial time 
      while (startTime != startTime + 1000){ //1 second more time
        digitalWrite(pin, HIGH);
      }
      unsigned long currentTime = millis(); //second time check
      while (currentTIme != currentTime + 1000){
        digitalWrite(pin, LOW);
      }
      
      //digitalWrite(pin, HIGH);
      //digitalWrite(pin, LOW);

      bool currentButtonState = digitalRead(buttonPin);
      unsigned long currentTime = millis();

      buttonState = digitalRead(buttonPin);

      if (buttonState == HIGH) { //check for * key to start 
        break;
      }
      break;

      if ((currentTime - lastButtonPressTime) > debounceDelay) {
        if (currentButtonState == LOW) {
          break;
        }
      }
      lastButtonState = currentButtonState;
      
    }
  }
  else {

    unsigned long buzzerStartTime = endTime - 3000;

    digitalWrite(pin, HIGH);

    while (millis() < endTime) {
      if (millis() >= buzzerStartTime) {
        tone(buzzerPin, 100);
      }
      delay(10);
    }

    digitalWrite(pin, LOW);
    noTone(buzzerPin);
    delay(50);
  }
}

void setup() {
  // put your setup code here, to run once:
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(greenPin, OUTPUT);
  pinMode(yellowPin, OUTPUT);
  pinMode(redPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);

  blink(redPin, 0, true);  //pin 13 for red , doesnt matter, infinite flashing true
}

void loop() {
  // put your main code here, to run repeatedly:
  blink(redPin, 15000, false);
  blink(greenPin, 15000, false);
  blink(yellowPin, 3000, false );
}
