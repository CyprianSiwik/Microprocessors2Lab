#include <RTClib.h>
#include <LiquidCrystal.h>
#include <arduinoFFT.h>
#include <string.h>


const int speedPin = 2;
const int backPin = 31;
const int forwardPin = 30;
const int buttonPin = 49;


#define CHANNEL A1
const uint16_t samples = 128; //This value MUST ALWAYS be a power of 2
const float samplingFrequency = 1000; //Hz, must be less than 10000 due to ADC
unsigned int sampling_period_us;
unsigned long microseconds;

float vReal[samples];
float vImag[samples];
int val = 0;
int minVal = 1024;
int maxVal = 0;

ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, samples, samplingFrequency, true);

bool rtcUpdate = false;
bool motorUpdate = false;
bool motorDirect = false;
bool motorDirectUpdate = false;

int motorVal = 1;
int motorSpeed = 0;

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

const int rs = 12, E = 11, d4 = 4, d5 = 5, d6 = 6, d7 = 7;

LiquidCrystal lcd(rs, E, d4, d5, d6, d7);
RTC_DS1307 rtc;

void setup(){
  Serial.begin(9600);
  pinMode(speedPin, OUTPUT);
  pinMode(backPin, OUTPUT);
  pinMode(forwardPin, OUTPUT);
  pinMode(A1, INPUT);

  lcd.begin(16,2);
  lcd.clear();
  lcd.print("Hello World");


  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  sampling_period_us = round(1000000*(1.0/samplingFrequency));

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


ISR(TIMER1_COMPA_vect){
  rtcUpdate = true;
  motorUpdate = false;
  motorDirectUpdate = false;
}


void loop(){
  
  if(rtcUpdate){
  DateTime now = rtc.now();

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second()));

  //Serial.println("up");
  rtcUpdate = false;
  }

  microseconds = micros();
  
  // Get samples

  for(int i=0; i<samples; i++)
  {
      int val = analogRead(CHANNEL);
      vReal[i] = analogRead(CHANNEL);
      vImag[i] = 0;

      if(val > maxVal) maxVal = val;
      if(val < minVal) minVal = val;
      while(micros() - microseconds < sampling_period_us){
        //empty loop
      }
      microseconds += sampling_period_us;
    
  }

  if((maxVal - minVal) > 10){

    float sum = 0;

    for(int i = 0; i < samples; i++){
      sum += vReal[i];
    }

    float avg = sum / samples;
    for(int j = 0; j < samples; j++){
      vReal[j] -= avg;
    }
  }

  FFT.windowing(vReal, samples, FFTWindow::Hamming, FFTDirection::Forward);

  FFT.compute(vReal, vImag, samples, FFTDirection::Forward);

  FFT.complexToMagnitude();

  float x = FFT.majorPeak();
  //Serial.println(x);
  
  if(x <= 267 && x >= 257 && !motorUpdate){
    if(motorVal < 4){
      motorVal++;
      Serial.println("Fan Up: " + String(x));
    }
    motorUpdate = true;
  }
  else if(x <= 448 && x >= 431 && !motorUpdate){
    if(motorVal > 0){
      motorVal--;
      Serial.println("Fan Down: " + String(x));
    }
    motorUpdate = true;
  }
  motorSpeed = map(motorVal, 0, 4, 0, 255);
  analogWrite(speedPin, motorSpeed);
  Serial.println(motorSpeed);



  if (digitalRead(buttonPin) == 1 && !motorDirectUpdate){
    motorDirect = !motorDirect;
    motorDirectUpdate = true;
    Serial.println("Toggled Fan");
  }

  if(motorDirect){
    digitalWrite(forwardPin, HIGH);
    digitalWrite(backPin, LOW);
    lcd.setCursor(0, 1);
    lcd.print("Dir: C;Speed: " + String(motorVal));
  }
  else if(!motorDirect){
    digitalWrite(forwardPin, LOW);
    digitalWrite(backPin, HIGH);
    lcd.setCursor(0, 1);
    lcd.print("Dir: CC;Speed: " + String(motorVal));
  } 

}