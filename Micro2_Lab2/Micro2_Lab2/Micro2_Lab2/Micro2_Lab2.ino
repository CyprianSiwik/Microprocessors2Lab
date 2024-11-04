#include <Wire.h>
#include <avr/interrupt.h>



const int xPin = A0;
const int yPin = A1;
const int buzzerPin = 9;

int xVal, yVal;
int16_t ax, ay, az;
int16_t gx, gy, gz;

int led_status = HIGH;
int lastTimeJoy = 0;
int lastTimeGyro = 0;

float GyroXError = 0;
float GyroYError = 0;

const int MPU = 0x68; // MPU6050 I2C address

char readJoyStick(int xAxis, int yAxis){
  char command_to_return = 0;
  if(xAxis <= 450){
    command_to_return = 'd'; 
  }
  else if(xAxis >= 600){
    command_to_return = 'a';
  }
  else if(yAxis <= 450){
    command_to_return = 'w';
  }
  else if(yAxis >= 600){
    command_to_return = 's';
  }

  return command_to_return; 
}

void GyroErrorCalc(){
  int c = 0;

  float GyroX, GyroY;
  float GyroErrorX, GyroErrorY;
  
  while(c <= 200){
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);

    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();

    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    c++;
  }

  GyroXError = GyroErrorX / 200;
  GyroYError = GyroErrorY / 200;

}

char readGyro(){

  float GyroX = 0;
  float GyroY = 0;
  char command = 0;

  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;

  // Correct the outputs with the calculated error values
  GyroX += GyroXError; 
  GyroY += GyroYError; 


  if(GyroX >= 30){
    command = 'd';
  }
  else if(GyroY >= 30){
    command = 'w';
  }
  else if(GyroX <= -80){
    command = 'a';
  }
  else if(GyroY <= -80){
    command = 's';
  }
  else{
    command = 0;
  }

  return command;
}

void setup() {
  Serial.begin(9600);
  pinMode(buzzerPin, OUTPUT);
  pinMode(xPin, INPUT);
  pinMode(yPin, INPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  led_status = HIGH;

  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission

  GyroErrorCalc();
  
}
void loop() {
  // Handle sensor readings in the main loop

  // Check for serial input to control the buzzer
  if (Serial.available() > 0) {
    int command = Serial.read();
    if (command == 'b') {
      tone(buzzerPin, 1000, 50);
    }
    if(command == 'E') {
      // flip LED
      led_status = (led_status == HIGH)? LOW : HIGH;
      digitalWrite(LED_BUILTIN, led_status);
    }
  }

  int xAxis = analogRead(xPin);
  int yAxis = analogRead(yPin);

  char inputJoy = readJoyStick(xAxis, yAxis);
  char inputGyro = readGyro();

  int Time = millis();
  if((inputJoy != 0) && ((Time - lastTimeJoy) >= 500)){
    Serial.println(inputJoy);
    inputJoy = 0;
    lastTimeJoy = Time;
  }
  if((inputGyro != 0) && ((Time - lastTimeGyro) >= 500)){
    Serial.println(inputGyro);
    inputGyro = 0;
    lastTimeGyro = Time;
  }

}
