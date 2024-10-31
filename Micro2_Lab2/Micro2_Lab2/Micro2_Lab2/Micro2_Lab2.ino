#include <Wire.h>
#include <avr/interrupt.h>



const int xPin = A0;
const int yPin = A1;
const int buzzerPin = 9;

int xVal, yVal;
int16_t ax, ay, az;
int16_t gx, gy, gz;

int led_status = HIGH;
int lastTime = 0;

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

char readGyro(){
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  // Correct the outputs with the calculated error values
  GyroX = GyroX + 0.56; // GyroErrorX ~(-0.56)
  GyroY = GyroY - 2; // GyroErrorY ~(2)
  GyroZ = GyroZ + 0.79; // GyroErrorZ ~ (-0.8)
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
/*
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    while (1);
*/
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

  char input = readJoyStick(xAxis, yAxis);

  int Time = millis();
  if((input != 0) && ((Time - lastTime) >= 1000)){
    Serial.println(input);
    input = 0;
    lastTime = Time;
  }

}
