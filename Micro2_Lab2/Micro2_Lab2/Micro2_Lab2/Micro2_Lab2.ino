#include <Wire.h>
#include <MPU6050.h>
#include <avr/interrupt.h>

MPU6050 mpu;

const int xPin = A0;
const int yPin = A1;
const int buzzerPin = 9;

int xVal, yVal;
int16_t ax, ay, az;
int16_t gx, gy, gz;

volatile bool readSensorsFlag = false;
volatile bool buzzerOn = false;
volatile unsigned int buzzerCount = 0;  // Counts timer interrupts for buzzer timing

void setup() {
  Serial.begin(9600);
  pinMode(buzzerPin, OUTPUT);
  pinMode(xPin, INPUT);
  pinMode(yPin, INPUT);

  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    while (1);
  }

  // Configure Timer1 for 100ms interval
  cli();                  // Disable global interrupts
  TCCR1A = 0;             // Clear Timer1 settings
  TCCR1B = 0;
  TCNT1 = 0;              // Initialize counter
  OCR1A = 15624;          // Set compare match register for 100ms (16MHz / (256 * 100ms) - 1)
  TCCR1B |= (1 << WGM12); // CTC mode
  TCCR1B |= (1 << CS12);  // Prescaler 256
  TIMSK1 |= (1 << OCIE1A); // Enable Timer1 compare interrupt
  sei();                  // Enable global interrupts
}

void loop() {
  // Handle sensor readings in the main loop
  if (readSensorsFlag) {
    readSensorsFlag = false;
    readJoystick();
    readGyro();
  }

  // Check for serial input to control the buzzer
  if (Serial.available()) {
    char command = Serial.read();
    if (command == 'b') {
      buzzerOn = true;
      buzzerCount = 0;  // Reset count to manage buzzer duration
      tone(buzzerPin, 1000);  // Start the buzzer
    }
  }

  // Manage buzzer off timing in the main loop
  if (buzzerOn && buzzerCount >= 5) {  // 5 counts of 100ms = 500ms
    noTone(buzzerPin);  // Stop the buzzer
    buzzerOn = false;
  }
}

// Timer1 interrupt service routine for 100ms interval
ISR(TIMER1_COMPA_vect) {
  readSensorsFlag = true;  // Set flag to read sensors every 100ms

  // Increment buzzer counter if it's on
  if (buzzerOn) {
    buzzerCount++;
  }
}

void readJoystick() {
  xVal = analogRead(xPin);
  yVal = analogRead(yPin);

  if (xVal > 600) {
    Serial.println('d');
  } else if (xVal < 400) {
    Serial.println('a');
  }

  if (yVal > 600) {
    Serial.println('w');
  } else if (yVal < 400) {
    Serial.println('s');
  }
}

void readGyro() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  if (gx > 10000) {
    Serial.println('d');
  } else if (gx < -10000) {
    Serial.println('a');
  }

  if (gy > 10000) {
    Serial.println('w');
  } else if (gy < -10000) {
    Serial.println('s');
  }
}















/*
  Serial-WR

  Output hello to Serial
  Read input from Serial. If there is a 'E' detected, flip the LED

  yluo
  
*/

// LED status
int led_status = HIGH;
int incomingByte = 0;

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  led_status = HIGH;
}

// the loop routine runs over and over again forever:
void loop() {
  // read from the Serial port:
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();
    Serial.println(incomingByte);
    
    if(incomingByte == 'E') {
      // flip LED
      led_status = (led_status == HIGH)? LOW : HIGH;
      digitalWrite(LED_BUILTIN, led_status);
    }
  }

  delay(1000);
  Serial.println("Hello");

}
