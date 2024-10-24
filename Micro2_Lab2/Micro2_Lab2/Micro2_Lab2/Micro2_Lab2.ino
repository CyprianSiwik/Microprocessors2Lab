#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

const int xPin = A0;
const int yPin = A1;
const int buzzerPin = 9;

int xVal, yVal;
int16_t ax, ay, az;
int16_t gx, gy, gz;

unsigned long previousMillis = 0;
const long interval = 100;

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
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

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

  if (Serial.available()) {
    char command = Serial.read();
    if (command == 'b') {
      activateBuzzer();
    }
  }
}

void activateBuzzer() {
  unsigned long buzzerStartMillis = millis();
  while (millis() - buzzerStartMillis < 500) {
    tone(buzzerPin, 1000);
  }
  noTone(buzzerPin);
}