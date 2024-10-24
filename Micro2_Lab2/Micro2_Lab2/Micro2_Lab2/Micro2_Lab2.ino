#include <Wire.h>
#include <MPU9250.h>

//Control Variables and Pin Assignments
MPU9250 mpu;

const int xPin = A0;
const int yPin = A1;

const int buzzerPin = 10;

const int buttonPin = 9;

float gx, gy;

volatile bool isJoystickMode = true;

volatile bool buzzerOn = false;
volatile uint16_t buzzerTimer = 0;

volatile bool timerTriggered = false;

volatile uint16_t debounceCounter = 0;
volatile bool buttonPressed = false; 

//Game Variables
const int gridWidth = 10;
const int gridHeight = 10;

struct Point {
  int x;
  int y;
};

Point snake[100];   // The snake's body (max length = 100)
int snakeLength = 1;
Point apple;        // Apple position
Point direction = {0, 1}; // Initial direction (moving right)

bool gameOver = false;

//Timer Setups
void setupTimer1() {
  noInterrupts();  // Disable interrupts during setup
  TCCR1A = 0;  // Clear Timer1 registers
  TCCR1B = 0;

  // Set CTC (Clear Timer on Compare Match) mode
  TCCR1B |= (1 << WGM12);  // CTC mode

  // Set the timer prescaler to 64 (1 tick every 4 microseconds)
  TCCR1B |= (1 << CS11) | (1 << CS10);

  // Set compare match register for a 100ms interval
  OCR1A = 24999;  // 100ms = 16Mhz / 64 / 100ms

  // Enable Timer1 compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  interrupts();  // Enable interrupts
}

void setupTimer2() {
  // Set Timer2 for Fast PWM mode
  TCCR2A = (1 << WGM21) | (1 << WGM20);  // Fast PWM
  TCCR2B = (1 << CS22);  // Set prescaler to 64

  // Set pin mode for buzzer
  pinMode(buzzerPin, OUTPUT);
}

ISR(TIMER1_COMPA_vect) {
  timerTriggered = true;

  // Handle buzzer duration countdown
  if (buzzerOn && buzzerTimer > 0) {
    buzzerTimer--;
    if (buzzerTimer == 0) {
      buzzerOn = false;  // Turn off the buzzer after the timer expires
      TCCR2A &= ~(1 << COM2A1);  // Stop PWM on buzzer pin
    }
  }

  // Increment debounce counter every 100ms
  debounceCounter++;
}

//Function Definitions
void joystickControl() {
  int xPos = analogRead(xPin);
  int yPos = analogRead(yPin);

  if (xPos < 300) {
    Serial.println("Left");
  } else if (xPos > 700) {
    Serial.println("Right");
  }

  if (yPos < 300) {
    Serial.println("Up"); 
  } else if (yPos > 700) {
    Serial.println("Down");
  }
}

void gyroControl() {
  mpu.update();

  //Gyro values in degrees/sec
  gx = mpu.getGyroX();
  gy = mpu.getGyroY();

  if (gx > 0.5) {
    Serial.println("Left"); 
  } else if (gx < -0.5) {
    Serial.println("Right");
  }

  if (gy > 0.5) {
    Serial.println("Up");
  } else if (gy < -0.5) {
    Serial.println("Down");
  }
}

void toggleBuzzer(uint16_t duration) {
  buzzerOn = true;
  buzzerTimer = duration;  // Set the buzzer duration in 100ms increments
  OCR2A = 128;  // Set duty cycle for 50% PWM
  TCCR2A |= (1 << COM2A1);  // Start PWM on buzzer pin
}

void checkButtonPress() {
  const uint16_t debounceDelay = 2;  // 200ms debounce delay (100ms per tick)

  if (digitalRead(buttonPin) == LOW) {  // Button pressed
    if (!buttonPressed && debounceCounter >= debounceDelay) {
      // Toggle the control mode
      isJoystickMode = !isJoystickMode;
      Serial.println(isJoystickMode ? "Joystick Mode" : "Gyro Mode");

      // Reset debounce counter and mark button as pressed
      debounceCounter = 0;
      buttonPressed = true;
    }
  } else {
    buttonPressed = false;  // Reset the button press state when released
  }
}

void eatApple() {
  if (Serial.available()) {
    char apple = Serial.read();
    if (apple == 'A') {  // If 'A' is received, apple is eaten
      toggleBuzzer(10);  // Buzzer on for 1 second (10 x 100ms)
    }
  }
}

void initializeGame() {
  // Initialize snake in the center of the grid
  snake[0].x = gridWidth / 2;
  snake[0].y = gridHeight / 2;
  snakeLength = 1;

  // Place the apple randomly
  placeApple();
  
  Serial.println("Game Initialized");
}

void placeApple() {
  apple.x = random(0, gridWidth);
  apple.y = random(0, gridHeight);
  
  Serial.print("Apple placed at: ");
  Serial.print(apple.x);
  Serial.print(", ");
  Serial.println(apple.y);
}

void moveSnake() {
  if (gameOver) {
    Serial.println("Game Over!");
    return;
  }

  // Calculate the new head position
  Point newHead;
  newHead.x = snake[0].x + direction.x;
  newHead.y = snake[0].y + direction.y;

  // Check for collisions with walls
  if (newHead.x < 0 || newHead.x >= gridWidth || newHead.y < 0 || newHead.y >= gridHeight) {
    gameOver = true;
    Serial.println("Snake hit the wall!");
    return;
  }

  // Check for collisions with itself
  for (int i = 0; i < snakeLength; i++) {
    if (newHead.x == snake[i].x && newHead.y == snake[i].y) {
      gameOver = true;
      Serial.println("Snake hit itself!");
      return;
    }
  }

  // Move the snake
  for (int i = snakeLength; i > 0; i--) {
    snake[i] = snake[i - 1];  // Shift each segment to follow the previous one
  }
  snake[0] = newHead;  // The new head is at the new position

  // Check if the snake ate the apple
  if (newHead.x == apple.x && newHead.y == apple.y) {
    snakeLength++;  // Grow the snake
    placeApple();   // Place a new apple
    toggleBuzzer(5); // Beep to indicate apple eaten
    Serial.println("Apple eaten!");
  }

  // Print the snake's position for debugging
  Serial.print("Snake head at: ");
  Serial.print(snake[0].x);
  Serial.print(", ");
  Serial.println(snake[0].y);
}

void changeDirection(char directionInput) {
  if (directionInput == 'w' && direction.y != 1) {  // Up
    direction.x = 0;
    direction.y = -1;
  } else if (directionInput == 's' && direction.y != -1) {  // Down
    direction.x = 0;
    direction.y = 1;
  } else if (directionInput == 'a' && direction.x != 1) {  // Left
    direction.x = -1;
    direction.y = 0;
  } else if (directionInput == 'd' && direction.x != -1) {  // Right
    direction.x = 1;
    direction.y = 0;
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(buttonPin, INPUT_PULLUP);

  Wire.begin();

  MPU9250Setting setting;
  if (!mpu.setup(0x68, setting, Wire)) {
    Serial.println("MPU9250 setup failed!");
    while (1);  // Stop if MPU-9250 setup fails
  } else {
    Serial.println("MPU9250 setup successful!");
  }

  setupTimer1();
  setupTimer2();

}

void loop() {
  if (timerTriggered) {
    // Check for button press to toggle between joystick and gyro control
    checkButtonPress();

    // Update the snake's direction based on joystick or gyro input
    if (isJoystickMode) {
      joystickControl();
    } else {
      gyroControl();
    }
    
    // Move the snake
    moveSnake();

    // Check for apple events (like apple eaten)
    eatApple();

    // Reset timer flag
    timerTriggered = false;
  }
}