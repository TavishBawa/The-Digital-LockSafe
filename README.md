//PROJECT : The Digital LockSafe
//PURPOSE : To rotate a servo motor and multiplex a display for unlocked and locked states of The Digital LockSafe 
//COURSE  : TEJ3M
//AUTHOR  : Tavish Bawa
//DATE    : 2025 01 17
//MCU     : 328p (Nano)
//STATUS  : Working
//REFERENCE: 
//NOTES   : 

#include <Servo.h>  // Servo library for controlling the locking motor

// Shift register control pins
#define DATA_PIN 12   // Serial data input to shift register
#define LATCH_PIN 11  // Latch pin (updates outputs)
#define CLOCK_PIN 10  // Clock pin (shifts bits)

// Display brightness control (PWM)
#define ENABLE 3

// Logic inputs
#define STATE_PIN 2    // Indicates OPENED (high) or CLOSED (low)
#define BUTTON_PIN A4  // Button must be pressed to allow a change

// Servo motor control
#define SERVO_PIN 6

// Potentiometer wiring (software-powered)
#define POT_VCC A0
#define POT_PIN A1
#define POT_GND A2

// Display timing
#define PACE 1  // Delay per digit (ms)

// Servo angles
#define SERVO_OPEN_ANGLE 0
#define SERVO_CLOSED_ANGLE 90

// Digit enable transistors (multiplexing)
const int transistorPins[6] = { 4, 13, 7, 5, 8, 9 };

// Create servo object
Servo doorServo;

// 14-segment patterns (common cathode)
// Each bit corresponds to a segment
uint16_t letters[] = {
  0b1111110000000000,  // O
  0b1100111100000000,  // P
  0b1001111100000000,  // E
  0b0110110010000100,  // N
  0b0111101100000000,  // D
  0b1001110000000000,  // C
  0b0001110000000000,  // L
  0b1011011100000000   // S
};

// Words spelled using indices into letters[]
const int OPENED[] = { 0, 1, 2, 3, 2, 4 };  // O P E N E D
const int CLOSED[] = { 5, 6, 0, 7, 2, 4 };  // C L O S E D

// What the system is currently showing / doing
bool currentOpenState = false;

// Used for rising-edge detection on the button
bool lastButtonState = LOW;

void setup() {

  // Power the potentiometer using GPIO pins
  pinMode(POT_VCC, OUTPUT);
  digitalWrite(POT_VCC, HIGH);

  pinMode(POT_GND, OUTPUT);
  digitalWrite(POT_GND, LOW);

  pinMode(POT_PIN, INPUT);

  // PWM brightness control
  pinMode(ENABLE, OUTPUT);

  // Configure shift register pins
  pinMode(DATA_PIN, OUTPUT);
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);

  // Inputs
  pinMode(STATE_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT);

  // Set all digit control transistors as outputs and turn them off
  for (int i = 0; i < 6; i++) {
    pinMode(transistorPins[i], OUTPUT);
    digitalWrite(transistorPins[i], LOW);
  }

  // Attach servo and start in CLOSED position
  doorServo.attach(SERVO_PIN);
  doorServo.write(SERVO_CLOSED_ANGLE);

  // Clear display by sending 0s to shift register
  digitalWrite(LATCH_PIN, LOW);
  shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, 0);
  shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, 0);
  digitalWrite(LATCH_PIN, HIGH);
}

void loop() {

  // Read current button state
  bool buttonNow = digitalRead(BUTTON_PIN);

  // Button Rising-Edge Detection
  // Only triggers when button goes from low to high
  if (!lastButtonState && buttonNow) {

    // Read the desired state only when button is pressed
    currentOpenState = digitalRead(STATE_PIN);

    // Move servo to match state
    doorServo.write(currentOpenState ? SERVO_OPEN_ANGLE : SERVO_CLOSED_ANGLE);
  }

  // Store button state for next loop iteration
  lastButtonState = buttonNow;

  // Select which word to display
  const int* word = currentOpenState ? OPENED : CLOSED;

  // Display Multiplexing
  // Refresh display continuously so it appears steady
  for (int digit = 0; digit < 6; digit++) {

    // Turn OFF all digits
    for (int i = 0; i < 6; i++) {
      digitalWrite(transistorPins[i], LOW);
    }

    // Send segment data to shift register
    digitalWrite(LATCH_PIN, LOW);
    shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, lowByte(letters[word[digit]]));
    shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, highByte(letters[word[digit]]));
    digitalWrite(LATCH_PIN, HIGH);

    // Enable the current digit
    digitalWrite(transistorPins[digit], HIGH);

    // Read potentiometer and scale to PWM for brightness per digit
    int potValue = analogRead(POT_PIN);
    analogWrite(ENABLE, potValue >> 2);  // 0-1023 -> 0-255

    // Short delay for persistence of vision
    delay(PACE);

    // Turn digit back off before moving on
    digitalWrite(transistorPins[digit], LOW);
  }
}
