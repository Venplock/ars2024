// Automatic Resistor Sorter 12/6/2024

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>
#include <math.h> 

#define SCREEN_WIDTH 128 // OLED display width
#define SCREEN_HEIGHT 32 // OLED display height
#define OLED_RESET    -1 // Reset pin (-1 if shared with Arduino reset)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Constants for Ohmmeter
#define NUM_REF_RESISTORS 8
#define NUM_SELECT_PINS   3
#define MAX_ANALOG_VALUE  1023
#define SWITCH_RESISTANCE 4.5

float rRef[NUM_REF_RESISTORS] = {49.9, 100, 1000, 10000, 100000, 1000000, 4990000, 10000000};
const byte rSelPins[NUM_SELECT_PINS] = {5, 4, 3};
const byte enableMux = 6;

// Define arm lengths (in cm)
const float L1 = 19.5; // Length of Link 1
const float L2 = 14.5; // Length of Link 2

// Servo objects for magnet, blue, and red servo control
Servo ServoMagnet; // Controls the magnet's up/down movement
Servo ServoBlue;   // Controls left/right movement
Servo ServoRed;    // Controls forward/backward movement

const int electromagnetPin = 13;  // Pin to control the electromagnet

float uVals[4] = {0.0, 0.0, 0.0, 0.0};   // Default threshold values for bin selection
float tol = 0.15; // Default tolerance percentage

// Function to calculate inverse kinematics
bool calculateAngles(float x, float y, float &theta1, float &theta2) {
  // Distance from origin to target
  float r = sqrt(x * x + y * y);

  // Check if the target is reachable
  if (r > (L1 + L2) || r < fabs(L1 - L2)) {
    return false;
  }

  // Elbow angle
  float cos_theta2 = (r * r - L1 * L1 - L2 * L2) / (2 * L1 * L2);
  theta2 = acos(cos_theta2) * (180.0 / M_PI);

  // Shoulder angle
  float phi = atan2(y, x) * (180.0 / M_PI);
  float psi = atan2(L2 * sin(theta2 * M_PI / 180.0), L1 + L2 * cos(theta2 * M_PI / 180.0)) * (180.0 / M_PI);
  theta1 = phi + psi;

  return true;
}

int calculatePulseWidth1(float angle) {
  float minPulse = 565;  // Minimum pulse width in µs
  float maxPulse = 2500; // Maximum pulse width in µs
  float maxAngle = 270;  // Maximum angle in degrees

  // Calculate pulse width for the given angle
  return minPulse + (angle * (maxPulse - minPulse)) / maxAngle;
}

int calculatePulseWidth2(float angle) {
  float minPulse = 500;  // Minimum pulse width in µs
  float maxPulse = 2500; // Maximum pulse width in µs
  float maxAngle = 270;  // Maximum angle in degrees

  // Calculate pulse width for the given angle
  return minPulse + (angle * (maxPulse - minPulse)) / maxAngle;
}

void setup() {
  Serial.begin(9600);
  // Initialize the OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // 0x3C is the I2C address
    for (;;); // Halt if OLED initialization fails
  }
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Resistor");
  display.println("Sorter");
  display.display();
  delay(100);

  // Configure multiplexer pins
  pinMode(enableMux, OUTPUT);
  digitalWrite(enableMux, HIGH);

  for (int i = 0; i < NUM_SELECT_PINS; i++) {
    pinMode(rSelPins[i], OUTPUT);
    digitalWrite(rSelPins[i], HIGH);
  }

  // Attach servos to designated pins
  ServoMagnet.attach(9);
  ServoBlue.attach(10);
  ServoRed.attach(11);

  pinMode(electromagnetPin, OUTPUT);
  digitalWrite(electromagnetPin, LOW); // Make sure electromagnet is off initially


  // Move to neutral
  ServoRed.writeMicroseconds(1400);  
  ServoBlue.writeMicroseconds(750);
  delay(500);

}

void loop() {
  // Read threshold values from user input
  if (Serial.available() > 0) {
    
    String input = Serial.readStringUntil('\n'); // Read until newline
    input.trim(); // Remove any leading/trailing whitespace

    // Split the input into space-separated values
    int firstSpaceIndex = input.indexOf(' ');
    int secondSpaceIndex = input.indexOf(' ', firstSpaceIndex + 1);
    int thirdSpaceIndex = input.indexOf(' ', secondSpaceIndex + 1);

    if (firstSpaceIndex != -1 && secondSpaceIndex != -1 && thirdSpaceIndex != -1) {
      // If four threshold values are provided
      String values[4];
      values[0] = input.substring(0, firstSpaceIndex);
      values[1] = input.substring(firstSpaceIndex + 1, secondSpaceIndex);
      values[2] = input.substring(secondSpaceIndex + 1, thirdSpaceIndex);
      values[3] = input.substring(thirdSpaceIndex + 1);

      // Convert strings to floats and set threshold values
      for (int i = 0; i < 4; i++) {
        uVals[i] = values[i].toFloat();
      }
     

      Serial.println("A");
    } else if (input.equals("DONE")) {
      // Move to neutral
      ServoRed.writeMicroseconds(1400);  
      ServoBlue.writeMicroseconds(750);
      delay(500);

    } else {
      // Handle coordinates input
      int spaceIndex = input.indexOf(' ');
      if (spaceIndex != -1) {
        String xValue = input.substring(0, spaceIndex);
        String yValue = input.substring(spaceIndex + 1);
        float targetX = xValue.toFloat();
        float targetY = yValue.toFloat();

        if (targetX != 0.0 || targetY != 0.0) {
          // Move to (x, y) coordinates
          float theta1, theta2;
          if (calculateAngles(targetX, targetY, theta1, theta2)) {
            int redPulse = calculatePulseWidth1(theta1);
            int bluePulse = calculatePulseWidth2(theta2);

            // Move servos to target (x, y)
            ServoRed.writeMicroseconds(redPulse);
            ServoBlue.writeMicroseconds(bluePulse);
            delay(500); // Allow time for servos to reach target

            // Move magnet servo downwards to pick up the resistor
            ServoMagnet.write(140); // was 180
            delay(300);

            // Turn on the electromagnet
            digitalWrite(electromagnetPin, HIGH);

            delay(300);

            // Move magnet servo upwards after picking up the resistor
            ServoMagnet.write(90);
            delay(200);

            // Proceed to bin sorting
            performBinSequence();
          }
          Serial.println("A");
        }
      }
    }
    Serial.flush(); // Clear the buffer
  }

  delay(100); // Short delay for stability before next loop
}

void performBinSequence() {
  float lRes = 0.0;

  // Move to probe
  ServoRed.writeMicroseconds(1475);
  ServoBlue.writeMicroseconds(1150);
  delay(400);

  // Move ServoMagnet downwards to measure resistor
  ServoMagnet.write(180);
  delay(300);

  // Measure resistance after moving down to probe
  unsigned long startTime = millis();
  while (millis() - startTime < 1000) {
    float mRes = getMeasuredResistance();
    if (mRes > 0 && mRes < 10000000) { // Valid range
      lRes = mRes;

      // Update OLED with the current measurement
      display.clearDisplay();
      display.setTextSize(2.5);
      display.setCursor(0, 0);
      display.print(lRes, 2); // Only show the resistance value
      display.display();
    }
    delay(100); // Short delay for stability
  }

  ServoMagnet.write(90); // Go back up
  delay(300);

  // Use the last valid resistance measurement

    // Display the final resistance value
    display.clearDisplay();
    display.setTextSize(2.5);
    display.setCursor(0, 0);
    display.print(lRes, 2); // Only show the resistance value
    display.display();

    // Determine next bin based on raw resistor value using thresholds and tolerance
    String binText = "";

    // Check against each threshold range and move to the appropriate bin
    if (lRes >= uVals[0] * (1.0 - tol ) && lRes <= uVals[0] * (1.0 + tol )) {
        // Bin 1 position
        ServoRed.writeMicroseconds(1520);  
        ServoBlue.writeMicroseconds(1380);
        binText = " 1";

    } else if (lRes >= uVals[1] * (1.0 - tol ) && lRes <= uVals[1] * (1.0 + tol )) {
        // Bin 2 position
        ServoRed.writeMicroseconds(1500);  
        ServoBlue.writeMicroseconds(1620);
        binText = " 2";

    } else if (lRes >= uVals[2] * (1.0 - tol ) && lRes <= uVals[2] * (1.0 + tol )) {
        // Bin 3 position
        ServoRed.writeMicroseconds(900);   
        ServoBlue.writeMicroseconds(1580);
        binText = " 3";

    } else if (lRes >= uVals[3] * (1.0 - tol ) && lRes <= uVals[3] * (1.0 + tol )) {
        // Bin 4 position
        ServoRed.writeMicroseconds(900);   
        ServoBlue.writeMicroseconds(1360);
        binText = " 4";

    } else {
        // Bin 5 position (default)
        ServoRed.writeMicroseconds(828);   
        ServoBlue.writeMicroseconds(1140);
        binText = " 5";
    }

    delay(400);

    // Turn off electromagnet
    digitalWrite(electromagnetPin, LOW);

    ServoMagnet.write(150); // Release the resistor
    delay(300);
    ServoMagnet.write(90); // Reset magnet position
    delay(200);

    // Update OLED display with formatted resistance and bin info
    display.clearDisplay();
    display.setTextSize(2);         // Medium text size
    display.setCursor(0, 0);        // Start at the top-left
    display.println(lRes, 2);  // Show resistance value with 2 decimal places
    display.print(" Bin");
    display.print(binText);        // Print the bin
    display.display();
    delay(100); // Allow time for the display update to be visible
  
}

float getMeasuredResistance() {
  int cOut;
  float delta, deltaBest1 = MAX_ANALOG_VALUE, deltaBest2 = MAX_ANALOG_VALUE;
  float rBest1 = -1, rBest2 = -1, rR, rX = -1;  // Default rX set to -1 to identify if the value wasn't updated properly

  for (byte count = 0; count < NUM_REF_RESISTORS; count++) {
    digitalWrite(rSelPins[0], count & 1);
    digitalWrite(rSelPins[1], count & 2);
    digitalWrite(rSelPins[2], count & 4);

    // Enable the multiplexer and give time for signals to stabilize
    digitalWrite(enableMux, LOW);
    delay(20); // Increased delay to allow signal stabilization

    // Take multiple analog readings and average them
    int analogSum = 0;
    const int numSamples = 5; // Number of samples to average
    for (int i = 0; i < numSamples; i++) {
      analogSum += analogRead(A0);
      delay(5); // Small delay between readings for stability
    }
    cOut = analogSum / numSamples;

    digitalWrite(enableMux, HIGH);  // Disable the multiplexer after reading
    delay(20); // Give a slight delay after reading

    if (cOut < MAX_ANALOG_VALUE && cOut > 0) { // Valid analog read range
      rR = rRef[count] + SWITCH_RESISTANCE;
      rX = (rR * cOut) / (MAX_ANALOG_VALUE - cOut);

      delta = (MAX_ANALOG_VALUE / 2.0 - cOut);
      if (fabs(delta) < fabs(deltaBest1)) {
        deltaBest2 = deltaBest1;
        rBest2 = rBest1;
        deltaBest1 = delta;
        rBest1 = rX;
      } else if (fabs(deltaBest2) > fabs(delta)) {
        deltaBest2 = delta;
        rBest2 = rX;
      }
    }
  }

  if (rBest1 >= 0 && rBest2 >= 0) {
    if (deltaBest1 * deltaBest2 < 0) {
      rX = rBest1 - deltaBest1 * (rBest2 - rBest1) / (deltaBest2 - deltaBest1);
    } else {
      rX = rBest1;
    }
  }

  return rX;
}