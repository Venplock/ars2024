#define NUM_REF_RESISTORS 8
#define NUM_SELECT_PINS   3
#define MAX_ANALOG_VALUE  1023
#define SWITCH_RESISTANCE 4.5

// Note that the MAX4617 8:1 Mutiplexer routes signals from X0, x1, ...x7 to X.
// It uses 3 select lines A, B and C to choose one of the signals.
// X0 is the least significant (ABC=0b000), while X7 is most significant (ABC=0b111)

// Reference Resistor values      x0    x1   x2     x3     x4       x5       x6       x7
float rRef[NUM_REF_RESISTORS] = {49.9, 100, 1000, 10000, 100000, 1000000, 4990000, 10000000};

// Multiplexer select pins              A  B  C
const byte rSelPins[NUM_SELECT_PINS] = {5, 4, 3};

const byte enableMux = 6; // 1 = no connection, 0 = one of eight signals connected

void setup()
{
  pinMode(enableMux, OUTPUT);
  digitalWrite(enableMux, HIGH);      // disable all switches
  
  for (int i = 0; i < NUM_SELECT_PINS; i++)
  {
    pinMode(rSelPins[i], OUTPUT);     // Mux select pins configured as outputs
    digitalWrite(rSelPins[i], HIGH);  // select the highest Rref
  }

  Serial.begin(9600);
  Serial.println("\nStarting ArduinOhmmeter...");
}

// This function scales the resistor value, so that it
// can be expressed in Ohms, KOhms, MOhms or GOhms.
// It then ensures that 3 digits of precision are 
// present in the final result.
char ScaleToMetricUnits(float *prVal, char fStr[])
{
  char unit;

  if (*prVal < 1000)
  {
    unit = ' ';
  }
  else if (*prVal >= 1000 && *prVal < 1000000)
  {
    *prVal /= 1000;
    unit = 'K';
  }
  else if (*prVal >= 1000000 && *prVal < 1000000000)
  {
    *prVal /= 1000000;
    unit = 'M';
  }
  else
  {
    *prVal /= 1000000000;
    unit = 'G';
  }

  // Cycle the decimal number in prVal until its whole number is 0.
  // Note that counter 'k' is decremented from 2 to 0 (inclusive),
  // which gives us the 3-digit precision we're looking for.
  for (int k=2, s=10; k >= 0; k--, s*=10)
  {
    if ((int)(*prVal) / s == 0)
    {
      dtostrf(*prVal, 4, k, fStr); // convert the float result to a string
      break;
    }
  }

  return unit;
}

void loop()
{
  int cOut;
  float delta, deltaBest1 = MAX_ANALOG_VALUE, deltaBest2 = MAX_ANALOG_VALUE;
  float rBest1 = -1, rBest2 = -1, rR, rX;
  char unit = 0, fStr[16];

  for (byte count = 0; count < NUM_REF_RESISTORS; count++)
  {
    // Set the Mux select pins to switch in one Rref at a time.
    // count=0: Rref0 (49.9 ohms), count=1: Rref1 (100 ohms), etc...
    digitalWrite(rSelPins[0], count & 1); // C: least significant bit
    digitalWrite(rSelPins[1], count & 2); // B:
    digitalWrite(rSelPins[2], count & 4); // A: most significant bit
    
    digitalWrite(enableMux, LOW);       // enable the selected reference resistor
    delay(count + 1);                   // delay 1ms for Rref0, 2ms for Ref1, etc...
    cOut = analogRead(A0);              // convert analog voltage Vx to a digital value
    digitalWrite(enableMux, HIGH);      // disable the selected reference resistor
    delay(NUM_REF_RESISTORS - count);   // delay 8ms for Rref0, 7ms for Ref1, etc...

    // Work only with valid digitized values
    if (cOut < MAX_ANALOG_VALUE)
    {
      // Identify the Rref value being used and compute Rx based on formula #2.
      // Note how Mux's internal switch resistance is added to Rref. 
      rR = rRef[count] + SWITCH_RESISTANCE; 
      rX = (rR * cOut) / (MAX_ANALOG_VALUE - cOut);

      // Compute the delta and track the top two best delta and Rx values
      delta = (MAX_ANALOG_VALUE / 2.0 - cOut);
      if (fabs(delta) < fabs(deltaBest1))
      {
        deltaBest2 = deltaBest1;
        rBest2 = rBest1;
        deltaBest1 = delta;
        rBest1 = rX;
      }
      else if (fabs(deltaBest2) > fabs(delta))
      {
        deltaBest2 = delta;
        rBest2 = rX;
      }
    }
  }

  // Make sure there are at least two good samples to work with
  if (rBest1 >= 0 && rBest2 >= 0)
  {
    // Check to see if need to interpolate between the two data points.
    // Refer to the documentation for details regarding this.
    if (deltaBest1 * deltaBest2 < 0)
    {
      rX = rBest1 - deltaBest1 * (rBest2 - rBest1) / (deltaBest2 - deltaBest1); // Yes
    }
    else
    {
      rX = rBest1;  // No. Just use the best value
    }

    // Convert the scaled float result to string and extract the units
    unit = ScaleToMetricUnits(&rX, fStr);
  }

  Serial.print(rX) ; 
  Serial.print(" ") ; 
  Serial.print(unit) ; 
  Serial.print('\n') ; 

  delay(1000);
}