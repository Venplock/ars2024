#include <Servo.h>

//servos
Servo base; //Controls up down
Servo shoulder; //Controls x,y
Servo elbow; //Controls x,y
Servo wrist; //Rotates claw
Servo claw; //Controls claw

const int len = 6; //arm segment length
int x, y, th; //RPI coordinates
int bin; //Which bin to move to
int contDel; //Delay of continuous motor

//setup servos, open serial monitor
void setup() {
  Serial.begin(9600);
  Serial.setTimeout(10);
  base.attach(3);
  shoulder.attach(5);
  elbow.attach(6);
  wrist.attach(9);
  claw.attach(10);
  base.write(180);
  claw.write(0);
  shoulder.write(90);
  elbow.writeMicroseconds(1500);
  wrist.write(90);
  x = y = th = 0;
  bin = 0;
}

void loop() {
  while(Serial.available() > 0)
  {
    x = Serial.parseInt();
    y = Serial.parseInt();
    th= Serial.parseInt();
    char r = Serial.read();
    if(r == '\n'){}
  }
  if(x != 0) {
    //Get the angle to move the resistor to
    moveAngles();
    delay(2000);
    //Moves the arm to grab resistor based on coordinates from RPI
    moveOhmmeter();
    delay(2000);
    //Measures the resistance with Ohmmeter
    //int bin = measure();
    delay(2000);
    //Moves the arm to the bins
    moveBins(bin);
    delay(2000);
  }
  x = y = th = 0;
}

//Moves the arm according to specified action
void moveAngles() {
  contDel = y * 100;
  shoulder.write(x);
  delay(500);
  elbow.writeMicroseconds(1280);
  delay(contDel);
  elbow.writeMicroseconds(1500);
  delay(1000);
  wrist.write(th);
  delay(1000);
  base.write(0);
  delay(1000);
  claw.write(180);
  delay(1000);
  base.write(180);
  delay(1000);

}

//Moves the arm to Ohmmeter
void moveOhmmeter() {
  //Position arm above ohmmeter
  shoulder.write(90);
  delay(1000);
  elbow.writeMicroseconds(1720);
  delay(contDel);
  elbow.writeMicroseconds(1500);
  delay(1000);
  wrist.write(90);
  delay(1000);
  //Lower Arm
  base.write(0);
  delay(1000);
}

//Moves the arm to Bins
void moveBins(int bin) {
  //Raise arm
  base.write(180);
  delay(1000); 
  //Sets shoulder and elbow position according to measured bin
  //Values for other bins will be added once prototype is assembled
  switch(bin) {
    case 1: //Bin 1
      break;
    case 2: //Bin 2
      break;
    case 3: //Bin 3
      break;
    case 4:  //Bin 4
      break;
    default: //Bin 5, reject resistors
      shoulder.write(0);
      delay(1000);
      break;
  }
  //Open claw
  claw.write(0);
  delay(1000);
}
