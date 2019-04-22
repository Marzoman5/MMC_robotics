/*
Brim

45/2/18 by Gaston Marzoratti

The following code supports
-Bluetooth Control
-Two Motors
*/


//Define the Pins

//Motor 1
int pinAIN1 = 3; //Direction
int pinAIN2 = 4; //Direction
int pinPWMA = 5; //Speed

//Motor 2
int pinBIN1 = 8; //Direction
int pinBIN2 = 7; //Direction
int pinPWMB = 6; //Speed

//Standby
int pinSTBY = 2;

//Constants to help remember the parameters
static boolean turnCW = 0;  //for motorDrive function
static boolean turnCCW = 1; //for motorDrive function
static boolean motor1 = 0;  //for motorDrive, motorStop, motorBrake functions
static boolean motor2 = 1;  //for motorDrive, motorStop, motorBrake functions



# define debug true


//#include <SoftwareSerial.h> //Creates serial ports for Digital IO
//SoftwareSerial BT(11, 12);  //Create Serial Port 11-TX, 12-RX
                            //for Bluetooth Module

#define RightDriveMotor 2
#define LeftDriveMotor  1

#define Claw            9

#define TurretMotor     10
#define ElbowMotor      2
#define WristMotor      1
#define Dumper          3

#include <Servo.h>
#include <Wire.h>  //arduino to motorshield
#include<Adafruit_MotorShield.h>
#include"utility/Adafruit_MS_PWMServoDriver.h"

Servo claw;  // create servo object to control a servo 
Servo turretMotor;

//Adafruit_MotorShield AFMS_bottom = Adafruit_MotorShield(0x60);  // Create Motorshield
Adafruit_MotorShield AFMS_top = Adafruit_MotorShield(0x61);  // Create Motorshield

//Adafruit_DCMotor *leftMotor = AFMS_bottom.getMotor(LeftDriveMotor);      // Create Motors
//Adafruit_DCMotor *rightMotor = AFMS_bottom.getMotor(RightDriveMotor);

Adafruit_DCMotor *elbowMotor = AFMS_top.getMotor(ElbowMotor);      // Create Motors
Adafruit_DCMotor *wristMotor = AFMS_top.getMotor(WristMotor);
Adafruit_DCMotor *dumper= AFMS_top.getMotor(Dumper);

//Joystick Values
int rx =508;    // variable to read the value from the analog pin
int ry=508;    // variable to read the value from the analog pin
int lx=508;    // variable to read the value from the analog pin
int ly=508;    // variable to read the value from the analog pin
int trAg=90;

//Button States
bool jrb=1;
bool jlb=1;

bool tb=1;
bool lb=1;
bool rb=1;
bool bb=1;

int motorSpeed;

void setup() 
{

  pinMode(pinPWMA, OUTPUT);
  pinMode(pinAIN1, OUTPUT);
  pinMode(pinAIN2, OUTPUT);

  pinMode(pinPWMB, OUTPUT);
  pinMode(pinBIN1, OUTPUT);
  pinMode(pinBIN2, OUTPUT);

  pinMode(pinSTBY, OUTPUT);
  

   Serial.begin(9600); // Begin Serial for debug
//   BT.begin(9600);      // Begin Serial for Bluetooth module

   AFMS_top.begin();         // Connect to the motor
/*   AFMS_bottom.begin();         // Connect to the motor
   
   rightMotor->setSpeed(100);  // 10 rpm
   rightMotor->run(FORWARD);         // turn on motor
   rightMotor->run(RELESE);

   leftMotor->setSpeed(100);  // 10 rpm
   leftMotor->run(FORWARD);          // turn on motor
   leftMotor->run(RELEASE);
*/
   elbowMotor->setSpeed(100);  // 10 rpm
   elbowMotor->run(FORWARD);          // turn on motor
   elbowMotor->run(RELEASE);         
   
   wristMotor->setSpeed(100);  // 10 rpm
   wristMotor->run(FORWARD);          // turn on motor
   wristMotor->run(RELEASE);       
   
   dumper->setSpeed(100);  // 10 rpm
   dumper->run(FORWARD);          // turn on motor
   dumper->run(RELEASE);

   claw.attach(Claw);  // attaches the servo on pin 9 to the servo object 
   claw.write(100);
   turretMotor.attach(TurretMotor);
   turretMotor.write(trAg);
//   TIMSK0=0; too stop jitter may not need it

//   servoC.attach(ServoClaw);
 
}

void loop() 
{
  if (Serial.available())  //Do this if BT Command found
  {
    if(debug){Serial.println("looking");}

    if ('s' == Serial.read())
    { //Read data
      getData();

      if(debug){
      Serial.print("Start--");
      Serial.print("s");
      Serial.print("--");
      Serial.print(rx);
      Serial.print("--");
      Serial.print(ry);
      Serial.print("--");
      Serial.print(lx);
      Serial.print("--");
      Serial.print(ly);
      Serial.print("--");
      Serial.print(jlb);
      Serial.print("--");
      Serial.print(jrb);
      Serial.print("--");
      Serial.print(tb);
      Serial.print("--");
      Serial.print(lb);
      Serial.print("--");
      Serial.print(rb);
      Serial.print("--");
      Serial.print(bb);
      Serial.println("--Done");
    }
    
    }//End Get Data
  } //End Bluetooth Data


  //Set Right Motor speed
  if (ry < 250)
  {

    motorDrive(motor1, turnCW, 255);

    /*
    rightMotor->run(BACKWARD);
    motorSpeed = map(ry, 250, 0, 0, 255);  //scale to 0 and 255
    rightMotor->setSpeed(motorSpeed)0;
    */
    Serial.print("back");
    Serial.print(motorSpeed);
  }
  else if (ry > 850)  //go forward
  {

  motorDrive(motor1, turnCCW, 255);

    /*
    rightMotor->run(FORWARD);
    motorSpeed = map(ry, 850, 1023, 0, 255);  //scale to 0 and 255
    rightMotor->setSpeed(motorSpeed);
    */
  }
  else
  {

    motorStop(motor1);
    /*
    motorSpeed = 0;
    rightMotor->run(RELEASE);
    motorSpeed = 0;
    */
  }

  //Set Left Motor speed
  if (ly < 250)
  {
    motorDrive(motor2, turnCCW, 255);
    /*
    leftMotor->run(BACKWARD);
    motorSpeed = map(ly, 250, 0, 0, 255);  //scale to 0 and 255
    leftMotor->setSpeed(motorSpeed);
    */
  }
  else if (ly > 850)  //go forward
  {
    motorDrive(motor2, turnCW, 255);
    /*
    leftMotor->run(FORWARD);
    motorSpeed = map(ly, 850, 1023, 0, 255);  //scale to 0 and 255
    leftMotor->setSpeed(motorSpeed);
    */
  }
  else
  {
    motorStop(motor2);
    /*
    motorSpeed = 0;
    leftMotor->run(RELEASE);
    motorSpeed = 0;
    */
  }


  //Set Turret speed
  if (rx < 200)
  {
    trAg = (trAg-2);
    trAg = constrain(trAg, 0, 180);
    turretMotor.write(trAg);
    //delay(100);
  }
  else if (rx > 812)  //go forward
  {
    trAg = (trAg+2);
    trAg = constrain(trAg, 0, 180);
    turretMotor.write(trAg);
    //delay(100);
  }
  /*else  
  {
    trAg = constrain(trAg, 0, 180);
    turretMotor.write(trAg);
    //delay(100);
  }
*/
  Serial.println(lx);
  //Set claw speed
  if (lx < 200)
  {
    claw.write(80);
    //delay(100);
    Serial.println("Open");
   
  }
  else if (lx > 800)  //close
  {
   claw.write(100);
   //delay(100);
   Serial.println("Close");

  }
    
  //Set Elbow speed
  if (bb == 0)
  {
    elbowMotor->run(FORWARD);
    elbowMotor->setSpeed(255);
    Serial.println("Elbow Up");
  }
  else if (tb==0)  //go forward
  {
    elbowMotor->run(BACKWARD);
    elbowMotor->setSpeed(200);
  }
  else
  {
    elbowMotor->run(RELEASE);
  }

  //Set wrist speed
  if (rb == 0)
  {
    wristMotor->run(FORWARD);
    wristMotor->setSpeed(250);
  }
  else if (lb==0)  //go forward
  {
    wristMotor->run(BACKWARD);
    wristMotor->setSpeed(250);
  }
  else
  {
    wristMotor->run(RELEASE);
  }

  //Set dumper speed
  if (jrb==0)
  {
    dumper->run(BACKWARD);
    dumper->setSpeed(50);
  }
  else if (jlb==0)  //go forward
  {
    dumper->run(FORWARD);
    dumper->setSpeed(50);
  }
  else
  {
    dumper->run(RELEASE);
  }





}  //end of main loop


void getData()
{
      //Read data
    rx = Serial.parseInt();
    ry = Serial.parseInt();
    lx = Serial.parseInt();
    ly = Serial.parseInt();
    jlb = Serial.parseInt();
    jrb = Serial.parseInt();
    tb = Serial.parseInt();
    lb = Serial.parseInt();    
    rb = Serial.parseInt();
    bb = Serial.parseInt();
}


void motorDrive(boolean motorNumber, boolean motorDirection, int motorSpeed)
{
  /*
  This Drives a specified motor, in a specific direction, at a specified speed:
    - motorNumber: motor1 or motor2 ---> Motor 1 or Motor 2
    - motorDirection: turnCW or turnCCW ---> clockwise or counter-clockwise
    - motorSpeed: 0 to 255 ---> 0 = stop / 255 = fast
  */

  boolean pinIn1;  //Relates to AIN1 or BIN1 (depending on the motor number specified)

 
//Specify the Direction to turn the motor
  //Clockwise: AIN1/BIN1 = HIGH and AIN2/BIN2 = LOW
  //Counter-Clockwise: AIN1/BIN1 = LOW and AIN2/BIN2 = HIGH
  if (motorDirection == turnCW)
    pinIn1 = HIGH;
  else
    pinIn1 = LOW;

//Select the motor to turn, and set the direction and the speed
  if(motorNumber == motor1)
  {
    digitalWrite(pinAIN1, pinIn1);
    digitalWrite(pinAIN2, !pinIn1);  //This is the opposite of the AIN1
    analogWrite(pinPWMA, motorSpeed);
  }
  else
  {
    digitalWrite(pinBIN1, pinIn1);
    digitalWrite(pinBIN2, !pinIn1);  //This is the opposite of the BIN1
    analogWrite(pinPWMB, motorSpeed);
  }
   
 

//Finally , make sure STBY is disabled - pull it HIGH
  digitalWrite(pinSTBY, HIGH);

}

void motorBrake(boolean motorNumber)
{
/*
This "Short Brake"s the specified motor, by setting speed to zero
*/

  if (motorNumber == motor1)
    analogWrite(pinPWMA, 0);
  else
    analogWrite(pinPWMB, 0);
   
}


void motorStop(boolean motorNumber)
{
  /*
  This stops the specified motor by setting both IN pins to LOW
  */
  if (motorNumber == motor1) {
    digitalWrite(pinAIN1, LOW);
    digitalWrite(pinAIN2, LOW);
  }
  else
  {
    digitalWrite(pinBIN1, LOW);
    digitalWrite(pinBIN2, LOW);
  } 
}


void motorsStandby()
{
  /*
  This puts the motors into Standby Mode
  */
  digitalWrite(pinSTBY, LOW);
}
