/*
Brim

45/2/18 by Gaston Marzoratti

The following code supports
-Bluetooth Control
-Two Motors
*/


# define debug true


#include <SoftwareSerial.h> //Creates serial ports for Digital IO
SoftwareSerial BT(11, 12);  //Create Serial Port 11-TX, 12-RX
                            //for Bluetooth Module

#define RightDriveMotor 1
#define LeftDriveMotor  2

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

Adafruit_MotorShield AFMS_bottom = Adafruit_MotorShield(0x60);  // Create Motorshield
Adafruit_MotorShield AFMS_top = Adafruit_MotorShield(0x61);  // Create Motorshield

Adafruit_DCMotor *leftMotor = AFMS_bottom.getMotor(LeftDriveMotor);      // Create Motors
Adafruit_DCMotor *rightMotor = AFMS_bottom.getMotor(RightDriveMotor);

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

   Serial.begin(9600); // Begin Serial for debug
   BT.begin(9600);      // Begin Serial for Bluetooth module

   AFMS_top.begin();         // Connect to the motor
   AFMS_bottom.begin();         // Connect to the motor
   
   rightMotor->setSpeed(100);  // 10 rpm
   rightMotor->run(FORWARD);         // turn on motor
   rightMotor->run(RELEASE);

   leftMotor->setSpeed(100);  // 10 rpm
   leftMotor->run(FORWARD);          // turn on motor
   leftMotor->run(RELEASE);

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
   TIMSK0=0; 

//   servoC.attach(ServoClaw);
 
}

void loop() 
{
  if (BT.available())  //Do this if BT Command found
  {
    if(debug){Serial.println("looking");}

    if ('s' == BT.read())
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
    rightMotor->run(BACKWARD);
    motorSpeed = map(ry, 250, 0, 0, 255);  //scale to 0 and 255
    rightMotor->setSpeed(motorSpeed);
    Serial.print("back");
    Serial.print(motorSpeed);
  }
  else if (ry > 850)  //go forward
  {
    rightMotor->run(FORWARD);
    motorSpeed = map(ry, 850, 1023, 0, 255);  //scale to 0 and 255
    rightMotor->setSpeed(motorSpeed);
  }
  else
  {
    motorSpeed = 0;
    rightMotor->run(RELEASE);
    motorSpeed = 0;
  }

  //Set Left Motor speed
  if (ly < 250)
  {
    leftMotor->run(BACKWARD);
    motorSpeed = map(ly, 250, 0, 0, 255);  //scale to 0 and 255
    leftMotor->setSpeed(motorSpeed);
  }
  else if (ly > 850)  //go forward
  {
    leftMotor->run(FORWARD);
    motorSpeed = map(ly, 850, 1023, 0, 255);  //scale to 0 and 255
    leftMotor->setSpeed(motorSpeed);
  }
  else
  {
    motorSpeed = 0;
    leftMotor->run(RELEASE);
    motorSpeed = 0;
  }


  //Set Turret speed
  if (rx < 200)
  {
    trAg = (trAg-1);
    trAg = constrain(trAg, 0, 180);
    turretMotor.write(trAg);
    //delay(100);
  }
  else if (rx > 812)  //go forward
  {
    trAg = (trAg+1);
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
  if (tb == 0)
  {
    elbowMotor->run(FORWARD);
    elbowMotor->setSpeed(255);
    Serial.println("Elbow Up");
  }
  else if (bb==0)  //go forward
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
  if (jrb=0)
  {
    dumper->run(BACKWARD);
    dumper->setSpeed(50);
  }
  else if (lb=0)  //go forward
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
    rx = BT.parseInt();
    ry = BT.parseInt();
    lx = BT.parseInt();
    ly = BT.parseInt();
    jlb = BT.parseInt();
    jrb = BT.parseInt();
    tb = BT.parseInt();
    lb = BT.parseInt();    
    rb = BT.parseInt();
    bb = BT.parseInt();
}
