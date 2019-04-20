#include <SoftwareSerial.h>;
SoftwareSerial BT(10, 11);
  
# define debug true

#define JoyRX 3
#define JoyRY 2
#define JoyLX 1
#define JoyLY 0

#define JoyRB 6
#define JoyLB 7

#define TopB    2
#define LeftB   4
#define RightB  3
#define BottomB 5

bool modeFlag = false;

int rx;   
int ry;    
int lx; 
int ly; 

int jrb;
int jlb;
int tb;
int lb;
int rb;
int bb;

int LEDPin = 13;

void setup() {
  Serial.begin(9600);
  BT.begin(9600);
  
  pinMode(LEDPin, OUTPUT);
  pinMode(JoyRB, INPUT_PULLUP);  
  pinMode(JoyLB, INPUT_PULLUP);
  pinMode(TopB, INPUT_PULLUP);
  pinMode(RightB, INPUT_PULLUP);
  pinMode(LeftB, INPUT_PULLUP);
  pinMode(BottomB, INPUT_PULLUP);
}

void loop() {
 //Read Joy Sticks axis
  rx = analogRead(JoyRX);   
  ry = analogRead(JoyRY);
  lx = analogRead(JoyLX);
  ly = analogRead(JoyLY);

  //Read Buttons
  jrb = digitalRead(JoyRB);
  jlb = digitalRead(JoyLB);
  tb = digitalRead(TopB);
  lb = digitalRead(LeftB);
  rb = digitalRead(RightB);
  bb = digitalRead(BottomB);

  //Send Data
  BT.print('s');
  BT.print(rx);
  BT.print(",");
  BT.print(ry);
  BT.print(",");
  BT.print(lx);
  BT.print(",");
  BT.print(ly);
  BT.print(",");
  BT.print(jlb);
  BT.print(",");
  BT.print(jrb);
  BT.print(",");
  BT.print(tb);
  BT.print(",");
  BT.print(lb);
  BT.print(",");
  BT.print(rb);
  BT.print(",");
  BT.print(bb);

  if(debug)
  {
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

  
  delay(100);                //Give time to execute

}
