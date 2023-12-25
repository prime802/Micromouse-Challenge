# MICROMOUSE

## Introduction
MICROMOUSE is an autonomous robot designed for the International Micromouse Challenge 2022 at IIT Bombay. The challenge involves building a robot capable of rapidly navigating a labyrinth to reach the center in the shortest time possible.

### Project Highlights
- **Techfest, IIT Bombay**
- **Team Size:** 4
- **Duration:** Oct 2022 - Dec 2022


## Overview
Designed and built an autonomous bot with 18cm square cells set over a 16 x 16 grid, achieving the least traversal time to the goal in the maze-solving challenge.

## Technical Details
- Fabricated a custom PCB to boost the bot's performance through integrated electronics.
- Incorporated six sharp infrared (IR) sensors for superior data accuracy during maze exploration.
- Executed maze-solving algorithms, such as flood fill and wall-following, to determine the quickest and most efficient route to the maze's center.
- Implemented PID control for precise and smooth movement, optimizing the robot's path-finding capabilities.

## Code Overview
The project utilizes Arduino Nano and two tires for a compact and speedy design. Given is a snippet of the Arduino IDE code used:

#include <Encoder.h>
int dir;
volatile int lastEncoded = 0; // Here updated value of encoder store.
volatile long encoderValue = 0; // Raw encoder value
Encoder knobLeft(2, 4);
Encoder knobRight(3, 5);

#define STOP 0
#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4

float P = 5;
float D = 15;
float I = 0;
float oldErrorP ;
float totalError ;
int offset = 4;

int wall_threshold = 5;
//int left_threshold = 10 ;
//int right_threshold = 10 ;
int front_threshold = 6;

boolean frontwall ;
boolean leftwall ;
boolean rightwall ;
boolean first_turn ;
boolean rightWallFollow ;
boolean leftWallFollow ;

int en1 = 7 ;
int en2 = 6 ;

int en3 = 8 ;
int en4 = 9 ;

int enA = 10;
int enB = 11 ;

int baseSpeed = 200;

int RMS ;
int LMS ;


float oldLeftSensor, oldRightSensor, leftSensor, rightSensor, frontSensor, oldFrontSensor, lSensor, rSensor, fSensor;
//ENCODER

//SIDE SENSORS
const int sensorPin[] = {A2,A1,A0};
float distance1[3];
const int AVERAGE_OF =50;
const float MCU_VOLTAGE = 5.0;

void setup()
{
  Serial.begin(9600);
  for(int i=6;i<=11;i++){
  pinMode(i, OUTPUT);
  }

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(5, INPUT);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(12,OUTPUT);
  digitalWrite(12,HIGH);
  first_turn=false ;
  rightWallFollow = false ;
  leftWallFollow = false ;

}

long positionLeft  = -999;
long positionRight = -999;
void readDistance(int sensor)
{
      float voltage_temp_average=0;
     
      for(int i=0; i < AVERAGE_OF; i++)
    {
      int sensorValue = analogRead(sensorPin[sensor] );
      delay(1);      
      voltage_temp_average +=sensorValue * (MCU_VOLTAGE / 1023.0);

    }
     voltage_temp_average /= AVERAGE_OF;

  distance1[sensor] = 33.9 + -69.5*(voltage_temp_average) + 62.3*pow(voltage_temp_average,2) + -25.4*pow(voltage_temp_average,3) + 3.83*pow(voltage_temp_average,4);
 
}
void loop() {
  ReadSensors();
  walls();

  if ( first_turn == false ) {

    pid_start();

  }
  else if (leftWallFollow == true ) {

    PID(true) ;

  }
  else if (rightWallFollow == true ) {
    PID(false) ;
  }


//   if (leftwall == true && rightwall == false && frontwall == true ) {

//     // turnright();
//     PID(false) ;

//     if ( first_turn == false ) {

//       //      right_threshold = right_threshold - offset ;
//       //      left_threshold = left_threshold + offset ;


//       first_turn = true ;
//       rightWallFollow = true ;
// //      
// //      digitalWrite(led2 , LOW );
// //      digitalWrite(led1 ,HIGH );
//     }
//   }
//    if (leftwall == false && rightwall == true && frontwall == true ) {

//     //  turnleft();
//     PID(true) ;

//     if ( first_turn == false ) {

//       //      left_threshold = left_threshold - offset ;
//       //      right_threshold = right_threshold + offset ;

//       first_turn = true ;
//       leftWallFollow = true ;
// //      digitalWrite(LED , HIGH);
       
//     }
//   }
//    if ( leftSensor == 0 || leftSensor > 100 && rightSensor == 0 || rightSensor > 100 && frontSensor == 0 || frontSensor > 100 ) {

//     setDirection(STOP);
//   }



  // read sensors & print the result to the serial monitor //


  // Serial.print(" Left : ");
  // Serial.print(leftSensor);
  // Serial.print(" cm ");
  // Serial.print(" Right : ");
  // Serial.print(rightSensor);
  // Serial.print(" cm ");
  // Serial.print(" Front : ");
  // Serial.print(frontSensor);
  // Serial.println(" cm ");

  //measure error & print the result to the serial monitor


  // Serial.print("error=");
  // Serial.println(totalError);
  long newLeft, newRight;
  newLeft = knobLeft.read();
  newRight = -1*knobRight.read();
   Serial.print("Left =  ");
    Serial.print(newLeft);
    Serial.print("   Right =  ");
    Serial.print(newRight);
    Serial.print("Left =  ");
    Serial.println();
  if (newLeft != positionLeft || newRight != positionRight) {
    // Serial.print("Left = ");
    // Serial.print(newLeft);
    // Serial.print(", Right = ");
    // Serial.print(newRight);
    // Serial.println();
    positionLeft = newLeft;
    positionRight = newRight;
  }
  if (Serial.available()) {
    Serial.read();
    // Serial.println("Reset both knobs to zero");
    knobLeft.write(0);
    knobRight.write(0);
  }

}
//--------------------------------- direction control ---------------------------------//

void setDirection(int dir) {

  if ( dir == FORWARD ) {
    digitalWrite(en1, LOW);   // Left wheel forward
    digitalWrite(en2, HIGH);
    digitalWrite(en3, LOW);  // Right wheel forward
    digitalWrite(en4, HIGH);
  }
  else if ( dir == LEFT ) {
    digitalWrite(en1, HIGH);   // Left wheel forward
    digitalWrite(en2, LOW );
    digitalWrite(en3, LOW );  // Right wheel forward
    digitalWrite(en4, HIGH);
  }
  else if ( dir == RIGHT ) {
    digitalWrite(en1, LOW);   // Left wheel forward
    digitalWrite(en2, HIGH);
    digitalWrite(en3, HIGH);  // Right wheel forward
    digitalWrite(en4, LOW);
  }
  else if ( dir == STOP ) {
    digitalWrite(en1,HIGH);   // Left wheel forward
    digitalWrite(en2, HIGH);
    digitalWrite(en3, HIGH);  // Right wheel forward
    digitalWrite(en4, HIGH);
  }
  else if ( dir == BACKWARD ) {
    digitalWrite(en1, HIGH );   // Left wheel forward
    digitalWrite(en2, LOW );
    digitalWrite(en3, HIGH );  // Right wheel forward
    digitalWrite(en4, LOW );
  }
}
//---------------------------------------------------------------------------//


//--------------------------------- Sensors ---------------------------------//

void ReadSensors() {
  readDistance(2);
  readDistance(1);
  readDistance(0);
  lSensor = distance1[0];
  rSensor = distance1[1];
  fSensor = distance1[2];


  leftSensor = (lSensor + oldLeftSensor) / 2; //average distance between old & new readings to make the change smoother
  rightSensor = (rSensor + oldRightSensor) / 2;
  frontSensor = (fSensor + oldFrontSensor) / 2;


  oldLeftSensor = leftSensor; // save old readings for movment
  oldRightSensor = rightSensor;
  oldFrontSensor = frontSensor;

}

void pid_start() {

  //ReadSensors()

  float errorP = leftSensor - rightSensor ;
  float errorD = errorP - oldErrorP;
  float errorI = (2.0 / 3.0) * errorI + errorP ;

  totalError = P * errorP + D * errorD + I * errorI ;

  oldErrorP = errorP ;


  RMS = baseSpeed + totalError ;
  LMS = baseSpeed - totalError ;

  //  if(RMS < -255) RMS = -255; if(RMS > 255)RMS = 255 ;
  //  if(LMS < -255) LMS = -255;  if(LMS > 255)LMS = 255 ;


  if (RMS < 0) {

    RMS = map(RMS , 0 , -255, 0, 255);

    analogWrite(enA , RMS);
    analogWrite(enB , LMS);

    setDirection(RIGHT);

  }
  else if (LMS < 0) {
    LMS = map(LMS , 0 , -255, 0, 255);


    analogWrite(enA , RMS);
    analogWrite(enB , LMS);

    setDirection(LEFT);
  }
  else {

    analogWrite(enA , RMS);
    analogWrite(enB , LMS);

    setDirection(FORWARD);
  }



}


//----------------------------- wall follow  control -------------------------------//

void PID( boolean left ) {

  if (left == true ) {

    float errorP = leftSensor - rightSensor - offset ;
    float errorD = errorP - oldErrorP;
    float errorI = (2.0 / 3) * errorI + errorP ;


    totalError = P * errorP + D * errorD + I * errorI ;

    oldErrorP = errorP ;


    RMS = baseSpeed + totalError ;
    LMS = baseSpeed - totalError ;

    //  if(RMS < -255) RMS = -255; if(RMS > 255)RMS = 255 ;
    //  if(LMS < -255) LMS = -255;  if(LMS > 255)LMS = 255 ;


    if (RMS < 0) {

      RMS = map(RMS , 0 , -255, 0, 255);

      analogWrite(enA , RMS);
      analogWrite(enB , LMS);

      setDirection(RIGHT);

    }
    else if (LMS < 0) {
      LMS = map(LMS , 0 , -255, 0, 255);


      analogWrite(enA , RMS);
      analogWrite(enB , LMS);

      setDirection(LEFT);
    }
    else {

      analogWrite(enA , RMS);
      analogWrite(enB , LMS);

      setDirection(FORWARD);
    }

  }
  else {

    float errorP = leftSensor - rightSensor + offset ;
    float errorD = errorP - oldErrorP;
    float errorI = (2.0 / 3) * errorI + errorP ;

    totalError = P * errorP + D * errorD + I * errorI ;

    oldErrorP = errorP ;


    RMS = baseSpeed + totalError ;
    LMS = baseSpeed - totalError ;

    //  if(RMS < -255) RMS = -255; if(RMS > 255)RMS = 255 ;
    //  if(LMS < -255) LMS = -255;  if(LMS > 255)LMS = 255 ;


    if (RMS < 0) {

      RMS = map(RMS , 0 , -255, 0, 255);

      analogWrite(enA , RMS);
      analogWrite(enB , LMS);

      setDirection(RIGHT);

    }
    else if (LMS < 0) {
      LMS = map(LMS , 0 , -255, 0, 255);


      analogWrite(enA , RMS);
      analogWrite(enB , LMS);

      setDirection(LEFT);
    }
    else {

      analogWrite(enA , RMS);
      analogWrite(enB , LMS);

      setDirection(FORWARD);
    }

  }

}

//--------------------------- wall detection --------------------------------//

void walls() {


  if ( leftSensor < wall_threshold ) {
    leftwall = true ;
  }
  else {
    leftwall = false ;
  }


  if ( rightSensor < wall_threshold ) {
    rightwall = true ;
  }
  else {
    rightwall = false ;


  } if ( frontSensor < front_threshold ) {
    frontwall = true ;
  }
  else {
    frontwall = false ;
  }

}



//---------------------------------------------------------------------------//

void turnright() {


  LMS = baseSpeed ;

  RMS = LMS * rightSensor / ( rightSensor + 11 ) ;


}


