#include <Servo.h>
// ramp lib
#include <Ramp.h> 

Servo servo1;
Servo servo2;
Servo servo3;

int servo1Offset = 2150;      // gripper closed - max 2200
int servo2Offset = 1120;      // shoulder back - min 600  (1120 mid)
int servo3Offset = 1180;      // elbow back - max 2180 (1180 mid)

// moving the foot forwards or backwards in the side plane
float shoulderAngle2;
float shoulderAngle2a;
float shoulderAngle2Degrees;
float shoulderAngle2aDegrees;
float z2;
float x;

// side plane of individual leg only
#define lowerLength 110     
#define upperLength 110
float armLength;
float z;
float shoulderAngle1;
float shoulderAngleDegrees;
float shoulderAngle1a;   
float elbowAngle;  
float elbowAngleDegrees;

// interpolation targets
float zTarget;
float xTarget;

// output scaling
float shoulderMs;
float shoulderMs2;
float elbowMs;

int ball;     // ball proximity

// wheel encoder interrupts

#define encoder0PinA 2      // encoder 1
#define encoder0PinB 3

volatile long encoder0Pos = 0;    // encoder 1
long encoder0Target = 0;

unsigned long currentMillis;
unsigned long previousMillis;

long previousStepMillis = 0;    // set up timers
int stepFlag = 0;

//*****************************
class Interpolation {  
public:
    rampInt myRamp;
    int interpolationFlag = 0;
    int savedValue;    

    int go(int input, int duration) {

      if (input != savedValue) {   // check for new data
          interpolationFlag = 0;
      }
      savedValue = input;          // bookmark the old value  
    
      if (interpolationFlag == 0) {                                        // only do it once until the flag is reset
          myRamp.go(input, duration, LINEAR, ONCEFORWARD);              // start interpolation (value to go to, duration)
          interpolationFlag = 1;
      }
    
      int output = myRamp.update();               
      return output;
    }
};    // end of class

Interpolation interpX;        // interpolation objects
Interpolation interpZ;
//*****************************

void setup() {

  servo1.attach(5);         // gripper
  servo2.attach(4);         // shoulder
  servo3.attach(7);         // elbow

  servo1.writeMicroseconds(servo1Offset);    // gripper
  servo2.writeMicroseconds(servo2Offset);    // shoulder
  servo3.writeMicroseconds(servo3Offset);    // elbow

  pinMode(encoder0PinA, INPUT_PULLUP);    // encoder pins
  pinMode(encoder0PinB, INPUT_PULLUP);

  pinMode(6, OUTPUT);         // PWM motor output piuns
  pinMode(11, OUTPUT);

  pinMode(8, INPUT_PULLUP);   // proximity sensor input

  attachInterrupt(0, doEncoderA, CHANGE);
  attachInterrupt(1, doEncoderB, CHANGE);

  Serial.begin(115200);

  zTarget = 100;      // set default position for arm
  xTarget = 50;


}

void loop() {

  currentMillis = millis();

    if (currentMillis - previousMillis >= 10) {  // start timed loop
          previousMillis = currentMillis;

          Serial.println(encoder0Pos);

          ball = digitalRead(8);      // detect ball

          // step sequencer

          if (ball == 0 && stepFlag == 0) {
             stepFlag = 1;
          }
        
          if (stepFlag == 1 && currentMillis - previousStepMillis >= 1000) {
              zTarget = -115;
              xTarget = 110;
              servo1.writeMicroseconds(servo1Offset -300);    // gripper open
              encoder0Target = 0;
              previousStepMillis = currentMillis;
              stepFlag = 2;
          }         
          else if (stepFlag == 2 && currentMillis - previousStepMillis >= 1200) {
              servo1.writeMicroseconds(servo1Offset);    // gripper closed
              previousStepMillis = currentMillis;
              stepFlag = 3;                  
          }
          else if (stepFlag == 3 && currentMillis - previousStepMillis >= 1000) {
              zTarget = 100;
              xTarget = 50;
              previousStepMillis = currentMillis;
              stepFlag = 4; 
          }
          else if (stepFlag == 4 && currentMillis - previousStepMillis >= 500) {
              encoder0Target = 20000;
              previousStepMillis = currentMillis;
              stepFlag = 6; 
          }
          else if (stepFlag == 6 && currentMillis - previousStepMillis >= 4000) {
              zTarget = 100;
              xTarget = 170;
              previousStepMillis = currentMillis;
              stepFlag = 7; 
          }
          else if (stepFlag == 7 && currentMillis - previousStepMillis >= 1500) {
              servo1.writeMicroseconds(servo1Offset - 300);    // gripper open
              stepFlag = 8; 
          }
          else if (stepFlag == 8 && currentMillis - previousStepMillis >= 2500) {
              encoder0Target = 0;
              zTarget = 100;
              xTarget = 50;
              servo1.writeMicroseconds(servo1Offset);    // gripper closed
              previousStepMillis = currentMillis;
              stepFlag = 9; 
          }
          else if (stepFlag == 9 && currentMillis - previousStepMillis >= 4000) {
              //wait to get back to the start before triggering again
              servo1.writeMicroseconds(servo1Offset);    // gripper closed
              previousStepMillis = currentMillis;
              stepFlag = 0; 
          }

          // end of step sequencer

          // start interpolation
          
          z = interpZ.go(zTarget,1000);
          x = interpX.go(xTarget,1000);

          // *** Inverse Kinematics ***
      
          // calculate modification to shoulder angle and arm length
      
          shoulderAngle2a = atan(z/x);
          shoulderAngle2aDegrees = shoulderAngle2a * (180/PI);    // degrees
          shoulderAngle2 = shoulderAngle2a - 0.7853908;  // take away the default 45' offset (in radians)
          shoulderAngle2Degrees = shoulderAngle2 * (180/PI);    // degrees
          shoulderMs2 = shoulderAngle2Degrees * 11;
          
          z2 = x/cos(shoulderAngle2a);     // calc new arm length to feed to the next bit of code below
      
          // ****************
      
          // calculate arm length based on upper/lower length and elbow and shoulder angle
          shoulderAngle1a = (sq(upperLength) + sq(z2) - sq(lowerLength)) / (2 * upperLength * z2);
          shoulderAngle1 = acos(shoulderAngle1a);     // radians
          elbowAngle = PI - (shoulderAngle1 *2);       // radians
      
          // calc degrees from angles
          shoulderAngleDegrees = shoulderAngle1 * (180/PI);    // degrees
          elbowAngleDegrees = elbowAngle * (180/PI);              // degrees 
      
          // calc milliseconds PWM to drive the servo.
          shoulderMs = shoulderAngleDegrees * 11;
          elbowMs = elbowAngleDegrees * 11;

          // *** end of Inverse Kinematics ***
      
          // write to servos, remove 45' and 90' offsets from arm default position
          servo2.writeMicroseconds(servo2Offset - (shoulderMs - 480) - shoulderMs2);    // shoulder
          servo3.writeMicroseconds(servo3Offset + (elbowMs - 1000));    // elbow     
      
          // drive linear slider motor

          if (encoder0Target < encoder0Pos - 150) {   // allow for a deadzone of 100 encoder counts
              analogWrite(6, 255);
              analogWrite(11, 0);
          }
          else if (encoder0Target > encoder0Pos + 150) {   // allow for a deadzone of 100 encoder counts
              analogWrite(6, 0);
              analogWrite(11, 255);
          }

          else {
            analogWrite(6, 0);
            analogWrite(11, 0);
          }
          



  
    }  // end of timed loop

}   //end if main loop



void doEncoderA(){  

  // look for a low-to-high on channel A
  if (digitalRead(encoder0PinA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == LOW) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinB) == HIGH) {   
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
 
}

void doEncoderB(){  

  // look for a low-to-high on channel B
  if (digitalRead(encoder0PinB) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(encoder0PinA) == HIGH) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinA) == LOW) {   
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
  

}
