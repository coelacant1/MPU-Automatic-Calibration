#include <Servo.h>
#include "DirectIO.h"

const int SERVOPIN = 9;

Output<2> ENABLE(LOW);
Output<3> BASESTP(LOW);
Output<4> BASEDIR(HIGH);
Output<5> PITCHSTP(LOW);
Output<6> PITCHDIR(HIGH);
Output<7> ACTUSTP(LOW);
Output<8> ACTUDIR(HIGH);
Output<13> LEDPIN(HIGH);

Servo roll;

float basePosition = 800.0f;
float pitchPosition = 800.0f;
float rollPosition = 800.0f;
float actuPosition = 500.0f * 80.0f;

const long calculationAccuracy = 10;
const int stepTime = 10;//microseconds
const int timeIncrement = stepTime / calculationAccuracy;
const int minimumStepTime = 1;//microseconds

void setup() {
    roll.attach(SERVOPIN, 425, 2225);//initialize servo and set range

    roll.write(90);//Center servo
    
    Serial.begin(2000000);
}

void loop() { 
    transitionGimbal( 90, 90, 90, 500);
    delay(5000);
    transitionGimbal( 180, 180, 180, 900);//100 mm transition
    delay(5000);
    transitionGimbal( 90, 90, 90, 500);
    delay(5000);
    transitionGimbal( 0, 0, 0, 100);//100 mm transition
    delay(5000);
}

void transitionGimbal(float y, float p, float r, float a){
  Serial.println("transition");
  y = (800.0f / 90.0f) * y;//0.1125 degrees
  p = (800.0f / 90.0f) * p;
  r = (800.0f / 90.0f) * r;//scale servo output to stepper range moving at 0.1125 degree increments
  a = a * 80.0f;//mm * steps/mm

  //Set stepper motor directions for transition
  if      (y > basePosition)  BASEDIR = HIGH;
  else if (y < basePosition)  BASEDIR = LOW;
  if      (p > pitchPosition) PITCHDIR = HIGH;
  else if (p < pitchPosition) PITCHDIR = LOW;
  if      (a > actuPosition)  ACTUDIR = HIGH;
  else if (a < actuPosition)  ACTUDIR =  LOW;
  float rMod = (r >= rollPosition) ? 1 : -1;

  //total distance covered during transition
  float baseDistance  = abs(basePosition  - y);
  float pitchDistance = abs(pitchPosition - p);
  float rollDistance  = abs(rollPosition  - r);
  float actuDistance  = abs(actuPosition  - a);

  //for(float yI = 0; yI < abs(basePosition - y); yI++)

  //highest degree of change to calculate incrementation
  float maxDistance = max(baseDistance, max(pitchDistance, max(rollDistance, actuDistance)));

  long baseIncrements, pitchIncrements, rollIncrements, actuIncrements;
  
  //100 / 100 * 100 = 100; 1600 / 1600 * 100 = 100
  //amount of steps skipped before stepper is moved
  if(baseDistance  < 1){
    baseIncrements = (long)(maxDistance  * calculationAccuracy);
  }
  else{
    baseIncrements  = (long)((maxDistance / baseDistance)  * calculationAccuracy);
  }
  
  if(pitchDistance < 1){
    pitchIncrements = (long)(maxDistance  * calculationAccuracy);
  }
  else{
    pitchIncrements = (long)((maxDistance / pitchDistance) * calculationAccuracy);
  }
  
  if(rollDistance  < 1){
    rollIncrements = (long)(maxDistance  * calculationAccuracy);
  }
  else{
    rollIncrements  = (long)((maxDistance / rollDistance)  * calculationAccuracy);
  }
  
  if(actuDistance  < 1){
    actuIncrements = (long)(maxDistance  * calculationAccuracy);
  }
  else{
    actuIncrements  = (long)((maxDistance / actuDistance)  * calculationAccuracy);
  }
  
  long maxIncrements = max(baseIncrements, max(pitchIncrements, max(rollIncrements, actuIncrements)));

  //incrementer for triggering step
  long bI = baseIncrements;
  long pI = pitchIncrements;
  long rI = rollIncrements;
  long aI = rollIncrements;

  long bIO = 0;//stepper off time incrementer
  long pIO = 0;
  long aIO = 0;
  
  Serial.print("DISTANCE: ");  Serial.print(",");
  Serial.print(baseDistance);  Serial.print(",");
  Serial.print(pitchDistance); Serial.print(",");
  Serial.print(rollDistance);  Serial.print(",");
  Serial.print(actuDistance);  Serial.println();

  Serial.print("INCREMENTS: ");  Serial.print(",");
  Serial.print(baseIncrements);  Serial.print(",");
  Serial.print(pitchIncrements); Serial.print(",");
  Serial.print(rollIncrements); Serial.print(",");
  Serial.print(actuIncrements);  Serial.println();

  //used to know when to turn a step signal off
  bool bStepped = false;
  bool pStepped = false;
  bool aStepped = false;

  Serial.print("FOR:"); Serial.println(maxDistance * calculationAccuracy);

  LEDPIN = LOW;
  
  //performs the transition
  for(long i = 0; i < maxDistance * calculationAccuracy; i++){
    /*
    if(DEBUG && false){
      Serial.print(i);   Serial.print(",");
      Serial.print(bI);  Serial.print(",");
      Serial.print(pI);  Serial.print(",");
      Serial.print(rI);  Serial.print(",");
      Serial.print(aI);  Serial.println(",");
    }
    */
    
    if(bI < baseIncrements){
      bI++;
    }
    else{
      bI = 0;
      //digitalWrite(BASESTP, HIGH);
      BASESTP = HIGH;
      bStepped = true;
      //if(DEBUG)Serial.println("BSTEP");
    }
      
    if(bStepped && bIO * timeIncrement < minimumStepTime){
      bIO++;
    }
    else if (bStepped){
      bIO = 0;
      //digitalWrite(BASESTP,  LOW);
      BASESTP = LOW;
      bStepped = false;
      //if(DEBUG)Serial.println("BSTEP OFF");
    }
    
    if(pI < pitchIncrements){
      pI++;
    }
    else{
      pI = 0;
      //digitalWrite(PITCHSTP, HIGH);
      PITCHSTP = HIGH;
      pStepped = true;
      //if(DEBUG)Serial.println("PSTEP");
    }
    
    if(pStepped && pIO * timeIncrement < minimumStepTime){
      pIO++;
    }
    else if (pStepped){
      pIO = 0;
      //digitalWrite(PITCHSTP,  LOW);
      PITCHSTP = LOW;
      pStepped = false;
      //if(DEBUG)Serial.println("PSTEP OFF");
    }
    
    if(rI < rollIncrements){
      rI++;
    }
    else{
      rollPosition = rollPosition + rMod;
      rI = 0;
      roll.write(rollPosition * 0.1125f);//move servo same increment as stepper at 1/16th mstepping
      //if(DEBUG){Serial.println("RStep");}
    }

    if(aI < actuIncrements){
      aI++;
    }
    else{
      aI = 0;
      //digitalWrite(ACTUSTP, HIGH);
      ACTUSTP = HIGH;
      aStepped = true;
      //if(DEBUG)Serial.println("ASTEP");
    }
    
    if(aStepped && aIO * timeIncrement < minimumStepTime){
      aIO++;
    }
    else if (aStepped){
      aIO = 0;
      //digitalWrite(ACTUSTP,  LOW);
      ACTUSTP = LOW;
      aStepped = false;
      //if(DEBUG)Serial.println("ASTEP OFF");
    }

    //delayMPU();
    //delayMicroseconds(timeIncrement);

    //Serial.print(maxValue * calculationAccuracy); Serial.print(","); Serial.println(i);
  }

  Serial.println("post transition");

  /*
  if (bStepped) digitalWrite(BASESTP,  LOW);
  if (pStepped) digitalWrite(PITCHSTP, LOW);
  if (aStepped) digitalWrite(ACTUSTP, LOW);
  */
  
  if (bStepped) BASESTP = LOW;
  if (pStepped) PITCHSTP = LOW;
  if (aStepped) ACTUSTP = LOW;
  
  LEDPIN = HIGH;
  
  basePosition  = y;
  pitchPosition = p;
  rollPosition  = r;
  actuPosition  = a;

  //delayMPU();//give time to allow the stepper drivers to be set low
}
