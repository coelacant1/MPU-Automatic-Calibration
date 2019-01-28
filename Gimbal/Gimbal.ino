#include <digitalWriteFast.h>
#include <Servo.h>

const int SERVOPIN = 9;

#define ENABLE 2
#define BASESTP 3
#define BASEDIR 4
#define PITCHSTP 5
#define PITCHDIR 6
#define ACTUSTP 7
#define ACTUDIR 8
#define LEDPIN 13

Servo roll;

float basePosition = 800.0f;
float pitchPosition = 800.0f;
float rollPosition = 800.0f;
float actuPosition = 500.0f * 80.0f;

const long calculationAccuracy = 1;
const int stepTime = 2;//microseconds
const int minimumStepTime = 1;//microseconds
const int targetVelocity = 300;//mm/s
const int minimumVelocity = 10;//mm/s
const int actuStepsPerMM = 80;//steps/mm
const float accelRampPercentage = 0.2f;//percentage of travel for acceleration/deceleration to occur

int timeIncrement = stepTime / calculationAccuracy;

bool DEBUG = false;

int motorWrite = 0;

void setup() {
    pinModeFast(ENABLE, OUTPUT);
    pinModeFast(BASESTP, OUTPUT);
    pinModeFast(BASEDIR, OUTPUT);
    pinModeFast(PITCHSTP, OUTPUT);
    pinModeFast(PITCHDIR, OUTPUT);
    pinModeFast(ACTUSTP, OUTPUT);
    pinModeFast(ACTUDIR, OUTPUT);
    pinModeFast(LEDPIN, OUTPUT);

    digitalWriteFast(ENABLE, LOW);
    digitalWriteFast(BASESTP, LOW);
    digitalWriteFast(BASEDIR, HIGH);
    digitalWriteFast(PITCHSTP, LOW);
    digitalWriteFast(PITCHDIR, HIGH);
    digitalWriteFast(ACTUSTP, LOW);
    digitalWriteFast(ACTUDIR, HIGH);
    digitalWriteFast(LEDPIN, HIGH);
    
    roll.attach(SERVOPIN, 425, 2225);//initialize servo and set range

    roll.write(90);//Center servo
    
    //Serial.begin(250000);
    //Serial1.begin(250000);
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

int mapFast(int x, int in_min, int in_max, int out_min, int out_max)
{
  return (long)(x - in_min) * (long)(out_max - out_min) / (long)(in_max - in_min) + out_min;
}

void transitionGimbal(float y, float p, float r, float a){
  //Serial.println("transition");
  y = (800.0f / 90.0f) * y;//0.1125 degrees
  p = (800.0f / 90.0f) * p;
  r = (800.0f / 90.0f) * r;//scale servo output to stepper range moving at 0.1125 degree increments
  a = a * 80.0f;//mm * steps/mm

  //Set stepper motor directions for transition
  if      (y > basePosition){ digitalWriteFast(BASEDIR, HIGH); }
  else if (y < basePosition){ digitalWriteFast(BASEDIR, LOW); }
  
  if      (p > pitchPosition){ digitalWriteFast(PITCHDIR, HIGH); }
  else if (p < pitchPosition){ digitalWriteFast(PITCHDIR, LOW); }
  
  if      (a > actuPosition){ digitalWriteFast(ACTUDIR, HIGH); }
  else if (a < actuPosition){  digitalWriteFast(ACTUDIR, LOW); }
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
  /*
  Serial.print("DISTANCE: ");  Serial.print(",");
  Serial.print(baseDistance);  Serial.print(",");
  Serial.print(pitchDistance); Serial.print(",");
  Serial.print(rollDistance);  Serial.print(",");
  Serial.print(actuDistance);  Serial.println();

  Serial.print("INCREMENTS: ");  Serial.print(",");
  Serial.print(baseIncrements);  Serial.print(",");
  Serial.print(pitchIncrements); Serial.print(",");
  Serial.print(rollIncrements);  Serial.print(",");
  Serial.print(actuIncrements);  Serial.println();
  */
  //used to know when to turn a step signal off
  bool bStepped = false;
  bool pStepped = false;
  bool aStepped = false;

  //Serial.print("FOR:"); Serial.println(maxDistance * calculationAccuracy);

  digitalWriteFast(LEDPIN, LOW);

  long writeIncrements = (int)(maxDistance * calculationAccuracy / 100.0f);
  long wI = 0;
  int writeCount = 0;

  int minVelocityPeriod = (int)(1000000.0f / (float)(minimumVelocity * actuStepsPerMM * actuIncrements));//microseconds per actuator step
  int maxVelocityPeriod = (int)(1000000.0f / (float)(targetVelocity * actuStepsPerMM * actuIncrements));

  unsigned int microsecondDelay = minVelocityPeriod;
  unsigned int maxSteps = maxDistance * calculationAccuracy;
  unsigned int rampUp = maxSteps * accelRampPercentage;
  unsigned int rampDown = maxSteps * (1.0f - accelRampPercentage);
  
  //performs the transition
  for(unsigned int i = 0; i < maxSteps; i++){
    /*
    if(wI < writeIncrements){
      wI++;
    }
    else if (writeCount < 4){
      writeMotorOut(writeCount);
      writeCount++;
    }
    else{
      wI = 0;
      writeCount = 0;
    }
    */

    timeIncrement = microsecondDelay;
    
    if(bI < baseIncrements){
      bI++;
    }
    else{
      bI = 0;
      digitalWriteFast(BASESTP, HIGH);
      bStepped = true;
    }
      
    if(bStepped && bIO * timeIncrement < minimumStepTime){
      bIO++;
    }
    else if (bStepped){
      bIO = 0;
      digitalWriteFast(BASESTP, LOW);
      bStepped = false;
    }
    
    if(pI < pitchIncrements){
      pI++;
    }
    else{
      pI = 0;
      digitalWriteFast(PITCHSTP, HIGH);
      pStepped = true;
    }
    
    if(pStepped && pIO * timeIncrement < minimumStepTime){
      pIO++;
    }
    else if (pStepped){
      pIO = 0;
      digitalWriteFast(PITCHSTP, LOW);
      pStepped = false;
    }
    
    if(rI < rollIncrements){
      rI++;
    }
    else{
      rollPosition = rollPosition + rMod;
      rI = 0;
      roll.write(rollPosition * 0.1125f);
    }

    if(aI < actuIncrements){
      aI++;
    }
    else{
      aI = 0;
      digitalWriteFast(ACTUSTP, HIGH);
      aStepped = true;
    }
    if(aStepped && aIO * microsecondDelay < minimumStepTime){
      aIO++;
    }
    else if (aStepped){
      aIO = 0;
      digitalWriteFast(ACTUSTP, LOW);
      aStepped = false;
    }

    if(i < rampUp){//acceleration ramp
      microsecondDelay = mapFast(i, 0, rampUp, minVelocityPeriod / 4, maxVelocityPeriod / 8);
    }
    else if(i > rampDown){//deceleration ramp
      microsecondDelay = mapFast(i, rampDown, maxSteps, maxVelocityPeriod / 8, minVelocityPeriod / 4);
    }
    else{
      microsecondDelay = maxVelocityPeriod;
    }

    if(microsecondDelay < 0){
      microsecondDelay = 0;
    }

    //Serial.println(microsecondDelay);

    //delayMPU();
    delayMicroseconds(microsecondDelay);

    //Serial.print(maxValue * calculationAccuracy); Serial.print(","); Serial.println(i);
  }

  //Serial.println("post transition");
  
  if (bStepped){ digitalWriteFast(BASESTP, LOW); }
  if (pStepped){ digitalWriteFast(PITCHSTP, LOW); }
  if (aStepped){ digitalWriteFast(ACTUSTP, LOW); }
  
  digitalWriteFast(LEDPIN, HIGH);
  
  basePosition  = y;
  pitchPosition = p;
  rollPosition  = r;
  actuPosition  = a;

  //delayMPU();//give time to allow the stepper drivers to be set low
}

void writeMotorOut(int pos){
  switch(pos){
    case 0:
      //Serial.print(actuPosition);   Serial.print(",");
      //Serial1.print(actuPosition);   Serial1.print(",");
      break;
    case 1:
      //Serial.print(basePosition);   Serial.print(",");
      //Serial1.print(basePosition);   Serial1.print(",");
      break;
    case 2:
      //Serial.print(pitchPosition);  Serial.print(",");
      //Serial1.print(pitchPosition);  Serial1.print(",");
      break;
    case 3:
      //Serial.print(rollPosition);   Serial.println();
      //Serial1.print(rollPosition);   Serial1.println();
      break;
  }
}
