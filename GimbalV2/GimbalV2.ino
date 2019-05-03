#include "OpenCollectorFastPin.h"
#include "OutputFastPin.h"
#include "InputFastPin.h"

const bool DEBUG = false;
const int MICROSTEP = 64;
const int STEPTIME = 3;//microseconds
const int ACCURACY = 10;
const int MINIMUMVELOCITY = 1;
const float RAMPPERCENTAGE = 0.1f;

Axis act = Axis(8,  12, 10, 3, 5.0f, 10.0f, 5.0f * MICROSTEP);
Axis rol = Axis(7,  11, 9,  4, 5.0f, 20.0f, 1.0f / (0.45f / MICROSTEP));
Axis pit = Axis(28, 32, 30, 5, 5.0f, 20.0f, 1.0f / (0.45f / MICROSTEP));
Axis yaw = Axis(27, 31, 29, 6, 5.0f, 20.0f, 1.0f / (0.9f  / MICROSTEP));

OutputFastPin CONTROL = OutputFastPinBuilder::BuildPin(2, GPIOOUTPUT);

void setup() {
  CONTROL.Low();
  
  homingSequence();
}

void loop() {
  concurrentTransition(800, 360, 360, 360, 10);

  delay(5000);

  homingSequence();

  delay(10000);
}

long mapFast(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (long)(x - in_min) * (long)(out_max - out_min) / (long)(in_max - in_min) + out_min;
}

void concurrentTransition(float actPos, float rolPos, float pitPos, float yawPos, float velocity){
  float actOffsetDistance = act.GetIncrementPosition() - actPos;
  float rolOffsetDistance = rol.GetIncrementPosition() - rolPos;
  float pitOffsetDistance = pit.GetIncrementPosition() - pitPos;
  float yawOffsetDistance = yaw.GetIncrementPosition() - yawPos;

  if(actOffsetDistance < 0) { act.Backward(); actOffsetDistance = abs(actOffsetDistance); else act.Forward();
  if(rolOffsetDistance < 0) { rol.Backward(); rolOffsetDistance = abs(rolOffsetDistance); else rol.Forward();
  if(pitOffsetDistance < 0) { pit.Backward(); pitOffsetDistance = abs(pitOffsetDistance); else pit.Forward();
  if(yawOffsetDistance < 0) { yaw.Backward(); yawOffsetDistance = abs(yawOffsetDistance); else yaw.Forward();
  
  long actSteps = act.StepsPerDistance(actOffsetDistance);
  long rolSteps = rol.StepsPerDistance(rolOffsetDistance);
  long pitSteps = pit.StepsPerDistance(pitOffsetDistance);
  long yawSteps = yaw.StepsPerDistance(yawOffsetDistance);

  long maxSteps =  max(actSteps, max(rolSteps, max(pitSteps, yawSteps)));

  long actIncrements, rolIncrements, pitIncrements, yawIncrements;

  //calculate run increments to skip in for each stepper 
  if(actSteps < 1) { actIncrements = maxSteps * ACCURACY; } else { actIncrements = ((float)maxSteps / (float)actSteps) * ACCURACY; }
  if(rolSteps < 1) { rolIncrements = maxSteps * ACCURACY; } else { rolIncrements = ((float)maxSteps / (float)rolSteps) * ACCURACY; }
  if(pitSteps < 1) { pitIncrements = maxSteps * ACCURACY; } else { pitIncrements = ((float)maxSteps / (float)pitSteps) * ACCURACY; }
  if(yawSteps < 1) { yawIncrements = maxSteps * ACCURACY; } else { yawIncrements = ((float)maxSteps / (float)yawSteps) * ACCURACY; }

  long maxIncrements = max(actIncrements, max(rolIncrements, max(pitIncrements, yawIncrements)));

  long actI = actIncrements;
  long rolI = rolIncrements;
  long pitI = pitIncrements;
  long yawI = yawIncrements;

  long actIO = 0;
  long rolIO = 0;
  long pitIO = 0;
  long yawIO = 0;

  bool actStepped = false;
  bool rolStepped = false;
  bool pitStepped = false;
  bool yawStepped = false;
  
  //microseconds per actuator step
  int minVelocityPeriod = act.MicroDelay(MINIMUMVELOCITY);
  int maxVelocityPeriod = act.MicroDelay(velocity);

  unsigned int microsecondDelay = minVelocityPeriod;
  long maxStepIncrements = maxSteps * calculationAccuracy;
  long rampUp = maxStepIncrements * RAMPPERCENTAGE;
  long rampDown = maxStepIncrements * (1.0f - RAMPPERCENTAGE);
  
  cli();

  for (long i = 0; i < maxStepIncrements; i++){
    //ACTUATOR
    if(actI < actIncrements){ actI++; }
    else{ actI = 0;  act.StepOn();  actStepped = true;  }//step actuator on
    
    if(actStepped && actIO * microsecondDelay < STEPTIME){ actIO++; }
    else{ actIO = 0; act.StepOff(); actStepped = false; }//step actuator off

    //ROLL
    if(rolI < rolIncrements){ rolI++; }
    else{ rolI = 0;  rol.StepOn();  rolStepped = true;  }//step roll on
    
    if(rolStepped && rolIO * microsecondDelay < STEPTIME){ rolIO++; }
    else{ rolIO = 0; rol.StepOff(); rolStepped = false; }//step roll off

    //PITCH
    if(pitI < pitIncrements){ pitI++; }
    else{ pitI = 0;  pit.StepOn();  pitStepped = true;  }//step pitch on
    
    if(pitStepped && pitIO * microsecondDelay < STEPTIME){ pitIO++; }
    else{ pitIO = 0; pit.StepOff(); pitStepped = false; }//step pitch off
    
    //YAW
    if(yawI < yawIncrements){ yawI++; }
    else{ yawI = 0;  yaw.StepOn();  yawStepped = true;  }//step pitch on
    
    if(yawStepped && yawIO * microsecondDelay < STEPTIME){ yawIO++; }
    else{ yawIO = 0; yaw.StepOff(); yawStepped = false; }//step pitch off

    if(i < rampUp){
      microsecondDelay = mapFast(i, 0, rampUp, minVelocityPeriod, maxVelocityPeriod);
    }
    else if(i > rampDown){
      microsecondDelay = mapFast(i, rampDown, maxSteps, maxVelocityPeriod, minVelocityPeriod);
    }
    else{
      microsecondDelay = maxVelocityPeriod;
    }

    if(microsecondDelay < 0) microsecondDelay = 0;

    delayMicroseconds(microsecondDelay);
  }

  if(actStepped) act.StepOff();
  if(rolStepped) rol.StepOff();
  if(pitStepped) pit.StepOff();
  if(yawStepped) yaw.StepOff();
}

void homingSequence(){
  act.HomeAxis();
  rol.HomeAxis();
  pit.HomeAxis();
  yaw.HomeAxis();
}
