/*
 * SLIP RINGS SUPPORT 250RPM
 * DPS -> RPM : DPS / 6 = RPM
 * 250RPM = 6 * 250 DPS = 1500 DPS
 * 
 * 8W -> >500,000,000 rotations
 * 12W -> >20,000,000 rotations
 */
#include "Axis.h"

const bool DEBUG = false;
const float MICROSTEP = 64.0f;
const int STEPTIME = 3;//microseconds
const int ACCURACY = 10;
const int MINIMUMVELOCITY = 1;
const float RAMPPERCENTAGE = 0.1f;

Axis act = Axis(8,  12, 10, 3, 45.0f, -5.0f, 5.0f * MICROSTEP, 'A');
Axis rol = Axis(7,  11, 9,  4, 30.0f, 85.0f,  (200.0f * MICROSTEP * (90.0f / 20.0f)) / 360.0f, 'R');
Axis pit = Axis(28, 32, 30, 5, 30.0f, 13.5f,  (200.0f * MICROSTEP * (90.0f / 20.0f)) / 360.0f, 'P');
Axis yaw = Axis(27, 31, 29, 6, 45.0f, -82.5f, (200.0f * MICROSTEP * (39.0f / 20.0f)) / 360.0f, 'Y');

float stepsPerIncrement = (200.0f * MICROSTEP * (39.0f / 20.0f)) / 360.0f;

OutputFastPin CONTROL = OutputFastPin(2);

void homingSequence(){
  act.HomeAxis();
  rol.HomeAxis();
  pit.HomeAxis();
  yaw.HomeAxis();
}

void TestAxes(){
  Serial.println("Testing Axes..");
  for(int i = 0; i < 4; i++){
    act.RampMoveAxis(-100, 100);
    rol.RampMoveAxis(90, 100);
    
    for(int j = 0; j < 4; j++){
      pit.RampMoveAxis(90, 100);

      for(int k = 0; k < 4; k++){
        yaw.RampMoveAxis(90, 100);
        
        delay(50);
      }
    }
  }
  
  act.RampMoveAxis(400, 100);

  delay(1000);
}

void setup() {
  CONTROL.Low();

  Serial.begin(115200);

  Serial.println("Enabling stepper drivers...");
  
  act.Enable();
  rol.Enable();
  pit.Enable();
  yaw.Enable();

  delay(100);

  Serial.println("Homing gimbal...");
  homingSequence();
  //rol.HomeAxis();

  delay(200);
  act.RampMoveAxis(-250, 100);
  delay(200);
}

void loop() {
  //TestAxes();
  //homingSequence();

  //rol.RampMoveAxis(1440, 2500);
  RampAllAxes(720, 3000);

  delay(1000);
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

  if(actOffsetDistance < 0) { act.Backward(); actOffsetDistance = abs(actOffsetDistance); } else act.Forward();
  if(rolOffsetDistance < 0) { rol.Backward(); rolOffsetDistance = abs(rolOffsetDistance); } else rol.Forward();
  if(pitOffsetDistance < 0) { pit.Backward(); pitOffsetDistance = abs(pitOffsetDistance); } else pit.Forward();
  if(yawOffsetDistance < 0) { yaw.Backward(); yawOffsetDistance = abs(yawOffsetDistance); } else yaw.Forward();
  
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
  long maxStepIncrements = maxSteps * ACCURACY;
  long rampUp = maxStepIncrements * RAMPPERCENTAGE;
  long rampDown = maxStepIncrements * (1.0f - RAMPPERCENTAGE);
  
  cli();
  CONTROL.High();

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

  sei();

  CONTROL.Low();
}

void TestDriverEnables(){
  Serial.println("Axis Enable");
  act.Enable();
  delay(2000);
  Serial.println("Axis Disable");
  act.Disable();
  delay(2000);

  Serial.println("Roll Enable");
  rol.Enable();
  delay(2000);
  Serial.println("Roll Disable");
  rol.Disable();
  delay(2000);
  
  Serial.println("Pitch Enable");
  pit.Enable();
  delay(2000);
  Serial.println("Pitch Disable");
  pit.Disable();
  delay(2000);
  
  Serial.println("Yaw Enable");
  yaw.Enable();
  delay(2000);
  Serial.println("Yaw Disable");
  yaw.Disable();
  delay(2000);
}

void TestLinearAxis(){
  Serial.println("Moving Axis 10mm");
  act.RampMoveAxis(-500, 200);

  delay(1000);
  
  Serial.println("Moving Axis -10mm");
  act.RampMoveAxis(500, 200);

  delay(1000);
}

int MicroDelay(float velocity) {
  //1 / (micros * steps/mm  * mm/s)
  return 1000000.0f / (stepsPerIncrement * velocity);
}

long StepsPerDistance(float distance) {
  //mm * steps/mm = steps
  return distance * stepsPerIncrement;
}

void RampAllAxes(float distance, float velocity){
    long steps = StepsPerDistance(distance);
    bool dForward = steps > 1;
    steps = abs(steps);

    if (dForward){
      rol.Forward();
      pit.Forward();
      yaw.Forward();
    }
    else{
      rol.Backward();
      pit.Backward();
      yaw.Backward();
    }
    
    float rampRatio = 0.1f;
    float minVel = 1.0f;
    int stepTime = 3;//microseconds

    long rampUp = steps * rampRatio;
    long rampDown = steps * (1.0f - rampRatio);
    unsigned int velocityInterp = minVel;

    for (long i = 0; i < steps; i++) {
      rol.StepOn();
      pit.StepOn();
      yaw.StepOn();

      delayMicroseconds(stepTime);
      
      rol.StepOff();
      pit.StepOff();
      yaw.StepOff();

      if (i < rampUp) {
        velocityInterp = mapFast(i, 0, rampUp, minVel, velocity);
      }
      else if (i > rampDown) {
        velocityInterp = mapFast(i, rampDown, steps, velocity, minVel);
      }
      else {
        velocityInterp = velocity;
      }

      if (velocityInterp < 0) velocityInterp = 0;

      delayMicroseconds(MicroDelay(velocityInterp));
    }
}
