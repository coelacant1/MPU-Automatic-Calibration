#include "OpenCollectorFastPin.h"
#include "OutputFastPin.h"
#include "InputFastPin.h"

class Axis{
  private:
    OpenCollectorFastPin ENA;
    OpenCollectorFastPin STP;
    OpenCollectorFastPin DIR;
    InputFastPin END;

    bool  moveDirection = true;
    float homingSpeed;
    float homeOffset;
    float stepsPerIncrement;
    
    long stepPosition = 0;
    const int STEPTIME = 3;//microseconds
    
  public:
    Axis(uint_8 ENA, uint_8 STP, uint_8 DIR, uint_8 END, float homingSpeed, float homeOffset, float stepsPerIncrement){
      this->ENA = OpenCollectorFastPin(ENA);
      this->STP = OpenCollectorFastPin(STP);
      this->DIR = OpenCollectorFastPin(DIR);
      this->END = InputFastPin(        END);

      this->homingSpeed = homingSpeed;
      this->homeOffset = homeOffset;
      this->stepsPerIncrement = stepsPerIncrement;

      ENA.High();
      STP.High();
      DIR.High();
    }

    void Enable(){ ENA.High(); }
    void Disable(){ ENA.Low(); }
    void StepOn(){ STP.High(); }

    void StepOff(){
      STP.Low();
      if(moveDirection) stepPosition++;
      else stepPosition--;
    }

    void Forward(){
      DIR.High();
      moveDirection = true;
    }

    void Backward(){
      DIR.Low();
      moveDirection = false;
    }

    long StepsPerDistance(float distance){
      //mm * steps/mm = steps
      return distance * stepsPerIncrement;
    }

    int MicroDelay(float velocity){
      //1 / (micros * steps/mm  * mm/s)
      return 1 / (1000000.0f * stepsPerIncrement * velocity);
    }
    
    void HomeAxis(){
      //Move Forward until homed
      Enable();
      Forward();
      
      while(!ReadEndstop()){
        Step();
        delayMicroseconds(MicroDelay(homingSpeed));
      }
    
      //move back 10mm
      Backward();
      for(long i = 0; i < StepsPerDistance(10); i++){
        StepAxis(axis);
        delayMicroseconds(MicroDelay(homingSpeed));
      }
      
      //rehome at 10% speed
      Forward();
      while(!ReadEndstop()){
        Step();
        delayMicroseconds(MicroDelay(homingSpeed / 10.0f));
      }
    
      //move by offset
      if(homeOffset < 0) Backward();
      for(long i = 0; i < StepsPerDistance(abs(homeOffset)); i++){
        Step();
        delayMicroseconds(MicroDelay(homingSpeed));
      }

      stepPosition = 0;
    }

    float GetIncrementPosition(){ return stepPosition / stepsPerIncrement; }// steps / steps/mm
    void ZeroStepPosition(){ stepPosition = 0; }
    long GetStepPosition(){return stepPosition; }
    float GetHomingSpeed(){ return homingSpeed; }
    float GetHomeOffset(){ return homeOffsetingSpeed; }
    float GetStepsPerIncrement(){ return stepsPerIncrement; }
    bool ReadEndstop(){ return END.Read(); }

    void Step(){
      StepOn();
      delayMicroseconds(STEPTIME);
      StepOff();
    }
};
