#include "OpenCollectorFastPin.h"
#include "OutputFastPin.h"
#include "InputFastPin.h"

class Axis {
  private:
    OpenCollectorFastPin ENA;
    OpenCollectorFastPin STP;
    OpenCollectorFastPin DIR;
    InputFastPin END;

    const float RAMPPERCENTAGE = 0.1f;
    const float MINIMUMVELOCITY = 1.0f;

    bool  moveDirection = true;
    float homingSpeed;
    float homeOffset;
    float stepsPerIncrement;
    char axisName;

    long stepPosition = 0;
    const int STEPTIME = 3;//microseconds

    long mapFast(long x, long in_min, long in_max, long out_min, long out_max)
    {
      return (long)(x - in_min) * (long)(out_max - out_min) / (long)(in_max - in_min) + out_min;
    }

  public:
    Axis(uint8_t ENA, uint8_t STP, uint8_t DIR, uint8_t END, float homingSpeed, float homeOffset, float stepsPerIncrement, char axisName) {
      this->ENA = OpenCollectorFastPin(ENA);
      this->STP = OpenCollectorFastPin(STP);
      this->DIR = OpenCollectorFastPin(DIR);
      this->END = InputFastPin(        END);

      this->homingSpeed = homingSpeed;
      this->homeOffset = homeOffset;
      this->stepsPerIncrement = stepsPerIncrement;
      this->axisName = axisName;

      this->ENA.Low();
      this->STP.Low();
      this->DIR.Low();
    }

    void Enable() {
      ENA.Low();
    }
    
    void Disable() {
      ENA.High();
    }
    
    void StepOn() {
      STP.High();
    }

    void StepOff() {
      STP.Low();
      if (moveDirection) stepPosition++;
      else stepPosition--;
    }

    void Forward() {
      DIR.High();
      moveDirection = true;
    }

    void Backward() {
      DIR.Low();
      moveDirection = false;
    }

    long StepsPerDistance(float distance) {
      //mm * steps/mm = steps
      return distance * stepsPerIncrement;
    }

    int MicroDelay(float velocity) {
      //1 / (micros * steps/mm  * mm/s)
      return 1000000.0f / (stepsPerIncrement * velocity);
    }

    void MoveAxis(float distance, float velocity) {
      long steps = StepsPerDistance(distance);
      bool dForward = steps > 1;
      steps = abs(steps);

      if (dForward) Forward();
      else Backward();

      for (long i = 0; i < steps; i++) {
        Step();
        delayMicroseconds(MicroDelay(velocity));
      }
    }

    void RampMoveAxis(float distance, float velocity) {
      long steps = StepsPerDistance(distance);
      bool dForward = steps > 1;
      steps = abs(steps);

      if (dForward) Forward();
      else Backward();

      long rampUp = steps * RAMPPERCENTAGE;
      long rampDown = steps * (1.0f - RAMPPERCENTAGE);
      unsigned int velocityInterp = MINIMUMVELOCITY;

      for (long i = 0; i < steps; i++) {
        Step();

        if (i < rampUp) {
          velocityInterp = mapFast(i, 0, rampUp, MINIMUMVELOCITY, velocity);
        }
        else if (i > rampDown) {
          velocityInterp = mapFast(i, rampDown, steps, velocity, MINIMUMVELOCITY);
        }
        else {
          velocityInterp = velocity;
        }

        if (velocityInterp < 0) velocityInterp = 0;

        delayMicroseconds(MicroDelay(velocityInterp));
      }
    }

    void HomeAxis() {
      //Move Forward until homed
      Enable();
      Forward();

      while (ReadEndstop()) {
        Step();
        delayMicroseconds(MicroDelay(homingSpeed));
      }

      //move back 10mm
      Backward();
      for (long i = 0; i < StepsPerDistance(10); i++) {
        Step();
        delayMicroseconds(MicroDelay(homingSpeed));
      }

      //rehome at 10% speed
      Forward();
      while (ReadEndstop()) {
        Step();
        delayMicroseconds(MicroDelay(homingSpeed / 20.0f));
      }

      //move by offset
      if (homeOffset < 0) Backward();
      for (long i = 0; i < StepsPerDistance(abs(homeOffset)); i++) {
        Step();
        delayMicroseconds(MicroDelay(homingSpeed));
      }

      stepPosition = 0;
    }

    float GetIncrementPosition() {
      return stepPosition / stepsPerIncrement;  // steps / steps/mm
    }
    void ZeroStepPosition() {
      stepPosition = 0;
    }
    long GetStepPosition() {
      return stepPosition;
    }
    float GetHomingSpeed() {
      return homingSpeed;
    }
    float GetHomeOffset() {
      return homeOffset;
    }
    float GetStepsPerIncrement() {
      return stepsPerIncrement;
    }
    bool ReadEndstop() {
      return END.Read();
    }

    void Step() {
      StepOn();
      delayMicroseconds(STEPTIME);
      StepOff();
    }
};
