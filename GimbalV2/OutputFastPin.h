#pragma once

class OutputFastPin{
  private:
    uint8_t pinBit;
    
  public:
    OutputFastPinA(uint8_t pinBit){
      this->pinBit = pinBit;
      pinMode(pinBit, OUTPUT);
    }
    
    void High(){
      digitalWriteFast(pinBit, HIGH)
    }
    
    void Low(){
      digitalWriteFast(pinBit, LOW)
    }

};
