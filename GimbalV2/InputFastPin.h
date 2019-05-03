class InputFastPin{
  private:
    uint8_t pinBit;
    
  public:
    InputFastPin(){
      
    }
    
    InputFastPin(uint8_t pinBit){
      this->pinBit = pinBit;
      pinMode(pinBit, INPUT);
    }
    
    bool Read(){
      return digitalReadFast(pinBit);
    }

};
