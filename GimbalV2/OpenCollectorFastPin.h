class OpenCollectorFastPin{
  private:
    uint8_t pinBit;
    
  public:
    OpenCollectorFastPin(){
      
    }
    
    OpenCollectorFastPin(uint8_t pinBit){
      this->pinBit = pinBit;
      digitalWriteFast(pinBit, LOW);//set port to zero as normal/low output
    }
  
    virtual void High(){
      pinMode(pinBit, OUTPUT);//set port to one
    }
    
    virtual void Low(){
      pinMode(pinBit, INPUT);//set port to zero
    }
};
