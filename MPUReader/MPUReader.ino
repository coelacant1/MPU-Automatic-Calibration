#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "DirectIO.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 accelgyro;
#define TCAADDR 0x70
#define DEBUG false

int16_t ax, ay, az;
int16_t gx, gy, gz;

byte mpuSelection = 0;
byte readPosition = 0;
byte previousMPU  = 7;

Output<13> LEDPIN(HIGH);

void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}


void setup() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  
  Serial.begin(2000000);

  for (int i = 0; i < 8; i++){
    tcaselect(i);
    
    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    delay(100);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  LEDPIN = HIGH;
  readMPU();
  LEDPIN = LOW;
}

int t = 0;

void readOutAccelGyro(int pos){
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    Serial.println(t);
    t++;
  /*
    switch(pos){
      case 0:
        Serial.print(mpuSelection);               Serial.print(",");//show which mpu is printing
        break;
      case 1:
        Serial.print(accelgyro.getTemperature()); Serial.print(",");
        break;
      case 2:
        Serial.print(ax);                         Serial.print(",");
        break;
      case 3:
        Serial.print(ay);                         Serial.print(",");
        break;
      case 4:
        Serial.print(az);                         Serial.print(",");
        break;
      case 5:
        Serial.print(gx);                         Serial.print(",");
        break;
      case 6:
        Serial.print(gy);                         Serial.print(",");
        break;
      case 7:
        Serial.print(gz);                         Serial.print(";");
        break;
    }
    */
}

void readMPU(){//replaces the ms delay to make the instructions useful
  if(readPosition == 0){
    tcaselect(mpuSelection);//select the MPU
    
    previousMPU = mpuSelection;
    mpuSelection++;//increment to the next mpu
    
    if(mpuSelection > 7){//check if the max multiplexer value is selected, then print a new line
      mpuSelection = 0;
      Serial.println();
    }
  }
  
  readOutAccelGyro(readPosition);//print all MPU values
  readPosition++;
  
  if(readPosition > 7){
    readPosition = 0;
  }
}

