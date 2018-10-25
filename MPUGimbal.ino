#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Servo.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 accelgyro;
#define TCAADDR 0x70

int16_t ax, ay, az;
int16_t gx, gy, gz;

const int BASESTP = 60;
const int BASEDIR = 61;
const int BASEENA = 56;
const int PITCHSTP = 54;
const int PITCHDIR = 55;
const int PITCHENA = 38;
const int SERVOPIN = 11;

Servo roll;

float basePosition = 0.0f;
float pitchPosition = 0.0f;
float rollPosition = 0.0f;

void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void setup() {
    pinMode(BASESTP, OUTPUT);
    pinMode(BASEDIR, OUTPUT);
    pinMode(BASEENA, OUTPUT);
    pinMode(PITCHSTP, OUTPUT);
    pinMode(PITCHDIR, OUTPUT);
    pinMode(PITCHENA, OUTPUT);
  
    roll.attach(SERVOPIN, 425, 2225);

    roll.write(90);
    
    digitalWrite(BASEENA,  LOW);
    digitalWrite(PITCHENA, LOW);
    digitalWrite(BASEDIR,  HIGH);
    digitalWrite(PITCHDIR, HIGH);

    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(250000);

    for (int i = 0; i < 8; i++){
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
    /*
    for (int i = 0; i < 8; i++){
      tcaselect(i);
      
      readOutAccelGyro();
    }

    Serial.println();
    */
    
    writeMotorPosition(90, 90, 0);
    delay(5000);
    writeMotorPosition(-90, -90, 0);
    delay(5000);
}

void readOutAccelGyro(){
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    Serial.print(ax); Serial.print(",");
    Serial.print(ay); Serial.print(",");
    Serial.print(az); Serial.print(",");
    Serial.print(gx); Serial.print(",");
    Serial.print(gy); Serial.print(",");
    Serial.print(gz); Serial.print(";");
}

void writeMotorPosition(float y, float p, float r){
  y = (800.0f / 90.0f) * y;
  p = (800.0f / 90.0f) * p;
  
  roll.write(r);

  int bI = 0;
  
  if(y > basePosition){
    digitalWrite(BASEDIR,  HIGH);
  }
  else if (y < basePosition){
    digitalWrite(BASEDIR,  LOW);
  }
  
  if(p > pitchPosition){
    digitalWrite(PITCHDIR,  HIGH);
  }
  else if (p < pitchPosition){
    digitalWrite(PITCHDIR,  LOW);
  }

  //total distance covered
  //total distance needed to cover each step

  //speed is based on microsecond delay

  float baseDistance = abs(basePosition - y);
  float pitchDistance = abs(pitchPosition - y);
  
  for(float yI = 0; yI < abs(basePosition - y); yI++){//90 - - 90
    digitalWrite(BASESTP,  HIGH);
    delayMicroseconds(500);
    digitalWrite(BASESTP,  LOW);
    delayMicroseconds(500);
  }
  
  for(float pI = 0; pI < abs(pitchPosition - y); pI++){
    digitalWrite(PITCHSTP,  HIGH);
    delayMicroseconds(500);
    digitalWrite(PITCHSTP,  LOW);
    delayMicroseconds(500);
  }

  basePosition = y;
  pitchPosition = p;
}


