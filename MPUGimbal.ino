#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Servo.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 accelgyro;
#define TCAADDR 0x70
#define DEBUG false

int16_t ax, ay, az;
int16_t gx, gy, gz;

const int BASESTP  = 0;
const int BASEDIR  = 1;
const int PITCHSTP = 2;
const int PITCHDIR = 3;
const int ACTUSTP  = 4;
const int ACTUDIR  = 5;
const int SERVOPIN = 6;

Servo roll;

float basePosition = 0.0f;
float pitchPosition = 0.0f;
float rollPosition = 0.0f;
float actuPosition = 0.0f;

const long calculationAccuracy = 100;
const int stepTime = 500;//microseconds
const int timeIncrement = stepTime / calculationAccuracy;
const int minimumStepTime = 1;//microseconds

byte mpuSelection = 0;

void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void setup() {
    pinMode(BASESTP,  OUTPUT);
    pinMode(BASEDIR,  OUTPUT);
    pinMode(PITCHSTP, OUTPUT);
    pinMode(PITCHDIR, OUTPUT);
    pinMode(ACTUSTP, OUTPUT);
    pinMode(ACTUDIR, OUTPUT);
  
    roll.attach(SERVOPIN, 425, 2225);//initialize servo and set range

    roll.write(0);//Center servo
    
    digitalWrite(BASESTP,  LOW);
    digitalWrite(PITCHSTP, LOW);
    digitalWrite(ACTUSTP, LOW);
    digitalWrite(BASEDIR,  HIGH);
    digitalWrite(PITCHDIR, HIGH);
    digitalWrite(ACTUDIR, HIGH);
    

    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);

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
  /*
    transitionGimbal( 90,   90,  90, 0);
    delay(5000);
    transitionGimbal(-90, -90, 0, 8000);//100 mm transition
    delay(5000);
    */

    digitalWrite(ACTUSTP, HIGH);

    delayMicroseconds(5);
    
    digitalWrite(ACTUSTP,  LOW);

    delayMicroseconds(500);
}

void readOutAccelGyro(){
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    Serial.print(mpuSelection);               Serial.print(",");//show which mpu is printing
    Serial.print(accelgyro.getTemperature()); Serial.print(",");
    Serial.print(ax);                         Serial.print(",");
    Serial.print(ay);                         Serial.print(",");
    Serial.print(az);                         Serial.print(",");
    Serial.print(gx);                         Serial.print(",");
    Serial.print(gy);                         Serial.print(",");
    Serial.print(gz);                         Serial.print(";");
}

void transitionGimbal(float y, float p, float r, float a){
  Serial.println("transition");
  y = (800.0f / 90.0f) * y;//0.1125 degrees
  p = (800.0f / 90.0f) * p;
  r = (800.0f / 90.0f) * r;//scale servo output to stepper range moving at 0.1125 degree increments

  //Set stepper motor directions for transition
  if      (y > basePosition)  digitalWrite(BASEDIR,  HIGH);
  else if (y < basePosition)  digitalWrite(BASEDIR,  LOW);
  if      (p > pitchPosition) digitalWrite(PITCHDIR,  HIGH);
  else if (p < pitchPosition) digitalWrite(PITCHDIR,  LOW);
  if      (a > actuPosition)  digitalWrite(ACTUDIR,  HIGH);
  else if (a < actuPosition)  digitalWrite(ACTUDIR,  LOW);
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
  
  Serial.print("DISTANCE: ");  Serial.print(",");
  Serial.print(baseDistance);  Serial.print(",");
  Serial.print(pitchDistance); Serial.print(",");
  Serial.print(rollDistance);  Serial.print(",");
  Serial.print(actuDistance);  Serial.println();

  Serial.print("INCREMENTS: ");  Serial.print(",");
  Serial.print(baseIncrements);  Serial.print(",");
  Serial.print(pitchIncrements); Serial.print(",");
  Serial.print(rollIncrements); Serial.print(",");
  Serial.print(actuIncrements);  Serial.println();

  //used to know when to turn a step signal off
  bool bStepped = false;
  bool pStepped = false;
  bool aStepped = false;

  Serial.print("FOR:"); Serial.println(maxDistance * calculationAccuracy);

  //performs the transition
  for(long i = 0; i < maxDistance * calculationAccuracy; i++){
    if(DEBUG){
      Serial.print(i);   Serial.print(",");
      Serial.print(bI);  Serial.print(",");
      Serial.print(pI);  Serial.print(",");
      Serial.print(rI);  Serial.print(",");
      Serial.print(aI);  Serial.println(",");
    }
    
    if(bI < baseIncrements){
      bI++;
    }
    else{
      bI = 0;
      digitalWrite(BASESTP, HIGH);
      bStepped = true;
      if(DEBUG)Serial.println("BSTEP");
    }
      
    if(bStepped && bIO * timeIncrement < minimumStepTime){
      bIO++;
    }
    else if (bStepped){
      bIO = 0;
      digitalWrite(BASESTP,  LOW);
      bStepped = false;
      if(DEBUG)Serial.println("BSTEP OFF");
    }
    
    if(pI < pitchIncrements){
      pI++;
    }
    else{
      pI = 0;
      digitalWrite(PITCHSTP, HIGH);
      pStepped = true;
      if(DEBUG)Serial.println("PSTEP");
    }
    
    if(pStepped && pIO * timeIncrement < minimumStepTime){
      pIO++;
    }
    else if (pStepped){
      pIO = 0;
      digitalWrite(PITCHSTP,  LOW);
      pStepped = false;
      if(DEBUG)Serial.println("PSTEP OFF");
    }
    
    if(rI < rollIncrements){
      rI++;
    }
    else{
      rollPosition = rollPosition + rMod;
      rI = 0;
      roll.write(rollPosition * 0.1125f);//move servo same increment as stepper at 1/16th mstepping
      if(DEBUG){
        Serial.println("RStep");
        Serial.println((rollPosition * 0.1125f));
      }
    }

    if(aI < actuIncrements){
      aI++;
    }
    else{
      aI = 0;
      digitalWrite(ACTUSTP, HIGH);
      aStepped = true;
      if(DEBUG)Serial.println("ASTEP");
    }
    
    if(aStepped && aIO * timeIncrement < minimumStepTime){
      aIO++;
    }
    else if (aStepped){
      aIO = 0;
      digitalWrite(ACTUSTP,  LOW);
      aStepped = false;
      if(DEBUG)Serial.println("ASTEP OFF");
    }

    //delayMPU();
    delayMicroseconds(timeIncrement);

    //Serial.print(maxValue * calculationAccuracy); Serial.print(","); Serial.println(i);
  }

  Serial.println("post transition");

  if (bStepped) digitalWrite(BASESTP,  LOW);
  if (pStepped) digitalWrite(PITCHSTP, LOW);
  if (aStepped) digitalWrite(ACTUSTP, LOW);
  
  basePosition  = y;
  pitchPosition = p;
  rollPosition  = r;
  actuPosition  = a;

  delayMPU();//give time to allow the stepper drivers to be set low
}

void delayMPU(){//replaces the ms delay to make the instructions useful
  tcaselect(mpuSelection);//select the MPU

  delayMicroseconds(400);
  
  readOutAccelGyro();//print all MPU values

  mpuSelection++;//increment to the next mpu
  
  if(mpuSelection > 7){//check if the max multiplexer value is selected, then print a new line
    mpuSelection = 0;
    Serial.println();
  }
}

