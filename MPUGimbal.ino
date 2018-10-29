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

int calculationAccuracy = 100;
int stepTime = 500;//microseconds
int timeIncrement = stepTime / calculationAccuracy;

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
    pinMode(BASEENA,  OUTPUT);
    pinMode(PITCHSTP, OUTPUT);
    pinMode(PITCHDIR, OUTPUT);
    pinMode(PITCHENA, OUTPUT);
  
    roll.attach(SERVOPIN, 425, 2225);//initialize servo and set range

    roll.write(0);//Center servo
    
    digitalWrite(BASESTP,  LOW);
    digitalWrite(PITCHSTP, LOW);
    digitalWrite(BASEENA,  LOW);
    digitalWrite(PITCHENA, LOW);
    digitalWrite(BASEDIR,  HIGH);
    digitalWrite(PITCHDIR, HIGH);

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
    for (int i = 0; i < 8; i++){
      tcaselect(i);
      
      readOutAccelGyro();
    }

    Serial.println();
    */
    
    transitionGimbal( 0,   0,  0);
    delay(5000);
    transitionGimbal( 90,  90, 180);
    delay(5000);
    transitionGimbal(-90, -90, 0);
    delay(5000);
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

void transitionGimbal(float y, float p, float r){

  Serial.println("transition");
  y = (800.0f / 90.0f) * y;//0.1125 degrees
  p = (800.0f / 90.0f) * p;
  r = (800.0f / 90.0f) * r;//scale servo output to stepper range moving at 0.1125 degree increments

  //Set stepper motor directions for transition
  if      (y > basePosition)  digitalWrite(BASEDIR,  HIGH);
  else if (y < basePosition)  digitalWrite(BASEDIR,  LOW);
  if      (p > pitchPosition) digitalWrite(PITCHDIR,  HIGH);
  else if (p < pitchPosition) digitalWrite(PITCHDIR,  LOW);
  float rMod = (r >= rollPosition) ? 1 : -1;

  //total distance covered during transition
  float baseDistance  = abs(basePosition  - y);
  float pitchDistance = abs(pitchPosition - p);
  float rollDistance  = abs(rollPosition  - r);

  //highest degree of change to calculate incrementation
  float maxValue = max(baseDistance, max(pitchDistance, rollDistance));

  //for(float yI = 0; yI < abs(basePosition - y); yI++)

  //Prevents division by zero by requiring a minimum movement
  if(baseDistance  < 1) baseDistance  = 0.1125f;
  if(pitchDistance < 1) pitchDistance = 0.1125f;
  if(rollDistance  < 1) rollDistance  = 0.1125f;
  
  //amount of steps skipped before stepper is moved
  const int baseIncrements  = (int)((maxValue / baseDistance)  * calculationAccuracy);//100 / 100 * 100 = 100; 1600 / 1600 * 100 = 100
  const int pitchIncrements = (int)((maxValue / pitchDistance) * calculationAccuracy);//100 / 80  * 100 = 125; 1600 / 800  * 100 = 200
  const int rollIncrements  = (int)((maxValue / rollDistance)  * calculationAccuracy);//100 / 60  * 100 = 167; 1600 / 400  * 100 = 400

  //incrementer for triggering step
  int bI = baseDistance;
  int pI = pitchDistance;
  int rI = rollIncrements;

  int bIO = 0;//stepper off time incrementer
  int pIO = 0;
  
  Serial.print("DISTANCE: ");  Serial.print(",");
  Serial.print(baseDistance);  Serial.print(",");
  Serial.print(pitchDistance); Serial.print(",");
  Serial.print(rollDistance);  Serial.println();

  Serial.print("INCREMENTS: ");  Serial.print(",");
  Serial.print(baseIncrements);  Serial.print(",");
  Serial.print(pitchIncrements); Serial.print(",");
  Serial.print(rollIncrements);  Serial.println();

  //used to know when to turn a step signal off
  bool bStepped = false;
  bool pStepped = false;

  //performs the transition
  for(int i = 0; i < maxValue * calculationAccuracy; i++){
    if(bI < baseIncrements){
      bI++;
    }
    else{
      bI = 0;
      digitalWrite(BASESTP, HIGH);
      bStepped = true;
      Serial.println("BStep");
    }
      
    if(bStepped && bIO < (stepTime / timeIncrement)){
      bIO++;
    }
    else{
      bIO = 0;
      digitalWrite(BASESTP,  LOW);
      bStepped = false;
    }
    
    if(pI < pitchIncrements){
      pI++;
      if (pStepped) {
        digitalWrite(PITCHSTP,  LOW);
        pStepped = false;
      }
    }
    else{
      pI = 0;
      digitalWrite(PITCHSTP, HIGH);
      pStepped = true;
      Serial.println("PSTEP");
    }
    
    if(pStepped && pIO < (stepTime / timeIncrement)){
      pIO++;
    }
    else{
      pIO = 0;
      digitalWrite(BASESTP,  LOW);
      pStepped = false;
    }
    
    if(rI < rollIncrements){
      rI++;
    }
    else{
      rI = 0;
      roll.write(rollPosition + (rMod * 0.1125f));//move servo same increment as stepper at 1/16th mstepping
      Serial.println("RStep");
    }

    //delayMPU();
    delayMicroseconds(timeIncrement);

    //Serial.print(maxValue * calculationAccuracy); Serial.print(","); Serial.println(i);
  }

  Serial.println("post transition");

  if (bStepped) digitalWrite(BASESTP,  LOW);
  if (pStepped) digitalWrite(PITCHSTP, LOW);
  
  basePosition  = y;
  pitchPosition = p;
  rollPosition  = r;

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

