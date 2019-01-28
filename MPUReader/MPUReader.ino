#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h"
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

float linear = 0.0;
float base   = 0.0;
float outer  = 0.0;
float inner  = 0.0;

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
  
  Serial.begin(115200);
  //Serial1.begin(250000);

  //Serial1.
  //getMotorPositions();

  delay(200);

  for (int i = 0; i < 8; i++){
    tcaselect(i);

    delay(100);
    
    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    delay(100);

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  LEDPIN = HIGH;
  readMPU();
  LEDPIN = LOW;
}

void readOutAccelGyro(){
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    float temperature = ((float)accelgyro.getTemperature() + 12412.0f) / 340.0f;

    //Serial.print(mpuSelection);               Serial.print(",");//show which mpu is printing
    //Serial.print(temperature);                Serial.print(",");
    Serial.print(ax);                         Serial.print(",");
    Serial.print(ay);                         Serial.print(",");
    Serial.print(az);                         Serial.print(",");
    Serial.print(gx);                         Serial.print(",");
    Serial.print(gy);                         Serial.print(",");
    Serial.print(gz);                         Serial.print(";");
}

void readMPU(){
  //tcaselect(mpuSelection);//select the MPU
  
  readOutAccelGyro();//print all MPU values
  
  previousMPU = mpuSelection;
  mpuSelection++;//increment to the next mpu
  
  if(mpuSelection > 7){//check if the max multiplexer value is selected, then print a new line
    mpuSelection = 0;
    Serial.println();
    //getMotorPositions();
    //Serial.print(linear);                     Serial.print(",");
    //Serial.print(base);                       Serial.print(",");
    //Serial.print(outer);                      Serial.print(",");
    //Serial.print(inner);                      Serial.print(";");
  }
  
}
/*
String serial1ReadLine(){
  String inData;
  
  while (Serial1.available() > 0)
  {
    char recieved = Serial1.read();
    inData += recieved; 

    // Process message when new line character is recieved
    if (recieved == '\n')
    {
        inData = ""; // Clear recieved buffer
    }
  }

  return inData;
}

String getValue(String data, char separator, int index){
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }
  
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void getMotorPositions(){
  //linear, base, outer, inner
  String data = serial1ReadLine();
  //float, float, float, float\n

  linear = getValue(data, ',', 0).toFloat();
  base   = getValue(data, ',', 1).toFloat();
  outer  = getValue(data, ',', 2).toFloat();
  inner  = getValue(data, ',', 3).toFloat();
}
*/
