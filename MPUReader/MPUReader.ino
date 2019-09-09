#include "MPU.h"

#define SDA0 19
#define SCL0 18
#define SDA1 16
#define SCL1 17
#define CONTROL_PIN 12
#define LED_PIN 13
uint8_t useBlink = 1;
bool blinkState = false;
bool fullPrint = false;
bool timePrint = false;
bool dataPrint = true;
unsigned long previousBlink;
unsigned long timeMicros;

MPU mpu0 = MPU(0x68, MPU6050_GYRO_FS_2000, MPU6050_ACCEL_FS_4, SDA0, SCL0);
MPU mpu1 = MPU(0x69, MPU6050_GYRO_FS_2000, MPU6050_ACCEL_FS_4, SDA0, SCL0);
MPU mpu2 = MPU(0x68, MPU6050_GYRO_FS_2000, MPU6050_ACCEL_FS_4, SDA1, SCL1);
MPU mpu3 = MPU(0x69, MPU6050_GYRO_FS_2000, MPU6050_ACCEL_FS_4, SDA1, SCL1);

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(CONTROL_PIN, INPUT);
  pinMode(SCL1, OUTPUT);
  pinMode(SDA1, INPUT);

  Wire.begin(I2C_MASTER, 0x00, SDA0, SCL0);
  Wire.setClock(800000);

  Serial1.end();
  //Serial1.begin(4608000);//0.16% error rate
  Serial1.begin(1000000);//0.00% error rate - 1 byte at 1 / (2000000 / 8)

  mpu0.initMPU();
  mpu1.initMPU();
  mpu2.initMPU();
  mpu3.initMPU();
}

void parseCommand(String value) {
  if (value.equals("FullPrint:True")) {
    fullPrint = true;
    Serial1.println("FULL_PRINT_ENABLED");
  }
  else if (value.equals("FullPrint:False")) {
    fullPrint = false;
    Serial1.println("FULL_PRINT_DISABLED");
  }
  else if (value.equals("Gyro:250")) {
    mpu0.setGyroAccuracy(MPU6050_GYRO_FS_250);
    mpu1.setGyroAccuracy(MPU6050_GYRO_FS_250);
    mpu2.setGyroAccuracy(MPU6050_GYRO_FS_250);
    mpu3.setGyroAccuracy(MPU6050_GYRO_FS_250);
    Serial1.println("GYRO_250");
  }
  else if (value.equals("Gyro:500")) {
    mpu0.setGyroAccuracy(MPU6050_GYRO_FS_500);
    mpu1.setGyroAccuracy(MPU6050_GYRO_FS_500);
    mpu2.setGyroAccuracy(MPU6050_GYRO_FS_500);
    mpu3.setGyroAccuracy(MPU6050_GYRO_FS_500);
    Serial1.println("GYRO_500");
  }
  else if (value.equals("Gyro:1000")) {
    mpu0.setGyroAccuracy(MPU6050_GYRO_FS_1000);
    mpu1.setGyroAccuracy(MPU6050_GYRO_FS_1000);
    mpu2.setGyroAccuracy(MPU6050_GYRO_FS_1000);
    mpu3.setGyroAccuracy(MPU6050_GYRO_FS_1000);
    Serial1.println("GYRO_1000");
  }
  else if (value.equals("Gyro:2000")) {
    mpu0.setGyroAccuracy(MPU6050_GYRO_FS_2000);
    mpu1.setGyroAccuracy(MPU6050_GYRO_FS_2000);
    mpu2.setGyroAccuracy(MPU6050_GYRO_FS_2000);
    mpu3.setGyroAccuracy(MPU6050_GYRO_FS_2000);
    Serial1.println("GYRO_2000");
  }
  else if (value.equals("Accel:2")) {
    mpu0.setAccelAccuracy(MPU6050_ACCEL_FS_2);
    mpu1.setAccelAccuracy(MPU6050_ACCEL_FS_2);
    mpu2.setAccelAccuracy(MPU6050_ACCEL_FS_2);
    mpu3.setAccelAccuracy(MPU6050_ACCEL_FS_2);
    Serial1.println("ACCEL_2");
  }
  else if (value.equals("Accel:4")) {
    mpu0.setAccelAccuracy(MPU6050_ACCEL_FS_4);
    mpu1.setAccelAccuracy(MPU6050_ACCEL_FS_4);
    mpu2.setAccelAccuracy(MPU6050_ACCEL_FS_4);
    mpu3.setAccelAccuracy(MPU6050_ACCEL_FS_4);
    Serial1.println("ACCEL_4");
  }
  else if (value.equals("Accel:8")) {
    mpu0.setAccelAccuracy(MPU6050_ACCEL_FS_8);
    mpu1.setAccelAccuracy(MPU6050_ACCEL_FS_8);
    mpu2.setAccelAccuracy(MPU6050_ACCEL_FS_8);
    mpu3.setAccelAccuracy(MPU6050_ACCEL_FS_8);
    Serial1.println("ACCEL_8");
  }
  else if (value.equals("Accel:16")) {
    mpu0.setAccelAccuracy(MPU6050_ACCEL_FS_16);
    mpu1.setAccelAccuracy(MPU6050_ACCEL_FS_16);
    mpu2.setAccelAccuracy(MPU6050_ACCEL_FS_16);
    mpu3.setAccelAccuracy(MPU6050_ACCEL_FS_16);
    Serial1.println("ACCEL_16");
  }
  else if (value.equals("Blink:Solid")) {
    useBlink = 0;
    Serial1.println("BLINK_SOLID");
  }
  else if (value.equals("Blink:Blink")) {
    useBlink = 1;
    Serial1.println("BLINK_BLINK");
  }
  else if (value.equals("Blink:None")) {
    useBlink = 2;
    Serial1.println("BLINK_NONE");
  }
  else if (value.equals("TimePrint:True")) {
    timePrint = true;
    Serial1.println("TIME_PRINT_TRUE");
  }
  else if (value.equals("TimePrint:False")) {
    timePrint = false;
    Serial1.println("TIME_PRINT_FALSE");
  }
  else if (value.equals("DataPrint:True")) {
    dataPrint = true;
    Serial1.println("DATA_PRINT_TRUE");
  }
  else if (value.equals("DataPrint:False")) {
    dataPrint = false;
    Serial1.println("DATA_PRINT_FALSE");
  }
  else {
    Serial1.println("COMMAND_INVALID");
  }
}

void loop() {
  if (Serial1.available() > 0) {
    parseCommand(Serial1.readStringUntil('\n'));
  }

  timeMicros = micros();
  mpu0.readMPU();
  mpu1.readMPU();
  mpu2.readMPU();
  mpu3.readMPU();

  //print collected data
  if (dataPrint){
    Serial1.write(digitalReadFast(CONTROL_PIN));
    if (fullPrint) {
      Serial1.write("\t"); mpu0.printData(false); mpu1.printData(false); mpu2.printData(false); mpu3.printData(false);
    }
    else {
      mpu0.printData(true);  mpu1.printData(true);  mpu2.printData(true);  mpu3.printData(true);
    }
    Serial1.write(" .|^|. \n");
    Serial1.flush();
  }

  if (useBlink == 0) {
    digitalWriteFast(LED_PIN, HIGH);

    useBlink = 4;
  }
  else if (useBlink == 1 && previousBlink + 100000 < timeMicros) {
    previousBlink = timeMicros;
    blinkState = !blinkState;
    digitalWriteFast(LED_PIN, blinkState);
  }
  else if (useBlink == 2) {
    digitalWriteFast(LED_PIN, LOW);

    useBlink = 4;
  }

  if(timePrint){
    Serial1.println(micros() - timeMicros);
  }
}
