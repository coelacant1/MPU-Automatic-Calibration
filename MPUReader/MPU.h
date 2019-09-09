#include "i2c_t3.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

typedef struct MPU {
    uint8_t address;
    uint8_t devStatus;
    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer
    int16_t d[6];
    uint8_t sda = 19, scl = 18, gAcc, aAcc;
    MPU6050 dev;

    MPU(uint8_t address, uint8_t gAcc, uint8_t aAcc, uint8_t sda = 19, uint8_t scl = 18) {
      this->address = address;
      this->sda = sda;
      this->scl = scl;

      this->dev = MPU6050(address);
    }

    void setAccuracy(uint8_t gAcc, uint8_t aAcc){
      selectMPU();
      
      dev.setFullScaleGyroRange(gAcc);
      dev.setFullScaleAccelRange(aAcc);
    }

    void setGyroAccuracy(uint8_t gAcc){
      selectMPU();
      
      dev.setFullScaleGyroRange(gAcc);
    }

    void setAccelAccuracy(uint8_t aAcc){
      selectMPU();
      
      dev.setFullScaleAccelRange(aAcc);
    }

    void selectMPU() {
      Wire.pinConfigure(sda, scl);
    }
    
    void printInt16Arr(bool bytes, int16_t *values, byte len) {
      for (int i = 0; i < len; i++) {
        if (bytes){
          printInt16(values[i]);
        }
        else{
          Serial1.print(values[i]);
          Serial1.print("\t");
        }
      }
    }
    
    void printInt16(int16_t value) {
      byte MSB, LSB = 0;
    
      LSB = value & 0xFF;
      MSB = (value >> 8) & 0xFF;
    
      Serial1.write(MSB);
      Serial1.write(LSB);
    }

    void printData(bool bytes = false){
      printInt16Arr(bytes, d, 6);
    }

    void initMPU() {
      selectMPU();

      dev.initialize();
      Serial1.println(dev.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

      dev.resetSensors();
      
      dev.resetGyroscopePath();
      dev.resetAccelerometerPath();
      dev.resetTemperaturePath();

      dev.setAccelerometerPowerOnDelay(0);

      dev.setFIFOEnabled(false);

      dev.setDHPFMode(0);
      dev.setSlaveEnabled(0, false);
      dev.setFSyncInterruptEnabled(false);
      dev.setI2CBypassEnabled(false);
      dev.setClockOutputEnabled(false);
      dev.setIntFreefallEnabled(false);
      dev.setIntMotionEnabled(false);
      dev.setIntZeroMotionEnabled(false);
      dev.setIntFIFOBufferOverflowEnabled(false);
      dev.setIntI2CMasterEnabled(false);
      dev.setIntDataReadyEnabled(false);
      dev.setIntEnabled(false);
      dev.setSleepEnabled(false);
      dev.setWakeCycleEnabled(false);
      dev.setTempSensorEnabled(false);
      dev.setClockSource(0);

      setAccuracy(gAcc, aAcc);

      dev.setRate(0);
      
      dev.setXGyroOffset(0);  dev.setYGyroOffset(0);  dev.setZGyroOffset(0);
      dev.setXAccelOffset(0); dev.setYAccelOffset(0); dev.setZAccelOffset(0);
      /*
      uint8_t devStatus = dev.dmpInitialize();

      dev.setXGyroOffset(220); dev.setYGyroOffset(76); dev.setZGyroOffset(-85); dev.setZAccelOffset(1788);

      if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        dev.CalibrateAccel(15);
        dev.CalibrateGyro(15);
        dev.PrintActiveOffsets();

        Serial1.println(F("Enabling DMP..."));
        dev.setDMPEnabled(false);

        packetSize = dev.dmpGetFIFOPacketSize();
      } else {
        Serial1.print(F("DMP Initialization failed (code "));
        Serial1.print(devStatus);
        Serial1.println(F(")"));
      }
      */
    }

    void readMPU() {
      selectMPU();
      dev.getMotion6(&d[0], &d[1], &d[2], &d[3], &d[4], &d[5]);
    }
} MPU;
