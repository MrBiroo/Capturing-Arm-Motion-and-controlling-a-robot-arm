#include "FastIMU.h"
#include <Wire.h>

#define IMU_ADDRESS 0x68    //Change to the address of the IMU
#define PERFORM_CALIBRATION //Comment to disable startup calibration
MPU6500 IMU;               //Change to the name of any supported IMU! 

// Currently supported IMUS: MPU9255 MPU9250 MPU6886 MPU6500 MPU6050 ICM20689 ICM20690 BMI055 BMX055 BMI160 LSM6DS3 LSM6DSL QMI8658

calData calib = { 0 };  //Calibration data
AccelData accelData;    //Sensor data
GyroData gyroData;
MagData magData;

void setup() {
  Wire.begin();
  Wire.setClock(400000); //400khz clock
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  int err = IMU.init(calib, IMU_ADDRESS);
  if (err != 0) {
    Serial.print("Error initializing IMU: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }
  
#ifdef PERFORM_CALIBRATION
  Serial.println("FastIMU calibration & data example");
  delay(5000);
  Serial.println("Keep IMU level.");
  delay(5000);
  IMU.calibrateAccelGyro(&calib);
  Serial.println("Calibration done!");
  Serial.println("Accel biases X/Y/Z: ");
  Serial.print(calib.accelBias[0]);
  Serial.print(", ");
  Serial.print(calib.accelBias[1]);
  Serial.print(", ");
  Serial.println(calib.accelBias[2]);
  Serial.println("Gyro biases X/Y/Z: ");
  Serial.print(calib.gyroBias[0]);
  Serial.print(", ");
  Serial.print(calib.gyroBias[1]);
  Serial.print(", ");
  Serial.println(calib.gyroBias[2]);
  
  delay(5000);
  IMU.init(calib, IMU_ADDRESS);
#endif

  //err = IMU.setGyroRange(500);      //USE THESE TO SET THE RANGE, IF AN INVALID RANGE IS SET IT WILL RETURN -1
  //err = IMU.setAccelRange(2);       //THESE TWO SET THE GYRO RANGE TO ±500 DPS AND THE ACCELEROMETER RANGE TO ±2g
  
  if (err != 0) {
    Serial.print("Error Setting range: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }
}

float roll = 0;
float pitch = 0;
float roll_accel = 0;
float pitch_accel = 0;
float pitch_gyro = 0;
float roll_gyro = 0;

void loop() {
  IMU.update();

  IMU.getGyro(&gyroData);
  float gx = (gyroData.gyroX)/131;
  float gy = (gyroData.gyroY)/131;
  

  IMU.getAccel(&accelData);
  float ax = accelData.accelX;
  float ay = accelData.accelY;
  float az = accelData.accelZ;

  // Calculate roll and pitch from accelerometer data
  roll_accel = atan2(ay, az) * RAD_TO_DEG;
  pitch_accel = atan(-ax / sqrt(ay * ay + az * az)) * RAD_TO_DEG;

  // Calculate roll and pitch from gyroscope data
  roll += gy * 0.05 * RAD_TO_DEG; 
  pitch += gx * 0.05 * RAD_TO_DEG;

  roll_gyro += gx * 0.1 * 180 / PI; 
  pitch_gyro += gy * 0.1 * 180 / PI;

  // Complementary filter
  float alpha = 0.9; // adjust this value to change the filter's responsiveness
  roll = alpha * (roll + gy * 0.05) + (1 - alpha) * roll_accel;
  pitch = alpha * (pitch + gx * 0.05) + (1 - alpha) * pitch_accel;

  
  Serial.print(pitch_accel);
  Serial.print(" , ");
  Serial.print(pitch_gyro);
  Serial.print(" , ");
  Serial.print(roll_accel);
  Serial.print(" , ");
  Serial.print(roll_gyro);
  Serial.println();

  delay(50); // adjust to match your sampling rate
}
