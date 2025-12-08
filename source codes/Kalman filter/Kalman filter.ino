#include "FastIMU.h"
#include <Wire.h>

#define IMU_ADDRESS 0x68    //Change to the address of the IMU
#define PERFORM_CALIBRATION //Comment to disable startup calibration
MPU6500 IMU;               //Change to the name of any supported IMU! 



calData calib = { 0 };  //Calibration data
AccelData accelData;    //Sensor data
GyroData gyroData;

// --- FILTER VARIABLES ---
double pitch = 0, roll = 0, pitch_no=0, roll_no=0, com_pitch=0, com_roll=0;  // Orientation angles (degrees)
double dt = 0.02;                     // Loop time in seconds (50 Hz)

// --- KALMAN FILTER PARAMETERS ---
double Q_angle = 0.001;  // Process noise variance for the accelerometer
double Q_bias = 0.003;   // Process noise variance for the gyroscope bias
double R_measure = 0.03; // Measurement noise variance
double angle = 0, bias = 0, rate = 0; // Kalman filter state variables
double P[2][2] = {{0, 0}, {0, 0}};    // Error covariance matrix

unsigned long lastTime; // For calculating loop time

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

  lastTime = millis(); // Set the initial time for the loop
}


void loop() {
  IMU.update();

  IMU.getGyro(&gyroData);
  double gx = (gyroData.gyroX)/131;
  double gy = (gyroData.gyroY)/131;
  

  IMU.getAccel(&accelData);
  double ax = accelData.accelX;
  double ay = accelData.accelY;
  double az = accelData.accelZ;

  // --- CALCULATE TIME STEP ---
  unsigned long currentTime = millis();
  dt = (currentTime - lastTime) / 1000.0; // Calculate time in seconds
  if (dt == 0) dt = 0.001;               // Prevent division by zero
  lastTime = currentTime;

  // --- NO KALMAN FILTER FOR PITCH AND ROLL ---
  pitch_no = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0/PI ;

  roll_no = atan2(ay, az) * 180.0 / PI;

  // --- KALMAN FILTER FOR PITCH AND ROLL ---
  pitch = Kalman_filter(pitch, gx, atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI);
 
  roll = Kalman_filter(roll, gy, atan2(ay, az) * 180.0 / PI);

  // --- COMPLEMENTARY FILTER ---
  com_roll += gx * dt * 180 / PI; 
  com_pitch += gy * dt * 180 / PI;
  float alpha = 0.9; // adjust this value to change the filter's responsiveness
  com_roll = alpha * (com_roll + gy * dt) + (1 - alpha) * roll_no;
  com_pitch = alpha * (com_pitch + gx * dt) + (1 - alpha) * pitch_no;

  Serial.print(com_roll);
  Serial.print(" , ");
  Serial.print(com_pitch);
  Serial.print(" , ");
  Serial.print(roll);
  Serial.print(" , ");
  Serial.print(pitch);
  Serial.println();

  delay(50); // adjust to match your sampling rate
}
// --- KALMAN FILTER FUNCTION ---
double Kalman_filter(double angle, double gyroRate, double accelAngle) {
  // Predict
  rate = gyroRate - bias;
  angle += dt * rate;
 
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;
 
  // Update
  double S = P[0][0] + R_measure; // Estimate error
  double K[2];                    // Kalman gain
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;
 
  double y = accelAngle - angle; // Angle difference
  angle += K[0] * y;
  bias += K[1] * y;
 
  double P00_temp = P[0][0];
  double P01_temp = P[0][1];
 
  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;
 
  return angle;
}
