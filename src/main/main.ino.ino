#include <Wire.h>
#include <Servo.h>
#include "PID.h"

#define x_servo_offset 85
#define y_servo_offset 95

PID pid_x(0, 2.6 , 0.0235 , 1.175);
PID pid_y(0, -1.7 , 0.02, 1.2);
//PID pid_z(0, 0.5, 0, 0);

Servo servo_x;
Servo servo_y;
Servo servo_z;

uint8_t servo_x_pin = 9;
uint8_t servo_y_pin = 10;
uint8_t servo_z_pin = 11;


#define multiMeasure 700
#define MPU 0x68 // MPU6050 I2C address

float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
void setup() {
  Serial.begin(9600);
  servo_x.attach(servo_x_pin);
  servo_y.attach(servo_y_pin);
  servo_z.attach(servo_z_pin);
  servo_x.write(85);
  servo_y.write(95);
  delay(1000);
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  
  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  // Wire.beginTransmission(MPU);
  // Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
  // Wire.write(0x10);                  //Set the register bits as 00010000 (+/- 8g full scale range)
  // Wire.endTransmission(true);
  // // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  // Wire.beginTransmission(MPU);
  // Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  // Wire.write(0x10);                   // Set the register bits as 00010000 (1000deg/s full scale)
  // Wire.endTransmission(true);
  // delay(20);
  
  // Call this function if you need to get the IMU error values for your module
  calculate_IMU_error();
  delay(20);
}
void loop() {
  // === Read acceleromter data === //
  
  MeasureAcc();
  MeasureGyro();



  // Complementary filter - combine acceleromter and gyro angle values
  roll = 0.95 * gyroAngleX + 0.05 * accAngleX;
  pitch = 0.95 * gyroAngleY + 0.05 * accAngleY;
  servo_x.write(pid_x.get_result(pitch) + x_servo_offset);
  servo_y.write(pid_y.get_result(roll ) + y_servo_offset);
  delay(20);
  // // Print the values on the serial monitor
  Serial.print(roll);
  Serial.print("\t");
  Serial.println(pitch);
  // delay(100);
  // Serial.print("/");
  // Serial.println(yaw);
}

void MeasureAcc(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorY; // AccErrorY ~(-1.58)
  // === Read gyroscope data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
}

void MeasureGyro(){
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  // Correct the outputs with the calculated error values
  GyroX = GyroX - GyroErrorX; // GyroErrorX ~(-0.56)
  GyroY = GyroY - GyroErrorY; // GyroErrorY ~(2)
  GyroZ = GyroZ - GyroErrorZ; // GyroErrorZ ~ (-0.8)
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw =  yaw + GyroZ * elapsedTime;
}


void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values multiMeasure times
  for (int i = 0; i < multiMeasure; i++) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    delay(1);
  }
  //Divide the sum by multiMeasure to get the error value
  AccErrorX = AccErrorX / multiMeasure;
  AccErrorY = AccErrorY / multiMeasure;
  // Read gyro values multiMeasure times
  for (int i = 0; i < multiMeasure; i++) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    delay(1);
  }
  //Divide the sum by multiMeasure to get the error value
  GyroErrorX = GyroErrorX / multiMeasure;
  GyroErrorY = GyroErrorY / multiMeasure;
  GyroErrorZ = GyroErrorZ / multiMeasure;

  // Serial.print(GyroErrorX);
  // Serial.print("\t");
  // Serial.println(GyroErrorY);
  // delay(100);

}