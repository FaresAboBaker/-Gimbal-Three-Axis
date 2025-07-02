#include "PID.h"
#include <Servo.h>
#include <Wire.h>

#define x_servo_offset 85
#define y_servo_offset 100

#define acc_lsb 4096.0
#define gyro_lsb 65.5

PID pid_x(0, 0.2, 0.9, 0.0);
PID pid_y(0, -0.1, -0.5, 0.0);
float PIDX_value = 0;
float PIDY_value = 0;

// PID pid_z(-180, 0.6, 0.0, 0.0);

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
void setup()
{
   Serial.begin(115200);
   servo_x.attach(servo_x_pin);
   servo_y.attach(servo_y_pin);
   servo_z.attach(servo_z_pin);
   servo_x.write(x_servo_offset);
   servo_y.write(y_servo_offset);
   delay(100);
   GyroSetup();
   // Call this function if you need to get the IMU error values for your module
   calculate_IMU_error();
   delay(20);
}
void loop()
{
   // === Read acceleromter data === //
   MeasureAcc();
   MeasureGyro();
   // Complementary filter - combine acceleromter and gyro angle values
   roll = 0.94 * gyroAngleX - 0.06 * accAngleX;
   pitch = 0.94 * gyroAngleY + 0.06 * accAngleY;

   PIDX_value = pid_x.get_result(pitch, PIDX_value);
   PIDY_value = pid_y.get_result(roll, PIDY_value);
   servo_x.write(PIDX_value + x_servo_offset);
   servo_y.write(PIDY_value + y_servo_offset);
   // servo_z.write(pid_z.get_result(yaw));

   delay(50);
   // Print the values on the serial monitor
   Serial.print(roll);
   Serial.print("\t");
   Serial.println(pitch);
   Serial.print("\t\t\t");
   
   Serial.print(PIDX_value);
   Serial.print("\t");
   Serial.println(PIDY_value);
}

void MeasureAcc(){
   Wire.beginTransmission(MPU);
   Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
   Wire.endTransmission(false);
   Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
   // For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
   AccX = (Wire.read() << 8 | Wire.read()) / acc_lsb; // X-axis value
   AccY = (Wire.read() << 8 | Wire.read()) / acc_lsb; // Y-axis value
   AccZ = (Wire.read() << 8 | Wire.read()) / acc_lsb; // Z-axis value
   // Calculating Roll and Pitch from the accelerometer data
   accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX;
   accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorY;
   // === Read gyroscope data === //
   previousTime = currentTime; // Previous time is stored before the actual time read
   currentTime = millis(); // Current time actual time read
   elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
}

void MeasureGyro(){
   Wire.beginTransmission(MPU);
   Wire.write(0x43); // Gyro data first register address 0x43
   Wire.endTransmission(false);
   Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
   GyroX = (Wire.read() << 8 | Wire.read()) / gyro_lsb;
   GyroY = (Wire.read() << 8 | Wire.read()) / gyro_lsb;
   GyroZ = (Wire.read() << 8 | Wire.read()) / gyro_lsb;
   // Correct the outputs with the calculated error values
   GyroX = GyroX - GyroErrorX; // GyroErrorX ~(-0.56)
   GyroY = GyroY - GyroErrorY; // GyroErrorY ~(2)
   GyroZ = GyroZ - GyroErrorZ; // GyroErrorZ ~ (-0.8)
   // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the
   // angle in degrees
   gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
   gyroAngleY = gyroAngleY + GyroY * elapsedTime;
   yaw = yaw + GyroZ * elapsedTime;
}

void calculate_IMU_error(){
   for (int i = 0; i < multiMeasure; i++) {
      Wire.beginTransmission(MPU);
      Wire.write(0x3B);
      Wire.endTransmission(false);
      Wire.requestFrom(MPU, 6, true);
      AccX = (Wire.read() << 8 | Wire.read()) / acc_lsb;
      AccY = (Wire.read() << 8 | Wire.read()) / acc_lsb;
      AccZ = (Wire.read() << 8 | Wire.read()) / acc_lsb;
      // Sum all readings
      AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
      AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
      delay(1);
   }
   // Divide the sum by multiMeasure to get the error value
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
      GyroErrorX = GyroErrorX + (GyroX / gyro_lsb);
      GyroErrorY = GyroErrorY + (GyroY / gyro_lsb);
      GyroErrorZ = GyroErrorZ + (GyroZ / gyro_lsb);
      delay(1);
   }
   // Divide the sum by multiMeasure to get the error value
   GyroErrorX = GyroErrorX / multiMeasure;
   GyroErrorY = GyroErrorY / multiMeasure;
   GyroErrorZ = GyroErrorZ / multiMeasure;

   // Serial.print(GyroErrorX);
   // Serial.print("\t");
   // Serial.println(GyroErrorY);
   // delay(100);
}

void GyroSetup() {
   Wire.begin();
   Wire.beginTransmission(MPU);
   Wire.write(0x6B);
   Wire.write(0x00);
   Wire.endTransmission(true);

   Wire.beginTransmission(MPU);
   Wire.write(0x1C);
   Wire.write(0x10);
   Wire.endTransmission(true);

   Wire.beginTransmission(MPU);
   Wire.write(0x1B);
   Wire.write(0x08);
   Wire.endTransmission(true);

   Wire.beginTransmission(MPU);
   Wire.write(0x1A);
   Wire.write(0x04);
   Wire.endTransmission(true);

   delay(20);
}