#include <Wire.h>
#include "diMPU.h"
#include <math.h>
#include <Arduino.h> 
// Acceleration scaling factor for ±8g
#define ACCEL_SCALE 4096.0
// Gyro scaling factor for ±500deg/s
#define GYRO_SCALE 65.5

diMPU::diMPU() {
    resetGyro();
}
void diMPU::init(){
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

void diMPU::calculate_IMU_error() {
    // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
    // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
    // Read accelerometer values multiMeasure times
    AccErrorX = 0;
    AccErrorY = 0;
    GyroErrorX = 0;
    GyroErrorY = 0;
    GyroErrorZ = 0;
    for (int i = 0; i < multiMeasure; i++) {
        Wire.beginTransmission(MPU);
        Wire.write(0x3B);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU, 6, true);
        AccX = (Wire.read() << 8 | Wire.read()) / ACCEL_SCALE ;
        AccY = (Wire.read() << 8 | Wire.read()) / ACCEL_SCALE ;
        AccZ = (Wire.read() << 8 | Wire.read()) / ACCEL_SCALE ;
        // Sum all readings
        AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
        AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
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
        GyroErrorX = GyroErrorX + (GyroX / GYRO_SCALE);
        GyroErrorY = GyroErrorY + (GyroY / GYRO_SCALE);
        GyroErrorZ = GyroErrorZ + (GyroZ / GYRO_SCALE);
        delay(1);
    }
    //Divide the sum by multiMeasure to get the error value
    GyroErrorX = GyroErrorX / multiMeasure;
    GyroErrorY = GyroErrorY / multiMeasure;
    GyroErrorZ = GyroErrorZ / multiMeasure;
}

void diMPU::MeasureGyro() {
    static unsigned long previousTime = millis();
    unsigned long currentTime = millis();
    elapsedTime = (currentTime - previousTime) / 1000.0; 
    previousTime = currentTime;

    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = (Wire.read() << 8 | Wire.read()) / GYRO_SCALE;
    GyroY = (Wire.read() << 8 | Wire.read()) / GYRO_SCALE;
    GyroZ = (Wire.read() << 8 | Wire.read()) / GYRO_SCALE;

    GyroX -= GyroErrorX;
    GyroY -= GyroErrorY;
    GyroZ -= GyroErrorZ;

    gyroAngleX += GyroX * elapsedTime;
    gyroAngleY += GyroY * elapsedTime;
    yaw += GyroZ * elapsedTime;
}

void diMPU::MeasureAcc() {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
    //For a range of +-2g, we need to divide the raw values by 4096, according to the datasheet
    AccX = (Wire.read() << 8 | Wire.read()) / ACCEL_SCALE; // X-axis value
    AccY = (Wire.read() << 8 | Wire.read()) / ACCEL_SCALE; // Y-axis value
    AccZ = (Wire.read() << 8 | Wire.read()) / ACCEL_SCALE; // Z-axis value
    // Calculating Roll and Pitch from the accelerometer data
    accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX; 
    accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorY;
}

float diMPU::get_Roll(){
    this->MeasureGyro();
    this->MeasureAcc();
    return 0.95 * gyroAngleX + 0.05 * accAngleX;
}
float diMPU::get_Pitch(){
    this->MeasureGyro();
    this->MeasureAcc();
    return 0.95 * gyroAngleY + 0.05 * accAngleY;
}
float diMPU::get_Yaw(){
    this->MeasureGyro();
    this->MeasureAcc();
    yaw = fmod(yaw, 360.0);
    return yaw;
}

void diMPU::resetGyro(){
    yaw = 0;
    gyroAngleX = gyroAngleY = 0;
}
