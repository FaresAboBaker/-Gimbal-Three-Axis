#ifndef diMPU_H
#define diMPU_H

#define multiMeasure 700
#define MPU 0x68 // MPU6050 I2C address

class diMPU{
private:
    float AccX, AccY, AccZ;
    float GyroX, GyroY, GyroZ;
    float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
    float roll, pitch, yaw;
    float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
    float elapsedTime, currentTime, previousTime;
public:
    diMPU();
    void init();
    void calculate_IMU_error();
    void MeasureGyro();
    void MeasureAcc();
    float get_Roll();
    float get_Pitch();
    float get_Yaw();
    void resetGyro();
};

#endif