#include "PID.h"
#include <Servo.h>
#include "diMPU.h"

#define x_servo_offset 85
#define y_servo_offset 100

#define acc_lsb 4096.0
#define gyro_lsb 65.5

diMPU mpu;

PID pid_x(0, 0.0, 0.0, 0.0);
PID pid_y(0, 0.0, 0.0, 0.0);
float PIDX_value = 0;
float PIDY_value = 0;

// PID pid_z(-180, 0.6, 0.0, 0.0);

Servo servo_x;
Servo servo_y;
Servo servo_z;

uint8_t servo_x_pin = 9;
uint8_t servo_y_pin = 10;
uint8_t servo_z_pin = 11;

void setup()
{
   // Serial.begin(115200);
   servo_x.attach(servo_x_pin);
   servo_y.attach(servo_y_pin);
   servo_z.attach(servo_z_pin);
   servo_x.write(x_servo_offset);
   servo_y.write(y_servo_offset);
   delay(100);
   mpu.init();
   mpu.calculate_IMU_error();
   delay(20);
}
void loop()
{
   PIDX_value = pid_x.get_result(mpu.get_Pitch(), PIDX_value);
   PIDY_value = pid_y.get_result(mpu.get_Roll(), PIDY_value);
   servo_x.write(PIDX_value + x_servo_offset);
   servo_y.write(PIDY_value + y_servo_offset);
   // servo_z.write(pid_z.get_result(yaw));

   delay(50);
   // // Print the values on the serial monitor
   // Serial.print(roll);
   // Serial.print("\t");
   // Serial.println(pitch);
   // Serial.print("\t\t\t");
   // Serial.print(PIDX_value);
   // Serial.print("\t");
   // Serial.println(PIDY_value);
}