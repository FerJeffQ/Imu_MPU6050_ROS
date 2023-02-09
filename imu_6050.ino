#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include "Simple_MPU6050.h"
#define MPU6050_DEFAULT_ADDRESS     0x68 // address pin low (GND)

Simple_MPU6050 mpu;

ros::NodeHandle nh;

sensor_msgs::Imu Imu_msg;
ros::Publisher imu_pub("imu/data", &Imu_msg);

// See mpu.on_FIFO(print_Values); in the Setup Loop

unsigned long currentMillis;
unsigned long prevMillis;

void imu_data(int16_t *gyro, int16_t *accel, int32_t *quat) {
  Quaternion q;
  VectorFloat gravity;
  float ypr[3] = { 0, 0, 0 };
  float xyz[3] = { 0, 0, 0 };
  mpu.GetQuaternion(&q, quat);
  mpu.GetGravity(&gravity, &q);
  mpu.GetYawPitchRoll(ypr, &q, &gravity);
  mpu.ConvertToDegrees(ypr, xyz);

  Imu_msg.linear_acceleration.x = accel[0];
  Imu_msg.linear_acceleration.y = accel[1];
  Imu_msg.linear_acceleration.z = accel[2];

  Imu_msg.angular_velocity.x = gyro[0];
  Imu_msg.angular_velocity.y = gyro[1];
  Imu_msg.angular_velocity.z = gyro[2];

  Imu_msg.orientation.w = q.w;
  Imu_msg.orientation.x = q.x;
  Imu_msg.orientation.y = q.y;
  Imu_msg.orientation.z = q.z;

  imu_pub.publish(&Imu_msg);
  nh.spinOnce();
}



void setup() {
  // initialize serial communication
  Serial.begin(57600);
  
  nh.initNode();
  nh.advertise(imu_pub);
 
   // Setup the MPU and TwoWire aka Wire library all at once
  mpu.begin();
  

  
  mpu.Set_DMP_Output_Rate_Hz(10);          // Set the DMP output rate from 200Hz to 5 Minutes.
  mpu.Set_DMP_Output_Rate_Seconds(10);   // Set the DMP output rate in Seconds
  mpu.Set_DMP_Output_Rate_Minutes(5);    // Set the DMP output rate in Minute
  mpu.CalibrateMPU();                      // Calibrates the MPU.
  mpu.load_DMP_Image();                    // Loads the DMP image into the MPU and finish configuration.
  mpu.on_FIFO(imu_data);               // Set callback function that is triggered when FIFO Data is retrieved

  
}



void loop() {
  currentMillis = millis();
  if (currentMillis - prevMillis >= 10){
    prevMillis = currentMillis;
    mpu.dmp_read_fifo(false);
  }
}
