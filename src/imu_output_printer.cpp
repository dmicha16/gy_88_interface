#include "ros/ros.h"
#include <iostream>
#include "imu_interface/gy_88_lib.h"

void print_gy_88(ChipMPU6050 chip_mpu6050, ChipHMC5883L chip_hmc5883l)
{
  ROS_INFO_STREAM(std::fixed);
  ROS_INFO_STREAM(std::setprecision(5));
  ROS_INFO_STREAM("A_X: " << chip_mpu6050.accel_x << " - " <<
                  "A_Y: " << chip_mpu6050.accel_x << " - " <<
                  "A_Z: " << chip_mpu6050.accel_x << " - ")

  ROS_INFO_STREAM("C_X:" << chip_hmc5883l.compass_x << " - " << \
                  "C_Y:" << chip_hmc5883l.compass_y << " - " << \
                  "C_Z:" << chip_hmc5883l.compass_z << " - " << \
                  "C_A:" << chip_hmc5883l.compass_angle)

//   ROS_INFO("%d", chip_mpu6050.accel_x);
//   ROS_INFO("%d", chip_mpu6050.accel_y);
//   ROS_INFO("%d", chip_mpu6050.accel_z);

  // ROS_INFO("%f", gyro.x);
  // ROS_INFO("%f", gyro.y);
  // ROS_INFO("%f", gyro.z);

  // ROS_INFO("%f", compass.x);
  // ROS_INFO("%f", compass.y);
  // ROS_INFO("%f", compass.z);
}