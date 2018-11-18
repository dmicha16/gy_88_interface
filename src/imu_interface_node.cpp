#include "gy_88_lib.h"
#include "ros/ros.h"
#include <iostream>
#include "imu_interface/Gy88Data.h"


void print_gy_88(accelerometer accel, gyroscope gyro)
{
  ROS_INFO("%f", accel.x);
  ROS_INFO("%f", accel.y);
  ROS_INFO("%f", accel.z);

  ROS_INFO("%f", gyro.x);
  ROS_INFO("%f", gyro.y);
  ROS_INFO("%f", gyro.z);
}

int main(int argc, char **argv)
{
  Gy88Interface imu;

  int slave_addr = 0x68;
  int pwr_mgmt_addr = 0x6B;

  if(!imu.connect_to_MPU6050(slave_addr, pwr_mgmt_addr))
    ROS_INFO("%s", "Couldn't connect to I2C bus!");
  else
    ROS_INFO("%s", "Connected to I2C bus!");

  ros::init(argc, argv, "imu_interface_node");
  ros::NodeHandle n;
  ros::Publisher publisher = n.advertise<imu_interface::Gy88Data>("gy88_data", 1000);
  ros::Rate loop_rate(1);

  imu_interface::Gy88Data gy88_data;

  while(ros::ok())
  {
    imu.read_bus(MPU6050);
    accelerometer accel = imu.get_accelerometer();
    gyroscope gyro = imu.get_gyroscope();

    gy88_data.accel_x = accel.x;
    gy88_data.accel_y = accel.y;
    gy88_data.accel_z = accel.z;

    gy88_data.gyro_x = gyro.x;
    gy88_data.gyro_y = gyro.y;
    gy88_data.gyro_z = gyro.z;

    publisher.publish(gy88_data);
    print_gy_88(accel, gyro);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}