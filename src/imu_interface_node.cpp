#include "imu_interface/gy_88_lib.h"
#include "ros/ros.h"
#include <iostream>
#include "imu_interface/Gy88Data.h"


void print_gy_88(MPU6050 MPU6050_)
{
  std::cout << std::fixed;
  std::cout << std::setprecision(15);
  std::cout << "ACCEL X:" << MPU6050_.accel_x << " -- " << \
               "ACCEL Y:" << MPU6050_.accel_y << " -- " << \
               "ACCEL Z:" << MPU6050_.accel_z << std::endl;

//   ROS_INFO("%d", MPU6050_.accel_x);
//   ROS_INFO("%d", MPU6050_.accel_y);
//   ROS_INFO("%d", MPU6050_.accel_z);

  // ROS_INFO("%f", gyro.x);
  // ROS_INFO("%f", gyro.y);
  // ROS_INFO("%f", gyro.z);

  // ROS_INFO("%f", compass.x);
  // ROS_INFO("%f", compass.y);
  // ROS_INFO("%f", compass.z);
}

unsigned long get_millis_since_epoch()
{
  unsigned long millis_since_epoch =
    std::chrono::duration_cast<std::chrono::milliseconds>
        (std::chrono::system_clock::now().time_since_epoch()).count();

  return millis_since_epoch;
}

void test_polling_speed(int test_num, Gy88Interface imu)
{
  unsigned long avg_speed = 0;
  for(size_t i = 0; i < test_num; i++)
  {
    unsigned long start_time = get_millis_since_epoch();

    for(size_t i = 0; i < 1001; i++)
    {
      imu.read_bus(MPU6050_CHIP, MPU6050_A_SCALE_2G);
      MPU6050 MPU6050_ = imu.get_MPU6050();
    }

    unsigned long end_time = get_millis_since_epoch();

    avg_speed += (end_time - start_time);

    // std::cout << "Total time it took to do a 1000 polls: " << end_time - start_time << std::endl;
  }
  avg_speed = avg_speed / test_num;

  std::cout << "This is how long it took to poll 1000, " << test_num << " times: " << avg_speed << std::endl;
}

int main(int argc, char **argv)
{
  Gy88Interface imu;

  if(!imu.connect_to_MPU6050())
    ROS_INFO("%s", "Couldn't connect to I2C bus!");
  else
    ROS_INFO("%s", "Connected to I2C bus!");

  // if(!imu.connect_to_HMC5883L(slave_addr_HMC5883L, pwr_mgmt_addr_HMC5883L))
  //   ROS_INFO("%s", "Couldn't connect to I2C bus!");
  // else
  //   ROS_INFO("%s", "Connected to I2C bus!");

  ros::init(argc, argv, "imu_interface_node");
  ros::NodeHandle n;
  ros::Publisher publisher = n.advertise<imu_interface::Gy88Data>("gy88_data", 1000);
  ros::Rate loop_rate(1);

  imu_interface::Gy88Data gy88_data;
  // test_polling_speed(5, imu);
  while(ros::ok())
  {
    imu.read_bus(MPU6050_CHIP, MPU6050_A_SCALE_2G);
    // imu.read_bus(HMC5883L);
    MPU6050 MPU6050_ = imu.get_MPU6050();

    // gy88_data.accel_x = accel.x;
    // gy88_data.accel_y = accel.y;
    // gy88_data.accel_z = accel.z;

    // gy88_data.gyro_x = gyro.x;
    // gy88_data.gyro_y = gyro.y;
    // gy88_data.gyro_z = gyro.z;

    // gy88_data.compass_x = compass.x;
    // gy88_data.compass_y = compass.y;
    // gy88_data.compass_z = compass.z;

    // publisher.publish(gy88_data);
    print_gy_88(MPU6050_);
    ros::spinOnce();
    // loop_rate.sleep();
  }
  return 0;
}