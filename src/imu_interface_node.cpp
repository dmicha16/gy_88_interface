#include "imu_interface/gy_88_lib.h"
#include "ros/ros.h"
#include <iostream>
#include "imu_interface/Gy88Data.h"

ulong_t get_millis_since_epoch()
{
  ulong_t millis_since_epoch =
    std::chrono::duration_cast<std::chrono::milliseconds>
        (std::chrono::system_clock::now().time_since_epoch()).count();

  return millis_since_epoch;
}

void test_polling_speed(int test_num, Gy88Interface imu)
{
  ulong_t avg_speed = 0;

  ChipMPU6050 chip_mpu6050;
  ChipHMC5883L chip_hmc5883l;
  for(size_t i = 0; i < test_num; i++)
  {
    ulong_t start_time = get_millis_since_epoch();

    for(size_t i = 0; i < 1001; i++)
    {
      imu.read_bus(MPU6050_CHIP, MPU6050_A_SCALE_2G, MPU6050_ANG_SCALE);
      imu.read_bus(HMC5883L_CHIP, MPU6050_A_SCALE_2G, MPU6050_ANG_SCALE);
      chip_mpu6050 = imu.get_MPU5060_data();
      chip_hmc5883l = imu.get_HMC5883L_data();
    }

    ulong_t end_time = get_millis_since_epoch();

    avg_speed += (end_time - start_time);

    // std::cout << "Total time it took to do a 1000 polls: " << end_time - start_time << std::endl;
  }
  avg_speed = avg_speed / test_num;

  std::cout << "This is how long it took to poll 1000, " << test_num << " times: " << avg_speed << std::endl;
}

int main(int argc, char **argv)
{

  ROS_INFO("%s", "Constructing IMU class..");
  Gy88Interface imu;
  ROS_INFO("%s", "Successfully constructed IMU class..");

  if(!imu.connect_to_MPU6050())
    ROS_ERROR("%s", "Couldn't connect to MPU650's I2C bus!");
  else
    ROS_INFO("%s", "Connected to MPU650's I2C bus!");

  if(!imu.connect_to_HMC5883L())
    ROS_ERROR("%s", "Couldn't connect to HMC5883L's to I2C bus!");
  else
    ROS_INFO("%s", "Connected to HMC5883L's I2C bus!");

  ros::init(argc, argv, "imu_interface_node");
  ros::NodeHandle n;
  ros::Publisher publisher = n.advertise<imu_interface::Gy88Data>("gy88_data", 1000);
  ros::Rate loop_rate(1);

  imu_interface::Gy88Data gy88_data;

  while(ros::ok())
  {
    ROS_INFO_STREAM_ONCE("Started advertising on topic gy_88_data..");

    imu.read_bus(MPU6050_CHIP, MPU6050_A_SCALE_2G, MPU6050_ANG_SCALE);
    imu.read_bus(HMC5883L_CHIP, MPU6050_A_SCALE_2G, MPU6050_ANG_SCALE);

    ChipMPU6050 chip_mpu6050 = imu.get_MPU5060_data();
    ChipHMC5883L chip_hmc5883l = imu.get_HMC5883L_data();

    gy88_data.accel_x = chip_mpu6050.accel_x;
    gy88_data.accel_y = chip_mpu6050.accel_y;
    gy88_data.accel_z = chip_mpu6050.accel_z;

    gy88_data.gyro_x = chip_mpu6050.gyro_x;
    gy88_data.gyro_y = chip_mpu6050.gyro_y;
    gy88_data.gyro_z = chip_mpu6050.gyro_z;

    gy88_data.compass_x = chip_hmc5883l.compass_x;
    gy88_data.compass_y = chip_hmc5883l.compass_y;
    gy88_data.compass_z = chip_hmc5883l.compass_z;
    gy88_data.compass_angle = chip_hmc5883l.compass_angle;

    gy88_data.timestamp = imu.get_read_timestamp();

    publisher.publish(gy88_data);
    // print_gy_88(chip_mpu6050, chip_hmc5883l);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}