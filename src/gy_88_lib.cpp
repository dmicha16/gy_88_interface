#include "imu_interface/gy_88_lib.h"

// ******************************** CONSTRUCTORS-DESTRUCTORS *******************************

Gy88Interface::Gy88Interface() {}

Gy88Interface::~Gy88Interface() {}

// **************************************** PUBLIC *****************************************

bool Gy88Interface::connect_to_MPU6050(const int slave_addr, const int pwr_mgmt_addr)
{
  MPU6050_connection_ = wiringPiI2CSetup(slave_addr);
    if (MPU6050_connection_ == -1)
        return false;

  wiringPiI2CReadReg8 (MPU6050_connection_, pwr_mgmt_addr);
  wiringPiI2CWriteReg16(MPU6050_connection_, pwr_mgmt_addr, 0);

  return true;
}

bool Gy88Interface::connect_to_HMC5883L(const int slave_addr, const int pwr_mgmt_addr)
{
  HMC5883L_connection_ = wiringPiI2CSetup(slave_addr);
    if (HMC5883L_connection_ == -1)
        return false;

  wiringPiI2CReadReg8 (HMC5883L_connection_, pwr_mgmt_addr);
  wiringPiI2CWriteReg16(HMC5883L_connection_, pwr_mgmt_addr, 0);

  return true;
}

magnetometer Gy88Interface::get_magnetometer()
{
  return compass_;
}

accelerometer Gy88Interface::get_accelerometer()
{
  return accel_;
}

gyroscope Gy88Interface::get_gyroscope()
{
  return gyro_;
}

bool Gy88Interface::read_bus(const int select_chip)
{

  switch (select_chip)
  {
    case MPU6050:
      read_MPU6059_accel_();
      read_MPU6059_gyro_();
      break;

    case HMC5883L:
      read_HMC5883L_compass_();
      break;
  }
}

// **************************************** PRIVATE ****************************************

void Gy88Interface::read_MPU6059_accel_()
{
  accel_.timestamp = get_millis_since_epoch_();
  accel_.x = wiringPiI2CReadReg8(MPU6050_connection_, ACCEL_X);
  accel_.y = wiringPiI2CReadReg8(MPU6050_connection_, ACCEL_Y);
  accel_.z = wiringPiI2CReadReg8(MPU6050_connection_, ACCEL_Z);
}

void Gy88Interface::read_MPU6059_gyro_()
{
  gyro_.timestamp = get_millis_since_epoch_();
  gyro_.x = wiringPiI2CReadReg8(MPU6050_connection_, GYRO_X);
  gyro_.y = wiringPiI2CReadReg8(MPU6050_connection_, GYRO_Y);
  gyro_.z = wiringPiI2CReadReg8(MPU6050_connection_, GYRO_Z);
}

void Gy88Interface::read_HMC5883L_compass_()
{
  compass_.timestamp = get_millis_since_epoch_();
  compass_.x = wiringPiI2CReadReg8(HMC5883L_connection_, COMPASS_X);
  compass_.y = wiringPiI2CReadReg8(HMC5883L_connection_, COMPASS_Y);
  compass_.z = wiringPiI2CReadReg8(HMC5883L_connection_, COMPASS_Z);
}

unsigned long Gy88Interface::get_millis_since_epoch_()
{
  unsigned long millis_since_epoch =
    std::chrono::duration_cast<std::chrono::milliseconds>
        (std::chrono::system_clock::now().time_since_epoch()).count();

  return millis_since_epoch;
}