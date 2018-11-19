#include "imu_interface/gy_88_lib.h"

// ******************************** CONSTRUCTORS-DESTRUCTORS *******************************

Gy88Interface::Gy88Interface()
{
  wiringPiSetup();
}

Gy88Interface::~Gy88Interface() {}

// **************************************** PUBLIC *****************************************

bool Gy88Interface::connect_to_MPU6050()
{
  MPU6050_fd_ = wiringPiI2CSetup(MPU6050_SLAVE_ADDR);
    if (MPU6050_fd_ == -1)
        return false;

  wiringPiI2CWriteReg16(MPU6050_fd_, MPU6050_PWR_MGMNT_ADDR, 0);

  return true;
}

bool Gy88Interface::connect_to_HMC5883L()
{
  HMC5883L_fd_ = wiringPiI2CSetup(HMC5883L_ADDRESS);
    if (HMC5883L_fd_ == -1)
        return false;

  // wiringPiI2CWriteReg16(HMC5883L_fd_, pwr_mgmt_addr, 0);

  return true;
}

MPU6050 Gy88Interface::get_MPU6050()
{
  return MPU6050_;
}

bool Gy88Interface::read_bus(const int select_chip)
{

  switch (select_chip)
  {
    case MPU6050_CHIP:
      read_MPU6059_accel_();
      read_MPU6059_gyro_();
      break;

    case HMC5883L_CHIP:
      // read_HMC5883L_compass_();
      break;
  }
}

// **************************************** PRIVATE ****************************************

void Gy88Interface::read_MPU6059_accel_()
{

  short msb = wiringPiI2CReadReg8(MPU6050_fd_, MPU6050_RA_ACCEL_XOUT_H);
  short lsb = wiringPiI2CReadReg8(MPU6050_fd_, MPU6050_RA_ACCEL_XOUT_L);

  MPU6050_.accel_x = (msb << 8 | lsb) / MPU6050_A_SCALE;

  msb = wiringPiI2CReadReg8(MPU6050_fd_, MPU6050_RA_ACCEL_YOUT_H);
  lsb = wiringPiI2CReadReg8(MPU6050_fd_, MPU6050_RA_ACCEL_YOUT_L);

  MPU6050_.accel_y = (msb << 8 | lsb) / MPU6050_A_SCALE;

  msb = wiringPiI2CReadReg8(MPU6050_fd_, MPU6050_RA_ACCEL_ZOUT_H);
  lsb = wiringPiI2CReadReg8(MPU6050_fd_, MPU6050_RA_ACCEL_ZOUT_L);

  MPU6050_.accel_z = (msb << 8 | lsb) / MPU6050_A_SCALE;

  MPU6050_.timestamp = get_millis_since_epoch_();
}

void Gy88Interface::read_MPU6059_gyro_()
{
  short msb = wiringPiI2CReadReg8(MPU6050_fd_, MPU6050_RA_GYRO_XOUT_H);
  short lsb = wiringPiI2CReadReg8(MPU6050_fd_, MPU6050_RA_GYRO_XOUT_L);

  MPU6050_.gyro_x = (msb << 8 | lsb) / MPU6050_ANG_SCALE;

  msb = wiringPiI2CReadReg8(MPU6050_fd_, MPU6050_RA_GYRO_YOUT_H);
  lsb = wiringPiI2CReadReg8(MPU6050_fd_, MPU6050_RA_GYRO_YOUT_L);

  MPU6050_.gyro_y = (msb << 8 | lsb) / MPU6050_ANG_SCALE;

  msb = wiringPiI2CReadReg8(MPU6050_fd_, MPU6050_RA_GYRO_ZOUT_H);
  lsb = wiringPiI2CReadReg8(MPU6050_fd_, MPU6050_RA_GYRO_ZOUT_L);

  MPU6050_.gyro_z = (msb << 8 | lsb) / MPU6050_ANG_SCALE;

  MPU6050_.timestamp = get_millis_since_epoch_();
}

// void Gy88Interface::read_HMC5883L_compass_()
// {
//   compass_.timestamp = get_millis_since_epoch_();
//   compass_.x = wiringPiI2CReadReg8(HMC5883L_fd_, COMPASS_X);
//   compass_.y = wiringPiI2CReadReg8(HMC5883L_fd_, COMPASS_Y);
//   compass_.z = wiringPiI2CReadReg8(HMC5883L_fd_, COMPASS_Z);
// }

unsigned long Gy88Interface::get_millis_since_epoch_()
{
  unsigned long millis_since_epoch =
    std::chrono::duration_cast<std::chrono::milliseconds>
        (std::chrono::system_clock::now().time_since_epoch()).count();

  return millis_since_epoch;
}