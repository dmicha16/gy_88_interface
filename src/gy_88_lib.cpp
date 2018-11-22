#include "imu_interface/gy_88_lib.h"
#include <iostream>

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

  wiringPiI2CWriteReg16(HMC5883L_fd_, HMC5883L_REG_MODE, HMC5883L_MODE_CONTINUOUS);

  return true;
}

int Gy88Interface::set_MPU6050_full_scale_range(int range)
{
  wiringPiI2CWriteReg8(MPU6050_fd_, MPU6050_ACCEL_CONFIG, range);
  int set_range = wiringPiI2CReadReg8(MPU6050_fd_, MPU6050_ACCEL_CONFIG);

  set_MPU6050_full_scale_range_(range);

  return set_range;
}

ChipMPU6050 Gy88Interface::get_MPU5060_data()
{
  return chip_mpu6050_;
}

ChipHMC5883L Gy88Interface::get_HMC5883L_data()
{
  return chip_hmc5883l_;
}

bool Gy88Interface::read_bus(const int select_chip, float ang_scale)
{

  set_millis_since_epoch_();

  switch (select_chip)
  {
    case MPU6050_CHIP:
      read_MPU6059_accel_();
      read_MPU6059_gyro_(ang_scale);
      break;

    case HMC5883L_CHIP:
      read_HMC5883L_compass_();
      break;
  }
}

uulong_t Gy88Interface::get_read_timestamp()
{
  return current_millis_since_epoch_;
}

// **************************************** PRIVATE ****************************************

void Gy88Interface::set_MPU6050_full_scale_range_(int range)
{
  switch (range)
  {
    case MPU6050_ACCEL_CONFIG_2G:
      accel_scale_range_ = MPU6050_A_SCALE_2G;
      break;
    case MPU6050_ACCEL_CONFIG_4G:
      accel_scale_range_ = MPU6050_A_SCALE_4G;
      break;
    case MPU6050_ACCEL_CONFIG_8G:
      accel_scale_range_ = MPU6050_A_SCALE_8G;
      break;
    case MPU6050_ACCEL_CONFIG_16G:
      accel_scale_range_ = MPU6050_A_SCALE_16G;
      break;
  }
}

void Gy88Interface::read_MPU6059_accel_()
{
  short msb = wiringPiI2CReadReg8(MPU6050_fd_, MPU6050_RA_ACCEL_XOUT_H);
  short lsb = wiringPiI2CReadReg8(MPU6050_fd_, MPU6050_RA_ACCEL_XOUT_L);

  chip_mpu6050_.accel_x = convert_bytes_to_short_(msb, lsb) / accel_scale_range_;

  msb = wiringPiI2CReadReg8(MPU6050_fd_, MPU6050_RA_ACCEL_YOUT_H);
  lsb = wiringPiI2CReadReg8(MPU6050_fd_, MPU6050_RA_ACCEL_YOUT_L);

  chip_mpu6050_.accel_y = convert_bytes_to_short_(msb, lsb) / accel_scale_range_;

  msb = wiringPiI2CReadReg8(MPU6050_fd_, MPU6050_RA_ACCEL_ZOUT_H);
  lsb = wiringPiI2CReadReg8(MPU6050_fd_, MPU6050_RA_ACCEL_ZOUT_L);

  chip_mpu6050_.accel_z = convert_bytes_to_short_(msb, lsb) / accel_scale_range_;
}

int Gy88Interface::convert_bytes_to_short_(short msb, short lsb)
{
  long t = msb * 0x100L + lsb;
  if(t >= 32768)
    t -= 65536;
  return (int)t;
}

void Gy88Interface::read_MPU6059_gyro_(float ang_scale)
{
  short msb = wiringPiI2CReadReg8(MPU6050_fd_, MPU6050_RA_GYRO_XOUT_H);
  short lsb = wiringPiI2CReadReg8(MPU6050_fd_, MPU6050_RA_GYRO_XOUT_L);

  chip_mpu6050_.gyro_x = convert_bytes_to_short_(msb, lsb) / ang_scale;

  msb = wiringPiI2CReadReg8(MPU6050_fd_, MPU6050_RA_GYRO_YOUT_H);
  lsb = wiringPiI2CReadReg8(MPU6050_fd_, MPU6050_RA_GYRO_YOUT_L);

  chip_mpu6050_.gyro_y = convert_bytes_to_short_(msb, lsb) / ang_scale;

  msb = wiringPiI2CReadReg8(MPU6050_fd_, MPU6050_RA_GYRO_ZOUT_H);
  lsb = wiringPiI2CReadReg8(MPU6050_fd_, MPU6050_RA_GYRO_ZOUT_L);

  chip_mpu6050_.gyro_z = convert_bytes_to_short_(msb, lsb) / ang_scale;
}

void Gy88Interface::read_HMC5883L_compass_()
{
  short msb = wiringPiI2CReadReg8(HMC5883L_fd_, HMC5883L_REG_MSB_X);
  short lsb = wiringPiI2CReadReg8(HMC5883L_fd_, HMC5883L_REG_LSB_X);

  chip_hmc5883l_.compass_x = msb << 8 | lsb;

  msb = wiringPiI2CReadReg8(HMC5883L_fd_, HMC5883L_REG_MSB_Y);
  lsb = wiringPiI2CReadReg8(HMC5883L_fd_, HMC5883L_REG_LSB_Y);

  chip_hmc5883l_.compass_y = msb << 8 | lsb;

  msb = wiringPiI2CReadReg8(HMC5883L_fd_, HMC5883L_REG_MSB_Z);
  lsb = wiringPiI2CReadReg8(HMC5883L_fd_, HMC5883L_REG_LSB_Z);

  chip_hmc5883l_.compass_z = msb << 8 | lsb;

  chip_hmc5883l_.compass_angle = calculate_compass_angle_();
}

float Gy88Interface::calculate_compass_angle_()
{
  float angle;
  angle = atan2(chip_hmc5883l_.compass_y, chip_hmc5883l_.compass_x) \
   * (180 / PI) + 180;

  return angle;
}

void Gy88Interface::set_millis_since_epoch_()
{
  current_millis_since_epoch_ = std::chrono::duration_cast<std::chrono::milliseconds>
    (std::chrono::system_clock::now().time_since_epoch()).count();
}