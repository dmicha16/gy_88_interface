#include <cmath>
#include <bitset>
#include <sstream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <time.h>
#include <string>
#include <string.h>
#include <iomanip>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <chrono>

// **************************************** ChipMPU6050 *****************************************

#define MPU6050_SLAVE_ADDR          0x68
#define MPU6050_PWR_MGMNT_ADDR      0x6B
#define MPU6050_REG_DATA_START      0x3b

#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_ACCEL_XOUT_L     0x3C
#define MPU6050_RA_ACCEL_YOUT_H     0x3D
#define MPU6050_RA_ACCEL_YOUT_L     0x3E
#define MPU6050_RA_ACCEL_ZOUT_H     0x3F
#define MPU6050_RA_ACCEL_ZOUT_L     0x40

#define MPU6050_RA_TEMP_OUT_H       0x41
#define MPU6050_RA_TEMP_OUT_L       0x42

#define MPU6050_RA_GYRO_XOUT_H      0x43
#define MPU6050_RA_GYRO_XOUT_L      0x44
#define MPU6050_RA_GYRO_YOUT_H      0x45
#define MPU6050_RA_GYRO_YOUT_L      0x46
#define MPU6050_RA_GYRO_ZOUT_H      0x47
#define MPU6050_RA_GYRO_ZOUT_L      0x48

#define MPU6050_A_SCALE_2G          16384.0
#define MPU6050_A_SCALE_4G          8192
#define MPU6050_A_SCALE_8G          4096
#define MPU6050_A_SCALE_16G         2048

#define MPU6050_ANG_SCALE           131.0

// **************************************** HMC5883L *****************************************

#define HMC5883L_ADDRESS            0x1e

#define HCM5883L_REG_CONFIG_A       0x00
#define HCM5883L_REG_CONFIG_B       0x01
#define HMC5883L_REG_MODE           0x02
#define HMC5883L_REG_MSB_X          0x03
#define HMC5883L_REG_LSB_X          0x04
#define HMC5883L_REG_MSB_Z          0x05
#define HMC5883L_REG_LSB_Z          0x06
#define HMC5883L_REG_MSB_Y          0x07
#define HMC5883L_REG_LSB_Y          0x08
#define HMC5883L_REG_STATUS         0x09
#define HMC5883L_REG_ID_A           0x0a
#define HMC5883L_REG_ID_B           0x0b
#define HMC5883L_REG_ID_C           0x0c

#define HMC5883L_MODE_CONTINUOUS    0x00
#define HMC5883L_MODE_SINGLE        0x01

#define MPU6050_CHIP  0
#define HMC5883L_CHIP 1
#define BMP085_CHIP   2

#define PI            3.141592

typedef unsigned long ulong_t;
typedef unsigned long long uulong_t;

struct ChipMPU6050
{
  float accel_x;
  float accel_y;
  float accel_z;

  float gyro_x;
  float gyro_y;
  float gyro_z;

  float temp;
};

struct ChipHMC5883L
{
  float compass_x;
  float compass_y;
  float compass_z;
  float compass_angle;
};

struct magnetometer
{
  float x;
  float y;
  float z;
};

class Gy88Interface
{
  public:
    Gy88Interface();
    ~Gy88Interface();

    bool connect_to_MPU6050();
    bool connect_to_HMC5883L();
    bool connect_to_BMP085();

    ChipMPU6050 get_MPU5060_data();
    ChipHMC5883L get_HMC5883L_data();

    uulong_t get_read_timestamp();

    bool read_bus(const int select_chip, float accel_resolution, float ang_scale);

  private:
    int MPU6050_fd_;
    int HMC5883L_fd_;

    uulong_t current_millis_since_epoch_;
    void set_millis_since_epoch_();

    void read_MPU6059_accel_(float accel_resolution);
    void read_MPU6059_gyro_(float ang_scale);
    void read_HMC5883L_compass_();

    float calculate_compass_angle_();

    ChipMPU6050 chip_mpu6050_;
    ChipHMC5883L chip_hmc5883l_;
};