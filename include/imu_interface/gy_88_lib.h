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

// **************************************** MPU6050 *****************************************

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

#define MPU6050_A_SCALE             16384.0
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

#define GYRO_X 0x43
#define GYRO_Y 0x45
#define GYRO_Z 0x47

#define ACCEL_X 0x3b
#define ACCEL_Y 0x3d
#define ACCEL_Z 0x3f

#define COMPASS_X 0x04
#define COMPASS_Y 0x08
#define COMPASS_Z 0x06

#define MPU6050_CHIP 0
#define HMC5883L_CHIP 1
#define BMP085_CHIP 2

struct MPU6050
{
  float accel_x;
  float accel_y;
  float accel_z;

  float gyro_x;
  float gyro_y;
  float gyro_z;

  float temp;

  unsigned long timestamp;
};

struct HMC5883L
{
  float compass_x;
  float compass_y;
  float compass_z;
  float compass_angle;

  unsigned long timestamp;
};

struct magnetometer
{
  float x;
  float y;
  float z;
  unsigned long timestamp;
};

class Gy88Interface
{
  public:
    Gy88Interface();
    ~Gy88Interface();

    bool connect_to_MPU6050();
    bool connect_to_HMC5883L();
    bool connect_to_BMP085();

    // bool connect_to_bus(char chip_1);
    // bool connect_to_bus(char chip_1, char chip_2);
    // bool connect_to_bus(char chip_1, char chip_2, char chip_3);

    MPU6050 get_MPU6050();

    bool read_bus(const int select_chip);

  private:

    std::string get_timestamp_();
    unsigned long get_millis_since_epoch_();
    int MPU6050_fd_;
    int HMC5883L_fd_;

    void read_MPU6059_accel_();
    void read_MPU6059_gyro_();
    void read_HMC5883L_compass_();

    MPU6050 MPU6050_;
};