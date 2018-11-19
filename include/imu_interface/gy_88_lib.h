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

#define GYRO_X 0x43
#define GYRO_Y 0x45
#define GYRO_Z 0x47

#define ACCEL_X 0x3b
#define ACCEL_Y 0x3d
#define ACCEL_Z 0x3f

#define COMPASS_X 0x00
#define COMPASS_Y 0x00
#define COMPASS_Z 0x00

#define MPU6050_I2C_ADDRESS 0x68
#define MPU6050_PWR_MGMT_1 0x6B

#define MPU6050 0
#define HMC5883L 1

struct magnetometer
{
  float x;
  float y;
  float z;
  unsigned long timestamp;
};

struct gyroscope
{
  float x;
  float y;
  float z;
  unsigned long timestamp;
};

struct accelerometer
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

    bool connect_to_MPU6050(const int slave_addr, const int pwr_mgmt_addr);
    bool connect_to_HMC5883L(const int slave_addr, const int pwr_mgmt_addr);

    magnetometer get_magnetometer();
    accelerometer get_accelerometer();
    gyroscope get_gyroscope();

    bool read_bus(const int select_chip);

  private:

    std::string get_timestamp_();
    unsigned long get_millis_since_epoch_();
    int MPU6050_connection_;
    int HMC5883L_connection_;

    void read_MPU6059_accel_();
    void read_MPU6059_gyro_();
    void read_HMC5883L_compass_();

    accelerometer accel_;
    gyroscope gyro_;
    magnetometer compass_;
};