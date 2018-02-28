#define USE_USBCON

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>

#include <Wire.h>
#include <LSM303.h>

LSM303 lsm303;

ros::NodeHandle_<ArduinoHardware, 2, 2, 125, 125> nh;

std_msgs::Int32MultiArray imu_msg;
long int imu_data[8];
ros::Publisher imu_pub("/imu_raw", &imu_msg);

void setup()
{
  nh.initNode();

  imu_msg.data = (long int *) &imu_data;
  imu_msg.data_length = 8;
  nh.advertise(imu_pub);
  nh.spinOnce();

  Wire.begin();
  
  /* Initialize the sensor */
  if(iicScan() > 0) {
    lsm303.init();
    lsm303.enableDefault();
    lsm303.setTimeout(1);

    // Enable temperature sensor
    byte ctrl5 = lsm303.readReg(0x24);
    ctrl5 |= 0x80;

    // Set the mag data rate to 50Hz
    ctrl5 &= ~0b00011100;
    ctrl5 |=  0b00010000;

    lsm303.writeReg(0x24, ctrl5);

    // Set the acc data rate to 50Hz
    byte ctrl1 = lsm303.readReg(0x20);
    ctrl1 &= ~0b11110000;
    ctrl1 |=  0b01010000;

    // Enable BDU - Synchronized conversion. 
    ctrl1 |= 0b00001000;

    lsm303.writeReg(0x20, ctrl1);
  }
}

int iicScan() {
  byte numberOfDevices = 0;

  for(byte address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (!error) {
      numberOfDevices++;
    }
  }

  return numberOfDevices;
}

void loop()
{
  if (iicScan() > 0) {
    lsm303.read();
    LSM303::vector<int16_t> acc = lsm303.a;
    LSM303::vector<int16_t> mag = lsm303.m;

    byte temp_lo = lsm303.readReg(0x05);
    byte temp_hi = lsm303.readReg(0x06);
    
    imu_data[0] = mag.x;
    imu_data[1] = mag.y;
    imu_data[2] = mag.z;
    imu_data[3] = acc.x;
    imu_data[4] = acc.y;
    imu_data[5] = acc.z;
    imu_data[6] = temp_lo;
    imu_data[7] = temp_hi;
    imu_pub.publish(&imu_msg);
  }

  nh.spinOnce();    
}
