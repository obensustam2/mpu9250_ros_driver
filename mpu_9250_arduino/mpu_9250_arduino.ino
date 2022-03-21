#include "MPU9250.h"
#include <ros.h>
#include <sensor_msgs/Imu.h>

ros::NodeHandle  nh;
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("/imu/data_raw", &imu_msg);
long sequence = 0;

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);
int status;

void setup() {
  // serial to display data
  Serial.begin(57600);
  while(!Serial) {}

  nh.initNode();
  nh.advertise(imu_pub);

  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
}

void loop() {
  // read the sensor
  IMU.readSensor();
  // display the data
/*
  Serial.print("AccelX: ");
  Serial.print(IMU.getAccelX_mss(),6);
  Serial.print("\t");
  Serial.print("AccelY: ");  
  Serial.print(IMU.getAccelY_mss(),6);
  Serial.print("\t");
  Serial.print("AccelZ: ");  
  Serial.println(IMU.getAccelZ_mss(),6);
  
  Serial.print("GyroX: ");
  Serial.print(IMU.getGyroX_rads(),6);
  Serial.print("\t");
  Serial.print("GyroY: ");  
  Serial.print(IMU.getGyroY_rads(),6);
  Serial.print("\t");
  Serial.print("GyroZ: ");  
  Serial.println(IMU.getGyroZ_rads(),6);

  Serial.print("MagX: ");  
  Serial.print(IMU.getMagX_uT(),6);
  Serial.print("\t");  
  Serial.print("MagY: ");
  Serial.print(IMU.getMagY_uT(),6);
  Serial.print("\t");
  Serial.print("MagZ: ");  
  Serial.println(IMU.getMagZ_uT(),6);
*/
  //Header
  imu_msg.header.seq = sequence++;
  imu_msg.header.stamp = nh.now();
  imu_msg.header.frame_id  ="?";

  //Linear Acceleration 
  imu_msg.linear_acceleration.x = IMU.getAccelX_mss() ;
  imu_msg.linear_acceleration.y = IMU.getAccelY_mss() ;
  imu_msg.linear_acceleration.z = IMU.getAccelZ_mss() ;
  
  //Angular Velocity
  imu_msg.angular_velocity.x = IMU.getGyroX_rads() ;
  imu_msg.angular_velocity.y = IMU.getGyroY_rads() ; 
  imu_msg.angular_velocity.z = IMU.getGyroZ_rads() ;  

  //Orientation
  imu_msg.orientation.x = 0 ;
  imu_msg.orientation.y = 0 ;
  imu_msg.orientation.z = 0 ;
  imu_msg.orientation.w = 0 ;

  imu_pub.publish(&imu_msg);
  nh.spinOnce();
  
  delay(10);
} 
