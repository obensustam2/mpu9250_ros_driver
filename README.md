## Resources
- **[Sensor Details](https://learn.sparkfun.com/tutorials/mpu-9250-hookup-guide/all)**

- **[Sensor Arduino Library](https://robojax.com/learn/arduino/?vid=robojax-MPU9250)**

- **[ROS Visualization](https://www.youtube.com/watch?v=a-mfCeykmYw)**

## Pin Configuration
```bash
MPU9250    -    Arduino Mega

SCL             21
SDA             20
VDD             3.3V
GND             GND
```

## ROS Installations
```bash
sudo apt-get install ros-<ros_version>-rosserial-arduino
sudo apt-get install ros-<ros_version>-rosserial
sudo apt-get install ros-<ros_version>-imu-complementary-filter*
```

## Arduino Libraries
- Install ros_lib library to your IDE library directory
```bash
cd <sketchbook>/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
```
- Copy MPU9250-master library to your IDE library directory


## Uploading Code to Arduino
If you get "Permission Denied" error during the uploading process of the code at mpu_9250_v22 directory:

Check your USB connection
```bash
ls /dev/tty*
```

After you find the name of your connection, make this port executable.
```bash
sudo chmod a+rw /dev/tty<port_name>
```

## Arduino Static Port Assignmet 
```bash
udevadm info --name=/dev/tty<port_name> --attribute-walk
cd /etc/udev/rules.d
sudo touch 99-usb-serial.rules
sudo nano 99-usb-serial.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0042", SYMLINK+="a_mega"
sudo udevadm trigger
ls -l /dev/<device_name>
sudo usermod -a -G tty $USER
sudo usermod -a -G dialout $USER
sudo reboot
```


## Code Execution
### arduino.launch
- arduino node that publishes "/imu/data_raw" message that doesn't have orientation data.
```bash
roslaunch mpu9250_ros_driver arduino.launch
```

### imu_demo.launch
- complementary_filter_node that subscribes "/imu/data_raw" and publishes "imu/data" message that has orientation data.
- robot_state_publisher node that publishes robot model.
- enable rpy_tf node that updates robot orientation by using upcoming "imu/data".
- enable RViZ.
```bash
roslaunch mpu9250_ros_driver imu_demo.launch
```
