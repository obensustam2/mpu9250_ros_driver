## Pin Configuration
```bash
MPU9250    -    Arduino Mega

SCL             21
SDA             20
VDD             3.3V
GND             GND
```

## Code Uploading to Arduino
If you get "Permission Denied" error during the uploading process of the code at mpu_9250_v22 directory:

Check your USB connection
```bash
ls /dev/tty*
```

After you find the name of your connection, make this port executable.
```bash
sudo chmod a+rw /dev/ttyXXXX
```

## arduino.launch
To enable arduino node that publishes "/imu/data_raw" message that doesn't have orientation data.
```bash
roslaunch imu_ros_arduino arduino.launch
```

## imu_demo.launch
To enable complementary_filter_node that subscribes "/imu/data_raw" and publishes "imu/data" message that has orientation data.

To enable robot_state_publisher node that publishes robot model.

To enable rpy_tf node that updates robot orientation by using upcoming "imu/data".

To enable RViZ.
```bash
roslaunch imu_ros_arduino imu_demo.launch
```