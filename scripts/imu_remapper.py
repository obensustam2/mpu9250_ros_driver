#!/usr/bin/env python
import rospy
from std_msgs.msg import Header, Float64
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion

new_imu = Imu()

def imu_callback(data):
    global new_imu
    
    new_imu.orientation_covariance = data.orientation_covariance
    new_imu.angular_velocity_covariance = data.angular_velocity_covariance
    new_imu.linear_acceleration_covariance = data.linear_acceleration_covariance

    new_imu.header.frame_id = 'new_camera_imu_optical_frame'
    new_imu.header.stamp = rospy.Time.now()

    new_imu.orientation.x = data.orientation.x
    new_imu.orientation.y = data.orientation.y
    new_imu.orientation.z = data.orientation.z
    new_imu.orientation.w = data.orientation.w

    new_imu.angular_velocity.x = data.angular_velocity.x
    new_imu.angular_velocity.y = data.angular_velocity.y
    new_imu.angular_velocity.z = data.angular_velocity.z

    new_imu.linear_acceleration.x = data.linear_acceleration.x
    new_imu.linear_acceleration.y = data.linear_acceleration.y
    new_imu.linear_acceleration.z = data.linear_acceleration.z


def talker():
    rospy.init_node('imu_converter', anonymous=True)
    rospy.Subscriber('/imu/data', Imu, imu_callback)
    pub_new_imu = rospy.Publisher('/imu/data_new', Imu, queue_size=10)
    rate = rospy.Rate(100) 

    while not rospy.is_shutdown():
        pub_new_imu.publish(new_imu)
        rospy.loginfo(new_imu)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
