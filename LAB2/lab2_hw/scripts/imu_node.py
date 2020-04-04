#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import serial
import utm
import numpy as np
from lab2_hw.msg import gps_data
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
# from squaternion import euler2quat, quat2euler, Quaternion
from tf.transformations import quaternion_from_euler


# def deg2rad(deg):
#     pi_on_180 = 0.017453292519943295
#     return deg * pi_on_180


if __name__ == "__main__":
    # SENSOR_NAME = "imu"

    rospy.init_node("imu_node")
    serial_port = rospy.get_param("~port", "/dev/ttyUSB0")
    serial_baud = rospy.get_param("~baudrate", 115200)
    sampling_rate = rospy.get_param("~sampling_rate", 5.0)

    port = serial.Serial(serial_port, serial_baud, timeout=3.0)
    rospy.logdebug(
        "Using IMU sensor on port " + serial_port + " at " + str(serial_baud)
    )
    # rospy.sleep(0.2)
    line = port.readline()

    imu_pub = rospy.Publisher("imu", Imu, queue_size=5)
    mag_pub = rospy.Publisher("magnet", MagneticField, queue_size=5)
    rospy.logdebug("Initialization complete")
    rospy.loginfo("Publishing IMU data.")

    imu_msg = Imu()
    mag_msg = MagneticField()
    # sleep_time = 1 / sampling_rate - 0.025

    try:
        while not rospy.is_shutdown():
            line = port.readline()
            # line = line.decode()
            if line == "":
                rospy.logwarn("IMU: No data")
            else:
                data = np.array(line.split(","))
                if data[0] == "$VNYMR":
                    imu_msg.header.stamp = rospy.Time.now()
                    mag_msg.header.stamp = rospy.Time.now()

                    imu_msg.header.frame_id = data[0]
                    mag_msg.header.frame_id = data[0]
                    try:
                        yaw = float(data[1])
                        pitch = float(data[2])
                        roll = float(data[3])
                        yaw = float(np.deg2rad(yaw))
                        pitch = float(np.deg2rad(pitch))
                        roll = float(np.deg2rad(roll))
                        mag_x = float(data[4])
                        mag_y = float(data[5])
                        mag_z = float(data[6])
                        acc_x = float(data[7])
                        acc_y = float(data[8])
                        acc_z = float(data[9])
                        gyr_x = float(data[10])
                        gyr_y = float(data[11])
                        data_1 = np.array(data[12].split("*"))
                        gyr_z = float(data_1[0])
                        quaternion = quaternion_from_euler(
                            roll, pitch, yaw)
                        # quaternion = euler2quat(yaw, pitch, roll, degree=True)
                        imu_msg.orientation.w = quaternion[0]
                        imu_msg.orientation.x = quaternion[1]
                        imu_msg.orientation.y = quaternion[2]
                        imu_msg.orientation.z = quaternion[3]
                        imu_msg.angular_velocity.x = gyr_x
                        imu_msg.angular_velocity.y = gyr_y
                        imu_msg.angular_velocity.z = gyr_z
                        imu_msg.linear_acceleration.x = acc_x
                        imu_msg.linear_acceleration.y = acc_y
                        imu_msg.linear_acceleration.z = acc_z

                        mag_msg.magnetic_field.x = mag_x
                        mag_msg.magnetic_field.y = mag_y
                        mag_msg.magnetic_field.z = mag_z

                        imu_pub.publish(imu_msg)
                        mag_pub.publish(mag_msg)

                    except:
                        rospy.logwarn("Data exception: " + line)
                        continue
            # rospy.sleep(sleep_time)

    except rospy.ROSInterruptException:
        port.close()

    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down gps node...")
