#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import serial
import utm
import numpy as np
from lab3_hw.msg import gps_data


def latlon_to_utm(lat, lon):
    # lat_deg = int(lat)+float(((lat*100) % 100)/60)
    # lon_deg = int(lon)+float(((lon % 100)/60)
    utm_data = utm.from_latlon(lat, lon)
    return utm_data


if __name__ == "__main__":
    SENSOR_NAME = "gps"
    rospy.init_node("gps_node")
    serial_port = rospy.get_param("~port", "/dev/ttyACM0")
    serial_baud = rospy.get_param("~baudrate", 115200)
    sampling_rate = rospy.get_param("~sampling_rate", 5.0)

    port = serial.Serial(serial_port, serial_baud, timeout=3.0)
    rospy.logdebug(
        "Using GPS sensor on port " + serial_port + " at " + str(serial_baud)
    )
    # rospy.logdebug("Initializing sensor with *0100P4\\r\\n ...")

    # sampling_count = int(round(1/(sampling_rate*0.007913)))
    # port.write('*0100EW*0100PR='+str(sampling_count)+'\r\n')  # cmd from 01 to 00 to set sampling period
    # rospy.sleep(0.2)
    line = port.readline()

    gps_pub = rospy.Publisher(SENSOR_NAME, gps_data, queue_size=20)
    rospy.logdebug("Initialization complete")
    rospy.loginfo("Publishing gps data.")

    gps_msg = gps_data()

    # sleep_time = 1 / sampling_rate - 0.025

    try:
        while not rospy.is_shutdown():
            line = port.readline()
            line = line.decode()
            # print(type(line))
            if line == "":
                rospy.logwarn("GPS: No data")
            else:
                data = np.array(line.split(","))
                # if line.startswith("$GPGGA"):
                # if line[0] == "$GPGGA":
                if data[0] == "$GNGGA":
                    gps_msg.header.stamp = rospy.Time.now()
                    # print(data[0])
                    try:
                        gps_msg.latitude = float(data[2])
                        gps_msg.longitude = float(data[4])
                        gps_msg.latitude = int(gps_msg.latitude / 100) + float((gps_msg.latitude % 100) / 60)
                        gps_msg.longitude = -(int(gps_msg.longitude / 100) + float((gps_msg.longitude % 100) / 60))
                        gps_msg.fix_style = float(data[6])  # to be fill up
                        gps_msg.altitude = float(data[9])
                        # print(data[9])
                        utm_data = latlon_to_utm(gps_msg.latitude, gps_msg.longitude)
                        gps_msg.utm_easting = float(utm_data[0])
                        gps_msg.utm_northing = float(utm_data[1])
                        gps_msg.Zone = utm_data[2]
                        gps_msg.fields = utm_data[3]

                        gps_pub.publish(gps_msg)

                    except:
                        rospy.logwarn("Data exception: " + line)
                        continue
            # rospy.sleep(sleep_time)
            # rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        port.close()

    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down gps node...")
