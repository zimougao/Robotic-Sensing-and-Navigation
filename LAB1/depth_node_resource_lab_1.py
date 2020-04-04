#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
from math import sin, pi
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry

def paro_to_depth(pressure, latitude):
    '''
    Given pressure (in m fresh) and latitude (in radians) returns ocean
    depth (in m.).  Uses the formula discovered and presented by Leroy and
    Parthiot in: Claude C. Leroy and Francois Parthiot, 'Depth-pressure
    relationships in the oceans and seas', J. Acoustic Society of America,
    March 1998, p1346-.
    '''
    # Convert the input m/fw into MPa, as the equation expects MPa.
    pressure = pressure * 0.0098066493
    # Gravity at Latitude.
    g = 9.780318 * (1 + 5.2788e-3*sin(latitude)**2 -
                        2.36e-5*sin(latitude)**4)
    # Now calculate the 'standard ocean' depth.
    Zs_num = (9.72659e2*pressure - 2.512e-1*pressure**2 +
              2.279e-4*pressure**3 - 1.82e-7*pressure**4)
    Zs_den = g + 1.092e-4*pressure
    return Zs_num / Zs_den

if __name__ == '__main__':
    SENSOR_NAME = "paro"
    rospy.init_node('depth_paro')
    serial_port = rospy.get_param('~port','/dev/ttyS1')
    serial_baud = rospy.get_param('~baudrate',9600)
    sampling_rate = rospy.get_param('~sampling_rate',5.0)
    offset = rospy.get_param('~atm_offset',12.121) # in meter ??
    latitude_deg = rospy.get_param('~latitude',41.526)  # deg 41.526 N is Woods Hole
  
    port = serial.Serial(serial_port, serial_baud, timeout=3.)
    rospy.logdebug("Using depth sensor on port "+serial_port+" at "+str(serial_baud))
    rospy.logdebug("Using latitude = "+str(latitude_deg)+" & atmosphere offset = "+str(offset))
    rospy.logdebug("Initializing sensor with *0100P4\\r\\n ...")
    
    sampling_count = int(round(1/(sampling_rate*0.007913)))
    port.write('*0100EW*0100PR='+str(sampling_count)+'\r\n')  # cmd from 01 to 00 to set sampling period
    rospy.sleep(0.2)        
    line = port.readline()
    port.write('*0100P4\r\n')  # cmd from 01 to 00 to sample continuously
    
    latitude = latitude_deg * pi / 180.
    depth_pub = rospy.Publisher(SENSOR_NAME+'/depth', Float64, queue_size=5)
    pressure_pub = rospy.Publisher(SENSOR_NAME+'/pressure', Float64, queue_size=5)
    odom_pub = rospy.Publisher(SENSOR_NAME+'/odom',Odometry, queue_size=5)
    
    rospy.logdebug("Initialization complete")
    
    rospy.loginfo("Publishing pressure and depth.")
    
    odom_msg = Odometry()
    odom_msg.header.frame_id = "odom"
    odom_msg.child_frame_id = SENSOR_NAME
    odom_msg.header.seq=0
    
    sleep_time = 1/sampling_rate - 0.025
    
    try:
        while not rospy.is_shutdown():
            line = port.readline()
            #print line
            if line == '':
                rospy.logwarn("DEPTH: No data")
            else:
                if line.startswith('*0001'):
                    odom_msg.header.stamp=rospy.Time.now()                
                    try: pressure = float(line[5:].strip())
                    except: 
                        rospy.logwarn("Data exception: "+line)
                        continue
                pressure_pub.publish(pressure)
                depth_mes = paro_to_depth(pressure - offset, latitude_deg)
                depth_pub.publish(depth_mes)
                odom_msg.pose.pose.position.z = -depth_mes
                odom_msg.header.seq+=0
                odom_pub.publish(odom_msg)
            rospy.sleep(sleep_time)
            
    except rospy.ROSInterruptException:
        port.close()
    
    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down paro_depth node...")

