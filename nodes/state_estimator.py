#!/usr/bin/env python
import rospy
import string
import math
import sys
import random
import numpy

from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

def imuCallback(data):
    global imuMsg
    imuMsg = data

def depthCallback(data):
    global depth
    depth = data

depth = PoseWithCovarianceStamped()
imuMsg = Imu()

rospy.init_node("state_estimator")
#We only care about the most recent measurement, i.e. queue_size=1
rospy.Subscriber('imu', Imu, imuCallback)
rospy.Subscriber('depth', PoseWithCovarianceStamped, depthCallback)

pub = rospy.Publisher('odom', Odometry, queue_size=1)

rate = rospy.Rate(30) # 30hz

rospy.loginfo("Starting up simple state estimator...")

odomMsg = Odometry()
seq = 0
while not rospy.is_shutdown():

    odomMsg.pose.pose.position = depth.pose.pose.position
    odomMsg.pose.pose.orientation = imuMsg.orientation
    
    odomMsg.header.frame_id = 'base_imu_link'
    odomMsg.header.stamp = rospy.get_rostime()
    odomMsg.header.seq = seq
    pub.publish(odomMsg) 

    seq = seq + 1
   
    try:
        rate.sleep()
    except rospy.ROSInterruptException: pass
