#!/usr/bin/env python
import rospy
import serial
import string
import math
import sys
import random
import numpy

from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float32

def callback(data):
    global pressure
    pressure = data.data
    
pressure = 0
z = 0.0   
pressureOffset = 270.0
pressureScale = -1.0/18

rospy.init_node("pressure_to_depth")
#We only care about the most recent measurement, i.e. queue_size=1
sub = rospy.Subscriber('pressure', Float32, callback)
pub = rospy.Publisher('depth', PoseWithCovarianceStamped, queue_size=1)

rate = rospy.Rate(30) # 30hz

poseMsg = PoseWithCovarianceStamped()

poseMsg.pose.covariance[0] = 0.025
poseMsg.pose.covariance[7] = 0.025
poseMsg.pose.covariance[14] = 0.025
poseMsg.pose.covariance[21] = 0.025
poseMsg.pose.covariance[28] = 0.025
poseMsg.pose.covariance[35] = 0.025

rospy.loginfo("Starting up pressure to depth conversion...")

seq = 0
while not rospy.is_shutdown():
    z = pressureScale*(pressure - pressureOffset)

    poseMsg.pose.pose.position.x = 0
    poseMsg.pose.pose.position.y = 0
    poseMsg.pose.pose.position.z = z
    poseMsg.header.frame_id = 'odom'
    poseMsg.header.stamp = rospy.get_rostime()
    poseMsg.header.seq = seq
    seq = seq + 1
    pub.publish(poseMsg) 

    rate.sleep()
        
