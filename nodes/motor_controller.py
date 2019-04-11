#!/usr/bin/env python
import rospy
import serial
import string
import math
import sys
import random
import numpy

from geometry_msgs.msg import PoseWithCovarianceStamped

rospy.init_node("motor_controller_node")
#We only care about the most recent measurement, i.e. queue_size=1
pub = rospy.Publisher('depth', PoseWithCovarianceStamped, queue_size=1)

rate = rospy.Rate(60) # 60hz

poseMsg = PoseWithCovarianceStamped()

poseMsg.pose.covariance[0] = 0.025
poseMsg.pose.covariance[7] = 0.025
poseMsg.pose.covariance[14] = 0.025
poseMsg.pose.covariance[21] = 0.025
poseMsg.pose.covariance[28] = 0.025
poseMsg.pose.covariance[35] = 0.025

rospy.loginfo("Starting up fake Depth Sensor...")

mu = 0
sigma = .001

x = 0
y = 0
z = -5

seq = 0
while not rospy.is_shutdown():
    noiseX = random.normalvariate(mu, sigma)
    noiseY = random.normalvariate(mu, sigma)
    noiseZ = random.normalvariate(mu, sigma)
    
    x = x + noiseX
    y = y + noiseZ
    z = z + noiseZ

    poseMsg.pose.pose.position.x = x
    poseMsg.pose.pose.position.y = y
    poseMsg.pose.pose.position.z = z
    poseMsg.header.frame_id = 'odom'
    poseMsg.header.stamp = rospy.get_rostime()
    poseMsg.header.seq = seq
    seq = seq + 1
    pub.publish(poseMsg) 

    rate.sleep()
        
