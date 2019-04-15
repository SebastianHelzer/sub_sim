#!/usr/bin/env python
import rospy
import tf
import string
import math
import sys
import random
import numpy

from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry

def callback(data):
    global r,p,y
    (r,p,y) = tf.transformations.euler_from_quaternion(
        [data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])
    
r = 0
p = 0
y = 0

rospy.init_node("euler_from_quat")
#We only care about the most recent measurement, i.e. queue_size=1
sub = rospy.Subscriber('odom', Odometry, callback)
pub = rospy.Publisher('euler', Vector3, queue_size=1)

rate = rospy.Rate(30) # 30hz

rospy.loginfo("Starting up pressure to depth conversion...")

seq = 0
while not rospy.is_shutdown():
    eulerMsg = Vector3()
    eulerMsg.x = r * 180.0/3.14159
    eulerMsg.y = p * 180.0/3.14159
    eulerMsg.z = y * 180.0/3.14159

    pub.publish(eulerMsg) 

    rate.sleep()
        
