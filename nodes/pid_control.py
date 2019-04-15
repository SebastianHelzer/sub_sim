#!/usr/bin/env python
import rospy
import serial
import string
import math
import sys
import random

#from time import time
from simple_pid import PID
from geometry_msgs.msg import Accel
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from dynamic_reconfigure.server import Server
from fake_imu.cfg import pidConfig

degrees2rad = math.pi/180.0

state = None

D_d = 0
D_kp = 0
D_ki = 0
D_kd = 0

P_d = 0
P_kp = 0
P_ki = 0
P_kd = 0

R_d = 0
R_kp = 0
R_ki = 0
R_kd = 0

pitch = 0
roll = 0
depth = 0

pidD = PID(0, 0, 0, setpoint=0)
pidP = PID(0, 0, 0, setpoint=0)
pidR = PID(0, 0, 0, setpoint=0)


def getDegBounds(value):
    while(value > 180): 
        value = value - 360
    while(value < -180):
        value = value + 360
    return value

# Callback for dynamic reconfigure requests
def reconfig_callback(config, level):
    global D_d,D_kp,D_ki,D_kd,P_d,P_kp,P_ki,P_kd,R_d,R_kp,R_ki,R_kd
    global pidD,pidP,pidR
    
    D_d = config['D_d']
    D_kp = config['D_kp']
    D_ki = config['D_ki']
    D_kd = config['D_kd']

    P_d = config['P_d']
    P_kp = config['P_kp']
    P_ki = config['P_ki']
    P_kd = config['P_kd']

    R_d = config['R_d']
    R_kp = config['R_kp']
    R_ki = config['R_ki']
    R_kd = config['R_kd']

    pidD = PID(D_kp, D_ki, D_kd, setpoint=D_d)
    pidR = PID(R_kp, R_ki, R_kd, setpoint=R_d)
    pidP = PID(P_kp, P_ki, P_kd, setpoint=P_d)
    return config

def odomCallback(msg):
    global pitch,roll,depth
    depth = msg.pose.pose.position.z
    (roll,pitch,yaw) = tf.transformations.euler_from_quaternion(
        [data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])
    
rospy.init_node("pid_control_node")
pub = rospy.Publisher('cmd_accel', Accel, queue_size=1)
rospy.Subscriber('odom', Odometry, odomCallback)
srv = Server(pidConfig, reconfig_callback)  # define dynamic_reconfigure callback

rate = rospy.Rate(30) # 30hz
cmdMsg = Accel()

while not rospy.is_shutdown():
    cP = pidP(pitch)
    cR = pidR(roll)
    cD = pidD(depth)

    cmdMsg.linear.z = cD
    cmdMsg.angular.x = cR
    cmdMsg.angular.y = cP

    pub.publish(cmdMsg)

    rate.sleep()
        
