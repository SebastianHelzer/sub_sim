#!/usr/bin/env python
import rospy
import tf
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
rad2degress = 180.0/math.pi

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

Y_d = 0
Y_kp = 0
Y_ki = 0
Y_kd = 0

Vx_d = 0
Vx_kp = 0
Vx_ki = 0
Vx_kd = 0

pitch = 0
roll = 0
depth = 0
yaw = 0
v_x = 0
v_y = 0
v_z = 0

pidD = PID(0, 0, 0, setpoint=0)
pidP = PID(0, 0, 0, setpoint=0)
pidR = PID(0, 0, 0, setpoint=0)
pidVx = PID(0, 0, 0, setpoint=0)
pidY = PID(0, 0, 0, setpoint=0)


def getDegBounds(value):
    while(value > 180): 
        value = value - 360
    while(value < -180):
        value = value + 360
    return value

# Callback for dynamic reconfigure requests
def reconfig_callback(config, level):
    global D_d,D_kp,D_ki,D_kd,P_d,P_kp,P_ki,P_kd,R_d,R_kp,R_ki,R_kd
    global pidD,pidP,pidR,pidVx,pidY
    
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

    Y_d = config['Y_d']
    Y_kp = config['Y_kp']
    Y_ki = config['Y_ki']
    Y_kd = config['Y_kd']
    
    Vx_d = config['Vx_d']
    Vx_kp = config['Vx_kp']
    Vx_ki = config['Vx_ki']
    Vx_kd = config['Vx_kd']

    pidD = PID(D_kp, D_ki, D_kd, setpoint=D_d)
    pidR = PID(R_kp, R_ki, R_kd, setpoint=R_d)
    pidP = PID(P_kp, P_ki, P_kd, setpoint=P_d)
    pidVx = PID(Vx_kp, Vx_ki, Vx_kd, setpoint=Vx_d)
    pidY = PID(Y_kp, Y_ki, Y_kd, setpoint=Y_d)
    return config

def mod180(a,n):
    return (a - math.floor(a/n) * n)
def getBoundedAngleError(angle):
    return mod180(angle + 180,360) - 180

def odomCallback(data):
    global pitch,roll,depth,yaw,v_x,v_y,v_z
    depth = data.pose.pose.position.z
    v_x = data.twist.twist.linear.x
    v_y = data.twist.twist.linear.y
    v_z = data.twist.twist.linear.z
    (r,p,y) = tf.transformations.euler_from_quaternion(
        [data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])
    roll =  getBoundedAngleError(r * rad2degress) 
    pitch = getBoundedAngleError(p * rad2degress)
    yaw =   getBoundedAngleError(y * rad2degress)


rospy.init_node("pid_control_node")
pub = rospy.Publisher('cmd_accel', Accel, queue_size=1)
rospy.Subscriber('odom', Odometry, odomCallback)
srv = Server(pidConfig, reconfig_callback)  # define dynamic_reconfigure callback

rate = rospy.Rate(30) # 30hz
cmdMsg = Accel()

rospy.loginfo("Starting up pid controller...")

while not rospy.is_shutdown():
    cP = pidP(pitch)
    cR = pidR(roll)
    cD = pidD(depth)
    cV_x = pidVx(v_x)
    cY = pidY(yaw)

    cmdMsg.linear.x = cV_x
    cmdMsg.linear.z = cD
    cmdMsg.angular.x = cR
    cmdMsg.angular.y = cP
    cmdMsg.angular.z = cY

    pub.publish(cmdMsg)

    try:
        rate.sleep()
    except rospy.ROSInterruptException: pass
        
