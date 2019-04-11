#!/usr/bin/env python
import rospy
import serial
import string
import math
import sys
import random
import numpy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Accel

def stateCallback(data):
    global x,y,z,q1,q2,q3,q4
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    z = data.pose.pose.position.z

    q1 = data.pose.pose.orientation.x
    q2 = data.pose.pose.orientation.y
    q3 = data.pose.pose.orientation.z
    q4 = data.pose.pose.orientation.w
    
def inputCallback(data): 
    global a,b,c,l,m,n
    
    rospy.loginfo("I heard %s",data.linear.x)
    a = data.linear.x
    b = data.linear.y
    c = data.linear.z

    l = data.linear.x
    m = data.linear.y
    n = data.linear.z

def calcKinematics(x,u):
    pn    = x(1)
    pe    = x(2)
    pe    = x(3)
    u     = x(4)
    v     = x(5)
    w     = x(6)
    phi   = x(7)
    theta = x(8)
    psi   = x(9)
    p     = x(10)
    q     = x(11)
    r     = x(12)
    fx    = uu(1)
    fy    = uu(2)
    fz    = uu(3)
    ell   = uu(4)
    m     = uu(5)
    n     = uu(6)

    gamma_1 = (Jy - Jz)/Jx
    gamma_2 = (Jz - Jx)/Jy
    gamma_3 = (Jx - Jy)/Jz
    
    rvb = v2b( [psi, theta, phi] )
    rbv = numpy.transpose(rvb)
    
    cph = cos(phi)
    sph = sin(phi)
    cth = cos(theta)
    tth = tan(theta)
    A_euler = [1, sph*tth,  cph*tth,
               0,     cph,     -sph,
               0, sph/cth,  cph/cth].reshape((3, 3))
           
    #Compute state derivative
    posdot_ned = rbv * numpy.transpose([u, v, w])
    veldot_b = numpy.transpose([r*v - q*w, p*w - r*u, q*u - p*v] + [a,b,c])
    attdot = A_euler * numpy.transpose([p,q,r])
    attratedot = numpy.transpose( [gamma_1*q*r,gamma_2*p*r,gamma_3*p*q]
                  + [1/Jx*ell,1/Jy*m,1/Jz*n])

    return [posdot_ned,veldot_b,attdot,attratedot]

rospy.init_node("control_model_node")
#We only care about the most recent measurement, i.e. queue_size=1
pub = rospy.Publisher('model/odom', Odometry, queue_size=1)
rospy.Subscriber("state", Odometry, stateCallback)
rospy.Subscriber("input", Accel, inputCallback)

rate = rospy.Rate(30) # 60hz

odomMsg = Odometry()

odomMsg.pose.covariance[0] = 0.025
odomMsg.pose.covariance[7] = 0.025
odomMsg.pose.covariance[14] = 0.025
odomMsg.pose.covariance[21] = 0.025
odomMsg.pose.covariance[28] = 0.025
odomMsg.pose.covariance[35] = 0.025

odomMsg.twist.covariance[0] = 0.025
odomMsg.twist.covariance[7] = 0.025
odomMsg.twist.covariance[14] = 0.025
odomMsg.twist.covariance[21] = 0.025
odomMsg.twist.covariance[28] = 0.025
odomMsg.twist.covariance[35] = 0.025

rospy.loginfo("Starting up fake robot model...")

mu = 0
sigma = .001

Jx = 1
Jy = 1
Jz = 1
mass = 5

# Position - State
x = 0 
y = 0
z = 0
# Velocity - State
u = 0
v = 0
w = 0
# Orientaion - State
q1 = 0 
q2 = 0 
q3 = 0 
q4 = 0
# Angular Rate - State
p = 0
q = 0
r = 0
# Accel Linear - Input
a = 0
b = 0
c = 0
# Accel Angular - Input
l = 0
m = 0
n = 0

seq = 0
while not rospy.is_shutdown():

    # [posdot_ned,veldot_b,attdot,attratedot] = calcKinematics(state,accel)

    odomMsg.pose.pose.orientation.x = q1
    odomMsg.pose.pose.orientation.y = q2
    odomMsg.pose.pose.orientation.z = q3
    odomMsg.pose.pose.orientation.w = q4

    odomMsg.pose.pose.position.x = x
    odomMsg.pose.pose.position.y = y
    odomMsg.pose.pose.position.z = z

    odomMsg.twist.twist.linear.x = u
    odomMsg.twist.twist.linear.y = v
    odomMsg.twist.twist.linear.z = w
    
    odomMsg.twist.twist.angular.x = p
    odomMsg.twist.twist.angular.y = q
    odomMsg.twist.twist.angular.z = r
    
    odomMsg.header.frame_id = 'odom'
    odomMsg.header.stamp = rospy.get_rostime()
    odomMsg.header.seq = seq
    seq = seq + 1
    pub.publish(odomMsg) 

    rate.sleep()
        
