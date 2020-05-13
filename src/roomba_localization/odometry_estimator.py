#!/usr/bin/env python3
import serial
import time
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64
from nav_msgs.msg import Odometry
import math
from squaternion import Quaternion, euler2quat

WHEEL_BASE = 0.235 #[m]
M_PER_TICK = 0.000437

class odometryEstimator:
    def __init__(self):
        # Initialize ROS pub/subs
        self.leftTicksSub = rospy.Subscriber("/left_ticks", Int64, self.getLeftTicks)
        self.rightTicksSub = rospy.Subscriber("/right_ticks", Int64, self.getRightTicks)
        self.odomPub = rospy.Publisher("/odom", Odometry, queue_size=1)
        self.prevLeftTicks = 0
        self.prevRightTicks = 0
        self.leftTicks = 0
        self.rightTicks = 0
        self.x = 0.0
        self.y = 0.0 
        self.theta = 0.0

    def getLeftTicks(self, msg):
        self.leftTicks = msg.data
        
    def getRightTicks(self, msg):
        self.rightTicks = msg.data

    def computeSpeed(self):
        self.speedLeft = (1.0/0.05) * (self.leftTicks - self.prevLeftTicks) * M_PER_TICK
        self.speedRight = (1.0/0.05) * (self.rightTicks - self.prevRightTicks) * M_PER_TICK
        self.prevLeftTicks = self.leftTicks
        self.prevRightTicks = self.rightTicks

    def computeOdom(self):
        while True:
            self.computeSpeed()
            xdot = (1/2)*math.cos(self.theta)*self.speedRight + (1/2)*math.cos(self.theta)*self.speedLeft
            ydot = (1/2)*math.sin(self.theta)*self.speedRight + (1/2)*math.sin(self.theta)*self.speedLeft
            thetadot = (1.0/WHEEL_BASE)*self.speedRight - (1.0/WHEEL_BASE)*self.speedLeft
            self.x = self.x + 0.05*xdot
            self.y = self.y + 0.05*ydot
            self.theta = self.theta + 0.05*thetadot
            
            q = euler2quat(0.0, 0.0, self.theta)
            odom = Odometry()
            h = Header()
            h.stamp = rospy.Time.now()
            h.frame_id = 'base_link'
            odom.header = h
            odom.child_frame_id = 'base_link'
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.orientation.x = q[1]
            odom.pose.pose.orientation.y = q[2]
            odom.pose.pose.orientation.z = q[3]
            odom.pose.pose.orientation.w = q[0]

            self.odomPub.publish(odom)
            time.sleep(0.05)

if __name__ == '__main__':
    rospy.init_node("odomEstimator")
    odomEst = odometryEstimator()
    odomEst.computeOdom()
    while True:
        try:
            rospy.spin()
        except KeyboardInterrupt:
            exit()
