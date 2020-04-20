#!/usr/bin/env python3
import time
from pyroombaadapter import PyRoombaAdapter
import numpy as np
import rospy
from sensor_msgs.msg import Joy

PORT = "/dev/ttyS0"

class RoombaDrive:
    def __init__(self):
        #self.roomba = PyRoombaAdapter(PORT)
        self.cmdSub = rospy.Subscriber("/joy", Joy, self.getCmd)

    def getCmd(self, cmd):
        print(cmd)

if __name__ == '__main__':
    rospy.init_node("roomba_interface")
    interface = RoombaDrive()
    while True:
        try:
            rospy.spin()
        except KeyboardInterrupt:
            exit()
            
            

    
