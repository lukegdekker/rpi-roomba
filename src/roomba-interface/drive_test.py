#!/usr/bin/env python3
import time
from pyroombaadapter import PyRoombaAdapter
import numpy as np
import rospy

PORT = "/dev/ttyS0"

def cmdCallback(cmd):
    print(cmd)

def init():
    cmdSub = rospy.Subscriber('/joy', Joy, cmdCallback)
    rospy.init_node('roomba_interface')
    rospy.spin()

if __name__ == '__main__':
    init()
            

    