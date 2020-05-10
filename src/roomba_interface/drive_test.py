#!/usr/bin/env python3
import time
from pyroombaadapter import PyRoombaAdapter
import numpy as np
import rospy
from sensor_msgs.msg import Joy

PORT = "/dev/serial0"

class RoombaDrive:
    def __init__(self):
        self.roomba = PyRoombaAdapter(PORT)
        self.cmdSub = rospy.Subscriber("/joy", Joy, self.getCmd)
        self.roomba.change_mode_to_full()
        self.clean = False

    def getCmd(self, cmd):
        self.drive = cmd.axes[1]
        self.steer = cmd.axes[3]
        self.cleanEnable = cmd.buttons[0]

        if self.cleanEnable:
            if not self.clean:
                self.clean = True
            else:
                self.clean = False

        if self.clean:
            self.roomba.send_moters_cmd(True, True, True, True, True)
        else:
            self.roomba.send_moters_cmd(False, True, False, True, False)

        self.roomba.move(self.drive, self.steer)

if __name__ == '__main__':
    rospy.init_node("roomba_interface")
    interface = RoombaDrive()
    while True:
        try:
            rospy.spin()
        except KeyboardInterrupt:
            exit()
            
            

    
