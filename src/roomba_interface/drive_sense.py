#!/usr/bin/env python3
import serial
import time
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64

WHEEL_BASE = 235 #[mm]
PORT = "/dev/ttyUSB0"

class DriveSense:
    def __init__(self):
        # Set up serial port
        self.ser = serial.Serial(port=PORT, baudrate=115200)
        
        # Set up Roomba Open Interface (OI)
        print("Starting Open Interface...")
        self.ser.write(bytes([128]))
        time.sleep(1)
        print("Setting Open Interface to Safe Mode...")
        self.ser.write(bytes([131]))
        time.sleep(1)

        # Initialize ROS pub/subs
        self.cmdSub = rospy.Subscriber("/cmd_vel", Twist, self.driveCmd)
        self.lblPub = rospy.Publisher("/lblRange", Range, queue_size=1)
        self.lbflPub = rospy.Publisher("/lbflRange", Range, queue_size=1)
        self.lbclPub = rospy.Publisher("/lbclRange", Range, queue_size=1)
        self.lbcrPub = rospy.Publisher("/lbcrRange", Range, queue_size=1)
        self.lbfrPub = rospy.Publisher("/lbfrRange", Range, queue_size=1)
        self.lbrPub = rospy.Publisher("/lbrRange", Range, queue_size=1)
        self.leftTickPub = rospy.Publisher("/left_ticks", Int64, queue_size=1)
        self.rightTickPub = rospy.Publisher("/right_ticks", Int64, queue_size=1)

    def driveCmd(self, cmd):
        # Get desired linear/angular speeds
        self.speed = cmd.linear.x * 500
        self.angle = cmd.angular.z

        # Compute left/right wheel velocities from diff drive model 
        vR = (WHEEL_BASE * self.angle)/2.0 + self.speed
        vL = (2.0*self.speed)-vR

        # Convert to bytes and send command to roomba
        vRBytes = self.int2bytes(self.int2sComp(int(vR),16))             
        vLBytes = self.int2bytes(self.int2sComp(int(vL),16))
        self.ser.write(bytes([145,vRBytes[0],vRBytes[1],vLBytes[0],vLBytes[1]]))

    def int2sComp(self, number, bits):
        if number < 0:
            return (1 << bits) + number
        else:
            return number

    def int2bytes(self, integer):
        return divmod(integer, 0x100)

    def getData(self):
        # Continuously request sensor data
        while True:
            self.ser.write(bytes([142,100]))
            self.byte = []
            while self.ser.in_waiting:
                # Read bytes
                self.byte.append(self.ser.read())
            
            # Publish light sensor range data
            self.publishRangeData('lbl', 57, 58, 80, self.lblPub)
            self.publishRangeData('lbfl', 59, 60, 80, self.lbflPub)
            self.publishRangeData('lbcl', 61, 62, 80, self.lbclPub)
            self.publishEncoderData('left_ticks', 52, 53, 80, self.leftTickPub)
            self.publishEncoderData('right_ticks', 54, 55, 80, self.rightTickPub)

            time.sleep(0.1)

    def publishRangeData(self, sensor, byte1, byte2, dataLength, publisher):
        if len(self.byte)==dataLength:
            # Get data (decimal) from data bytes
            raw = int.from_bytes(self.byte[byte1] + self.byte[byte2], "big", signed=False)
            # Populate ROS message
            h = Header()
            h.stamp = rospy.Time.now()
            h.frame_id = sensor
            rangeMsg = Range()
            rangeMsg.header = h
            rangeMsg.radiation_type = 1
            rangeMsg.field_of_view = 0.3
            rangeMsg.min_range = 0.0
            rangeMsg.max_range = 0.2
            rangeMsg.range = (float(700-raw)/700.0)*0.2
            
            # Publish data
            publisher.publish(rangeMsg)

        else:
            print("Data not formatted properly...")

    def publishEncoderData(self, sensor, byte1, byte2, dataLength, publisher):
        if len(self.byte)==dataLength:
            # Get data (decimal) from data bytes
            raw = int.from_bytes(self.byte[byte1] + self.byte[byte2], "big", signed=True)
            # Populate ROS message
            ticks = Int64()
            ticks.data = raw
            
            # Publish data
            publisher.publish(ticks)

        else:
            print("Data not formatted properly...")

if __name__ == '__main__':
    rospy.init_node("roombaDriveSense")
    roomba = DriveSense()
    roomba.getData()
    while True:
        try:
            rospy.spin()
        except KeyboardInterrupt:
            exit()
