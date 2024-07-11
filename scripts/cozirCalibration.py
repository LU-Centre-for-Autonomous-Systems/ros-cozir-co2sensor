#!/usr/bin/env python3

import serial
import time
import rospy
from std_msgs.msg import Int64

def talker():
    #setting the serial port of the COZIR sensor
    ser = serial.Serial("/dev/ttyS0")
    print("Calibrating Cozir sensor...\n")

    # setting the calibration mode to fresh air calibration
    calMode = "G\r\n"

    # letting the sensor readings stabilise
    time.sleep(60)

    # sending the auto fresh air calibration command to the sensor
    ser.write(calMode.encode("utf-8"))
    resp = ser.read(10)
    print(resp)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

