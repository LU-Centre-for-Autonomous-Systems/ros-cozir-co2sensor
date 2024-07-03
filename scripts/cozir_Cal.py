#!/usr/bin/env python3
import serial
import time
import rospy
from std_msgs.msg import Int64

def talker():
    ser = serial.Serial("/dev/ttyS0")
    print("Calibrating Cozir-AH-1 sensor\n")

    displayMode = "M 4\r\n"
    operMode = "K 2\r\n"
    filterParamRead = "a\r\n"

    #ser.write(displayMode.encode("utf-8"))
    #res=ser.read(10)
    #print(res)

    #ser.write(operMode.encode("utf-8"))
    #res=ser.read(10)
    #print(res)

    #ser.write(filterParamRead.encode("utf-8"))
    #res=ser.read(10)
    #print(res)

    calMode = "G\r\n"

    time.sleep(60)

    ser.write(calMode.encode("utf-8"))
    resp = ser.read(10)
    print(resp)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

