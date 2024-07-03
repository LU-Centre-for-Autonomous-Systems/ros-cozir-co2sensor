#!/usr/bin/env python3
import sys
import serial
import time
import rospy
import os
from std_msgs.msg import Int64
from cozir.msg import concStamped


def talker(input_args):
    rospy.init_node('cozir',anonymous=True)
    ser = serial.Serial("/dev/ttyS0")
    rospy.loginfo("Running the Cozir-AX-1 Sensor")
    ser.flushInput()

    if input_args[1] == "filtered":
        mask=4
    elif input_args[1] == "unfiltered":
        mask=2
    else:
        mask=4

    if len(input_args) > 2:
        filter_param = int(input_args[2])
    else:
        filter_param = 16 

    displayMode = "M {}\r\n".format(mask)
    filterParamRead = "a\r\n"

    ser.write(displayMode.encode("utf-8"))
    res=ser.read(10)
    if int(res[3:8])==2:
       rospy.loginfo("Mask value set to unfiltered output")
    elif int(res[3:8])==4:
       rospy.loginfo("Mask value set to filtered output")
    else:
       rospy.loginfo("Mask value set to {}".format(res[3:8]))

    #operMode = "K 2\r\n"
    #ser.write(operMode.encode("utf-8"))
    #res=ser.read(10)
    #print(res)

    if mask == 2:
       filter_param=None
       rospy.logwarn("Filter parameter rejected!!")

    ser.write(filterParamRead.encode("utf-8"))
    res = ser.read(10)
    curr_filtpara = int(res[3:8])
    if filter_param:
        if curr_filtpara != filter_param:
            ser.write("A {}\r\n".format(filter_param).encode("utf-8"))
            res = ser.read(10)
            rospy.loginfo("Filter parameter updated!!")
        rospy.loginfo("Filter Parameter: {}".format(int(res[3:8])))

    pub = rospy.Publisher('concentration', concStamped, queue_size=10)

    rate=rospy.Rate(1)
    if mask == 2:
        readMode = "z\r\n"
    else:
        readMode = "Z\r\n"

    ser.flushInput()
    frame_prefix=''
    if os.environ.get('bot_ns'):
        frame_prefix = os.environ.get('bot_ns')

    while not rospy.is_shutdown():
        ser.write(readMode.encode("utf-8"))
        resp = ser.read(10)
        #print(resp)

        resp = resp[:8]
        conc = int(resp[2:])
        pub.publish(concentration=conc,stamp=rospy.get_rostime(),frame_id=frame_prefix+'/cozir')
        rate.sleep()

if __name__ == '__main__':
    input_args = rospy.myargv(argv=sys.argv)
    #print(input_args)
    try:
        talker(input_args)
    except rospy.ROSInterruptException:
        pass

