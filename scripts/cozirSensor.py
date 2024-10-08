#!/usr/bin/env python3
import sys
import serial
import time
import rospy
import os
from std_msgs.msg import Int64
from cozir_ros.msg import cozirStamped


def talker(input_args):
# initialise the ros node for the sensor
    rospy.init_node('cozir',anonymous=True)
# setting the serial port of the COZIR sensor
    ser = serial.Serial("/dev/ttyS0")
# Initialisation
    rospy.loginfo("Running the Cozir-AX-1 Sensor")
    ser.flushInput()

    mask = 4160 # mask value to output temperature and humidity values

# Updating mask value based on the user preference of filtered or unfiltered co2 output; default is set to filtered
    if len(input_args)>1:
      if input_args[1] == "filtered":
          mask = mask + 4
      elif input_args[1] == "unfiltered":
          mask= mask + 2
    else:
      mask=mask + 4

# setting the filter parameter for the co2 reading output; default is set to 16
    if len(input_args) > 2:
        filter_param = int(input_args[2])
    else:
        filter_param = 16

# sending the mask to the sensor
    displayMode = "M {}\r\n".format(mask)
    ser.write(displayMode.encode("utf-8"))
    res=ser.read(10)

# logging the mask value set during operation
    if int(res[3:8])%10==2:
       rospy.loginfo("Unfiltered CO2 chosen")
    else:
       rospy.loginfo("Filtered CO2 chosen")

# Operation Mode configuration lines should be used to configure the sensor at the beginning
    #operMode = "K 2\r\n"
    #ser.write(operMode.encode("utf-8"))
    #res=ser.read(10)
    #print(res)

# Obtaining the multiplier required to convert co2 concentration
    ser.write(".\r\n".encode("utf-8"))
    res=ser.read(10)
    multiplier=int(res[3:8])
    #print(multiplier)

# Redundancy measure to reject filter parameter passed with an unfiltered co2 user choice passed during run time
    if mask%10 == 2:
       filter_param=None
       if len(input_args)>2:
         rospy.logwarn("Filter parameter rejected!!")

# Updating filter parameter if the current parameter is different from the user specified value corresponding to the filtered output mask
    filterParamRead = "a\r\n"
    ser.write(filterParamRead.encode("utf-8"))
    res = ser.read(10)
    curr_filtpara = int(res[3:8])
    if filter_param:
        if curr_filtpara != filter_param and len(input_args)>2:
            ser.write("A {}\r\n".format(filter_param).encode("utf-8"))
            res = ser.read(10)
            rospy.loginfo("Filter parameter updated!!")
        rospy.loginfo("Filter Parameter: {}".format(int(res[3:8])))

# initialising a ros-publisher to output data onto the network
    pub = rospy.Publisher('concentration', cozirStamped, queue_size=10)

    rate=rospy.Rate(0.5) # setting the publishing rate for the ros node
    readMode = "Q\r\n" # read mode set for the polling mode communication with the sensor

    ser.flushInput() # flushing all unwanted response on the serial port before publishing output

# setting the frame_id based on the robot namespace prefix for the robot
    frame_prefix=''
    if os.environ.get('bot_ns'):
        frame_prefix = os.environ.get('bot_ns')+'/'

# initiating publishing loop while ros is running
    while not rospy.is_shutdown():
        ser.write(readMode.encode("utf-8"))
        resp = ser.read(26)
        #print(resp)

        temperature = (int(resp[12:17])-1000)/10 # temperature in degree celcius
        #print(type(temperature))
        humidity = int(resp[4:9])/10 # relative humidity in percentage
        #print(type(humidity))
        co2 = int(resp[20:25])*multiplier # co2 concentration in PPM

        #print("T: {} C, RH: {} %, CO2: {} PPM".format(temperature, humidity, co2))
        pub.publish(T=temperature,RH=humidity,CO2=co2,stamp=rospy.get_rostime(),frame_id=frame_prefix+'cozir') #publishing the data
        #pub.publish(CO2=co2,stamp=rospy.get_rostime(),frame_id=frame_prefix+'cozir') #publishing the data
        rate.sleep()

if __name__ == '__main__':
    input_args = rospy.myargv(argv=sys.argv)
    #print(input_args)
    try:
        talker(input_args)
    except rospy.ROSInterruptException:
        pass

