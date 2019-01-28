#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import *

#global variable
latitude = 0.0
longitude = 0.0
altitude = 0.0

def setLand():
    rospy.wait_for_service('/mavros/cmd/land')
    try:
        landService = rospy.ServiceProxy('/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
        #http://wiki.ros.org/mavros/CustomModes for custom modes
        isLanding = landService(altitude = 0, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
    except rospy.ServiceException, e:
        print "service land call failed: %s. The vehicle cannot land "%e
          
def setArm():
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(True)
    except rospy.ServiceException, e:
        print "Service arm call failed: %s"%e
        
def setDisarm():
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(False)
    except rospy.ServiceException, e:
        print "Service arm call failed: %s"%e


def setTakeoff():
    rospy.wait_for_service('/mavros/cmd/takeoff')
    try:
        takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL) 
        # takeoffService(altitude = 5, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
        takeoffService(0, 0, 0, 0, 5)
    except rospy.ServiceException, e:
        print "Service takeoff call failed: %s"%e
    
    

def globalPositionCallback(globalPositionCallback):
    global latitude
    global longitude
    global altitude
    latitude = globalPositionCallback.latitude
    altitude = globalPositionCallback.altitude
    longitude = globalPositionCallback.longitude
    #print ("longitude: %.7f" %longitude)
    #print ("latitude: %.7f" %latitude)

def menu():
    print "1: to set mode to ARM"
    print "2: to set mode to DISARM"
    print "3: to set mode to TAKEOFF"
    print "4: to set mode to LAND"
    print "x: to print GPS info"
    
def myLoop():
    x='1'
    while ((not rospy.is_shutdown())and (x in ['1','2','3','4','x'])):
        menu()
        x = raw_input("Enter your input: ")
        if (x=='1'):
            setArm()
        elif(x=='2'):
            setDisarm()
        elif(x=='3'):
            setTakeoff()
        elif(x=='4'):
            setLand()
        elif(x=='x'):
            global latitude
            global longitude
            global altitude
            print ("longitude: %.7f" %longitude)
            print ("latitude: %.7f" %latitude)
            print ("altitude: %.7f" %altitude)
        else: 
            print "Exit"
        
        
    

if __name__ == '__main__':
    rospy.init_node('iris_node', anonymous=True)
    rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, globalPositionCallback)

    myLoop()