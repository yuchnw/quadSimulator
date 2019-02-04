#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import *
from mavros_msgs.msg import *

#global variable
latitude = 0.0
longitude = 0.0
altitude = 0.0
          
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
    rospy.wait_for_service('/mavros/setpoint_position/local')
    try:
        takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL) 
        takeoffService(altitude = 5, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
    except rospy.ServiceException, e:
        print "Service takeoff call failed: %s"%e

def setLand():
    rospy.wait_for_service('/mavros/cmd/land')
    try:
        landService = rospy.ServiceProxy('/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
        isLanding = landService(altitude = 0, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
    except rospy.ServiceException, e:
        print "service land call failed: %s. The vehicle cannot land "%e

def setLocation():
       goalPos = PoseStamped()
       goalPos.pose.position.x = 0
       goalPos.pose.position.y = 0
       goalPos.pose.position.z = 15
       goalPos.header.stamp = rospy.Time.now()
       goalPos.header.frame_id = 'map'

    #    locPub.publish(goalPos)

def move():
    locPub = rospy.Publisher('/mavros/setpoint_position/local',PoseStamped,queue_size=10)
    # locPub=SP.get_pub_position_local(queue_size=10)
    loop_rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        goalPos = PoseStamped()
        goalPos.pose.position.x = 0
        goalPos.pose.position.y = 0
        goalPos.pose.position.z = 15
        goalPos.header.stamp = rospy.Time.now()
        goalPos.header.frame_id = 'base_link'

        locPub.publish(goalPos)
        loop_rate.sleep()

def globalPositionCallback(globalPos):
    global latitude
    global longitude
    global altitude
    latitude = globalPos.latitude
    altitude = globalPos.altitude
    longitude = globalPos.longitude

def localPositionCallback(localPos):
    global x
    global y
    global z
    x = localPos.pose.position.x
    y = localPos.pose.position.y
    z = localPos.pose.position.z
    
def userInput():
    enter ='1'
    while ((not rospy.is_shutdown())and (enter in ['1','2','3','4','x'])):
        print "1: Set mode to ARM"
        print "2: Set mode to DISARM"
        print "3: Set mode to TAKEOFF"
        print "4: Set mode to LAND"
        print "x: Print GPS info"
        enter = raw_input("Enter your input: ")
        if (enter=='1'):
            setArm()
        elif(enter=='2'):
            setDisarm()
        elif(enter=='3'):
            setLocation()
            # setTakeoff()
        elif(enter=='4'):
            setLand()
        elif(enter=='x'):
            global latitude
            global longitude
            global altitude
            global x
            global y
            global z
            print ("longitude: %.7f" %longitude)
            print ("latitude: %.7f" %latitude)
            print ("altitude: %.7f" %altitude)
            print ("x: %.5f" %x)
            print ("y: %.5f" %y)
            print ("z: %.5f" %z)
        elif(enter=='c'): 
            print "Exit"
            break
        else:
            print "Invalid command!"

if __name__ == '__main__':
    rospy.init_node('iris_node', anonymous=True)
    # rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, globalPositionCallback)
    # rospy.Subscriber("/mavros/local_position/pose", PoseStamped, localPositionCallback)
     

    # userInput()
    move()