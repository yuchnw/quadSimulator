#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import String, Int32
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
from mavros_msgs.srv import *
from mavros_msgs.msg import *
# from iris_sim.msg import Progress

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
    rospy.wait_for_service('/mavros/cmd/takeoff')
    try:
        takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL) 
        takeoffService(altitude = 10, latitude = 47.3977420, longitude = 8.5455936, min_pitch = 0, yaw = 0)
    except rospy.ServiceException, e:
        print "Service takeoff call failed: %s"%e

def setLand():
    rospy.wait_for_service('/mavros/cmd/land')
    try:
        landService = rospy.ServiceProxy('/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
        landService(altitude = 0, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
    except rospy.ServiceException, e:
        print "service land call failed: %s. The vehicle cannot land "%e

def setLocation():
    locPub = rospy.Publisher('/mavros/setpoint_position/local',PoseStamped,queue_size=10)
    loop_rate = rospy.Rate(50)
    global x,y,z,path_length
    while not rospy.is_shutdown():
        goalPos = PoseStamped()
        goalPos.pose.position.x = 5
        goalPos.pose.position.y = 5
        goalPos.pose.position.z = 10
        goalPos.header.stamp = rospy.Time.now()
        goalPos.header.frame_id = 'base_link'

        locPub.publish(goalPos)
        setOffboard()

        if abs(x-goalPos.pose.position.x)<0.1 and abs(y-goalPos.pose.position.y)<0.1 and abs(z-goalPos.pose.position.z)<0.1:
            print "Reach target"
            break
        loop_rate.sleep()

def setOffboard():
    rospy.wait_for_service('/mavros/set_mode')
    try:
        landService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
        landService(custom_mode = "OFFBOARD")
    except rospy.ServiceException, e:
        print "service switch to offboard call failed: %s. The vehicle cannot land "%e

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

# def currentPositionCallback(currentPos):
#     global current_x
#     global current_y
#     global current_z
#     current_x = currentPos.pose.pose.position.x
#     current_y = currentPos.pose.pose.position.y
#     current_z = currentPos.pose.pose.position.z

def pathCallback(path):
    global path_length
    path_length = path
    
def userInput():
    enter ='1'
    while (not rospy.is_shutdown()):
        print "1: Set mode to ARM"
        print "2: Set mode to DISARM"
        print "3: Set mode to TAKEOFF"
        print "4: Set mode to LAND"
        print "5: Set mode to TARGET"
        print "x: Print GPS info"
        print "c: Exit"
        enter = raw_input("Enter your input: ")
        if (enter=='1'):
            setArm()
        elif(enter=='2'):
            setDisarm()
        elif(enter=='3'):
            setTakeoff()
        elif(enter=='4'):
            setLand()
        elif(enter=='5'):
            setLocation()
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
    rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, globalPositionCallback)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, localPositionCallback)
    # rospy.Subscriber("/mavros/global_position/local", PoseWithCovarianceStamped, currentPositionCallback)
    rospy.Subscriber("/iris/path_length", Int32, pathCallback)

    userInput()
    # move()