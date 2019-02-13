#!/usr/bin/env python

import numpy as np
from rrt import RRT, Node
import rospy
from std_msgs.msg import Int32,Bool
from geometry_msgs.msg import PoseStamped
from iris_sim.msg import Progress

# Parse Gazebo world into RTree map
# Obs1: (-23,23), 6.8x5.4x14
# Obs2: (0,27), 25x10x6
# Obs3: (-30,10), 25x15x6
# Obs4: (-35,-6), 10x3x2
# Obs5: (40,-40), 10x10x50

o1 = np.array([(-26.4,20.3,0,-19.6,25.7,14)])
o2 = np.array([(-12.5,22,0,12.5,32,6)])
o3 = np.array([(-42.5,2.5,0,-17.5,17.5,6)])
o4 = np.array([(-40,-7.5,0,-30,-4.5,2)])
o5 = np.array([(35,-45,0,45,-35,50)])

obstacle = np.concatenate((o1,o2,o3,o4,o5))

Obstacle = np.array(
        [(20, 20, 20, 40, 40, 40), (20, 20, 60, 40, 40, 80), (20, 60, 20, 40, 80, 40), (60, 60, 20, 80, 80, 40),
        (60, 20, 20, 80, 40, 40), (60, 20, 60, 80, 40, 80), (20, 60, 60, 40, 80, 80), (60, 60, 60, 80, 80, 80)])

xRange = 50
yRange = 50
zRange = 80

q_init = Node(0,0,0)
q_goal = Node(-30,30,1)

rrt_path = RRT(obstacle,q_init,q_goal,xRange,yRange,zRange)
path = rrt_path.main()

path.reverse()

current = 1
global reach 
reach = False

# def sendPath():
#         path_publisher = rospy.Publisher('/iris/path_length', Int32, queue_size=10)
#         loop_rate = rospy.Rate(50)
#         while not rospy.is_shutdown():
#                 path_publisher.publish(len(path)-1)
#                 loop_rate.sleep()

def sendPoint():
        global reach, current
        point_publisher = rospy.Publisher('/iris/next_point', PoseStamped, queue_size=10)
        progress_publisher = rospy.Publisher('/iris/progress', Progress, queue_size=10)
        loop_rate = rospy.Rate(50)
        while not rospy.is_shutdown():
                nextPos = PoseStamped()
                currentProgress = Progress()

                nextPos.pose.position.x = path[current].x
                nextPos.pose.position.y = path[current].y
                nextPos.pose.position.z = path[current].z
                nextPos.header.stamp = rospy.Time.now()
                nextPos.header.frame_id = 'base_link'

                currentProgress.total_length = len(path)-1
                currentProgress.current_order = current

                point_publisher.publish(nextPos)
                progress_publisher.publish(currentProgress)

                print(reach)

                if(reach):
                        print("move to next point")
                        current = current + 1
                loop_rate.sleep()

def reachCallback(reachPoint):
        global reach
        reach = reachPoint

if __name__ == '__main__':
	rospy.init_node('sendpath_node',anonymous=True)
        rospy.Subscriber("/iris/reach_point", Bool, reachCallback)
        sendPoint()