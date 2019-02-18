#!/usr/bin/env python

import numpy as np
import random
from rrt import RRT, Node
import rospy
from std_msgs.msg import Int32,Bool
from geometry_msgs.msg import PoseStamped
from iris_sim.msg import Progress,Feedback

# Parse Gazebo world into RTree map
# Obs1: (-23,23), 6.8x5.4x14
# Obs2: (0,27), 25x10x6
# Obs3: (-30,10), 25x15x6
# Obs4: (-35,-6), 10x3x2
# Obs5: (40,-40), 10x10x50

# o1 = np.array([(-27,20,0,-19,26,14)])
# o2 = np.array([(-12.5,22,0,12.5,32,6)])
# o3 = np.array([(-42.5,2.5,0,-17.5,17.5,6)])
# o4 = np.array([(-40,-7.5,0,-30,-4.5,2)])
# o5 = np.array([(35,-45,0,45,-35,50)])

pp = []

for i in range(8):
        for j in range(8):
                pp.append([-40+10*i,-40+10*j,0,-35+10*i,-35+10*j,random.randint(10,20)])

# obstacle = np.concatenate((o1,o2,o3,o4,o5))

obstacle = np.asarray(pp)

xRange = 50
yRange = 50
zRange = 40

q_init = Node(-47,-47,3)
q_goal = Node(20,45,1)

rrt_path = RRT(obstacle,q_init,q_goal,xRange,yRange,zRange)
path = rrt_path.main()

path.reverse()

map_current = 1
global reach
reach = 0

def sendPoint():
        global reach, map_current, quad_at
        point_publisher = rospy.Publisher('/iris/next_point', PoseStamped, queue_size=10)
        progress_publisher = rospy.Publisher('/iris/progress', Progress, queue_size=10)
        loop_rate = rospy.Rate(30)
        while not rospy.is_shutdown():
                if map_current == len(path):
                        print("done")
                        break
                nextPos = PoseStamped()
                currentProgress = Progress()

                # Send the pose position of the target point
                nextPos.pose.position.x = path[map_current].x
                nextPos.pose.position.y = path[map_current].y
                nextPos.pose.position.z = path[map_current].z
                nextPos.header.stamp = rospy.Time.now()
                nextPos.header.frame_id = 'base_link'

                currentProgress.total_length = len(path)-1
                currentProgress.map_current = map_current

                point_publisher.publish(nextPos)
                progress_publisher.publish(currentProgress)


                if(reach==1 and quad_at == map_current + 1):
                        # Set the target point to the next waypoint
                        print("move to next point")
                        map_current = quad_at
                loop_rate.sleep()


def feedbackCallback(feedback):
        global reach, quad_at
        reach = feedback.reach_current
        quad_at = feedback.quad_current

if __name__ == '__main__':
	rospy.init_node('sendpath_node',anonymous=True)
        rospy.Subscriber("/iris/feedback", Feedback, feedbackCallback)
        # sendPoint()