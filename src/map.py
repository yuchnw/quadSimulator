#!/usr/bin/env python

import numpy as np
from rrt import RRT, Node

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

q_init = Node(-30,30,1)
q_goal = Node(50,-50,30)

rrt_path = RRT(obstacle,q_init,q_goal,xRange,yRange,zRange)
path = rrt_path.main()