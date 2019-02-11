#!/usr/bin/env python

import numpy as np
import math

# Implement 5th order trajectory optimization

class Optimize(object):

    def __init__(self,path):
        self.path = path

    def getDistance(self,p1,p2):
        distance = math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)
        return distance

    def getTime(self,speed):
        # Given the quad a constant speed of 2unit/s, calculate the timestamp and the length of each path segment
        total_length = 0
        total_time = 0
        for i in range(len(self.path)-1):
            seg_length = self.getDistance(self.path[i],self.path[i+1])
            total_length = total_length + seg_length
            total_time = total_time + seg_length/speed
        return total_time

    def matrix(self,point):
        M = np.array([[1,0,0,point.x],[0,1,0,point.y],[0,0,1,point.z],[0,0,0,1]])
        return M