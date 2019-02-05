#!/usr/bin/env python

import numpy as np
import math

X_RANGE = 100
Y_RANGE = 100
Z_RANGE = 100

class Node(object):
    
    def __init__(self,x,y,z):
        self.x = x
        self.y = y
        self.z = z

class RRT(object):

    def _init_(self,map,x,y,z,q_init,q_goal,delta_q):
        self.map = map[:,:,:].astype(int).T
        self.x = x
        self.y = y
        self.z = z
        self.q_init = Node(10,10,0)
        self.q_goal = Node(75,75,50)
        self.delta_q = delta_q
        self.points = []
        self.parent = []
        self.checkNum = 100

        self.points.append(q_init)
        self.parent.append(q_init)
    
    def getNearest(self,point):
        minDistance = self.getDistance(point,self.points[0])
        near = self.points[0]
        for q in self.points:
            if self.getDistance(point,q) < minDistance:
                minDistance = self.getDistance(point,q)
                near = q
        return near

    def getNew(self,rand,near):
        scale = self.delta_q/math.sqrt(self.getDistance(rand,near))
        new_X = near.x + scale*(rand.x-near.x)
        new_Y = near.y + scale*(rand.y-near.y)
        new_Z = near.z + scale*(rand.z-near.z)
        new = self.check_boundary(Node(new_X,new_Y,new_Z))
        return new

    def check_obstacle(self,point):
        if self.map[point.x,point.y,point.z] == 0:
            return True
        else:
            return False

    def getDistance(self,p1,p2):
        distance = (p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2
        return distance

    def check_boundary(self,point):
        if point.x > X_RANGE:
            point.x = X_RANGE
        if point.y > Y_RANGE:
            point.y = Y_RANGE
        if point.z > Z_RANGE:
            point.z = Z_RANGE
        return point

    def check_line(self,p1,p2):
        dx = (p2.x-p1.x)/self.checkNum
        dy = (p2.y-p1.y)/self.checkNum
        dz = (p2.z-p1.z)/self.checkNum
        for i in range(self.checkNum):
            temp = Node(dx+i*p1.x,dy+i*p2.y,dz+i*p1.z)
            if self.check_obstacle(temp):
                # current line segment is within obstacle
                return False
        return True

    def check_done(self,point):
        if self.check_line(point,self.q_goal):
            self.points.append(self.q_goal)
            self.parent.append(point)
            return True
        return False

    def main(self):
        while True:
            q_rand = Node(np.random.randint(0,X_RANGE),np.random.randint(0,Y_RANGE),np.random.randint(0,Z_RANGE))
            q_near = self.getNearest(q_rand)
            q_new = self.getNew(q_rand,q_near)
            if self.check_line(q_near,q_new):
                self.points.append(q_new)
                self.parent.append(q_near)
                continue
        return 0