#!/usr/bin/env python

import numpy as np
import math
import plotly
import plotly.graph_objs as go

X_RANGE = 3
Y_RANGE = 3
Z_RANGE = 3

class Node(object):
    
    def __init__(self,x,y,z):
        self.x = x
        self.y = y
        self.z = z

class RRT(object):

    def _init_(self):
        self.map = self.map[:,:,:].astype(int).T
        self.q_init = Node(2,2,-2.5)
        self.q_goal = Node(-2.5,2.5,2)
        self.delta_q = 0.1
        self.points = []
        self.parent = []
        self.checkNum = 100

        self.points.append(self.q_init)
        self.parent.append(self.q_init)
    
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

    def plot(self):
        x, y, z = np.random.multivariate_normal(np.array([0,0,0]), np.eye(3), 200).transpose()
        trace1 = go.Scatter3d(
            x=x,
            y=y,
            z=z,
            mode='markers',
            marker=dict(
                size=12,
                line=dict(
                    color='rgba(217, 217, 217, 0.14)',
                    width=0.5
                ),
                opacity=0.8
            )
        )
        data = [trace1]
        layout = go.Layout(
            margin=dict(
                l=0,
                r=0,
                b=0,
                t=0
            )
        )
        fig = go.Figure(data=data, layout=layout)
        plotly.offline.plot(fig, filename='3d_obstacle.html',auto_open=True)

    def main(self):
        while True:
            q_rand = Node(np.random.randint(-X_RANGE,X_RANGE),np.random.randint(-Y_RANGE,Y_RANGE),np.random.randint(-Z_RANGE,Z_RANGE))
            q_near = self.getNearest(q_rand)
            q_new = self.getNew(q_rand,q_near)
            if self.check_done(q_new):
                break
            if not self.check_line(q_near,q_new):
                continue
            if self.check_obstacle(q_new):
                continue
            self.points.append(q_new)
            self.parent.append(q_near)

if __name__=="__main__":
    # N_map = plt.imread("images/N_map.png")
    rrt = RRT()
    rrt.plot()