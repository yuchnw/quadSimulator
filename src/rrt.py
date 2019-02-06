#!/usr/bin/env python

import numpy as np
import random
import math
import plotly
import plotly.graph_objs as go
import uuid
from rtree import index

X_RANGE = 100
Y_RANGE = 100
Z_RANGE = 100

class Node(object):
    
    def __init__(self,x,y,z):
        self.x = x
        self.y = y
        self.z = z

class RRT(object):

    def __init__(self):
        self.q_init = Node(0,99,0)
        self.q_goal = Node(50,0,50)
        self.delta_q = 10
        self.points = []
        self.parent = []
        self.checkNum = 1
        self.Obstacles = np.array(
        [(20, 20, 20, 40, 40, 40), (20, 20, 60, 40, 40, 80), (20, 60, 20, 40, 80, 40), (60, 60, 20, 80, 80, 40),
        (60, 20, 20, 80, 40, 40), (60, 20, 60, 80, 40, 80), (20, 60, 60, 40, 80, 80), (60, 60, 60, 80, 80, 80)])
        p = index.Property()
        p.dimension = 3
        self.obs = index.Index(self.getObstacle(), interleaved=True, properties=p)
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
        scale = float(self.delta_q/math.sqrt(self.getDistance(rand,near)))
        new_X = near.x + scale*(rand.x-near.x)
        new_Y = near.y + scale*(rand.y-near.y)
        new_Z = near.z + scale*(rand.z-near.z)
        print(scale)
        new = self.check_boundary(Node(new_X,new_Y,new_Z))
        return new

    def check_obstacle(self,point):
        #True if not inside of obstacle, false otherwise
        if self.obs.count((point.x,point.y,point.z)) == 0:
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
        count = math.sqrt(self.getDistance(p1,p2))/self.checkNum
        dx = (p2.x-p1.x)/count
        dy = (p2.y-p1.y)/count
        dz = (p2.z-p1.z)/count
        for i in range(int(count)):
            temp = Node(i*dx+p1.x,i*dy+p1.y,i*dz+p1.z)
            if self.check_obstacle(temp) == False:
                # current line segment is within obstacle
                return False
        return True

    def check_done(self,point):
        if self.check_line(point,self.q_goal) == True:
            self.points.append(self.q_goal)
            self.parent.append(point)
            return True
        else:
            return False

    def getObstacle(self):
        for obs in self.Obstacles:
            yield (uuid.uuid4(), obs, obs)

    def getMap(self,obs):
        return

    def plot(self):
        plotly.tools.set_credentials_file(username='yuchnw', api_key='U4zDLjU3ftIprgEzyXTz')
        trace1 = go.Scatter3d(
            x=[0,50],
            y=[99,0],
            z=[0,50],
            mode='markers',
            marker=dict(
                size=20,
                line=dict(
                    color='rgba(217, 217, 217, 0.14)',
                    width=10
                ),
                opacity=0.8
            )
        )
        data = [trace1]
        Obstacles = np.array(
        [(20, 20, 20, 40, 40, 40), (20, 20, 60, 40, 40, 80), (20, 60, 20, 40, 80, 40), (60, 60, 20, 80, 80, 40),
        (60, 20, 20, 80, 40, 40), (60, 20, 60, 80, 40, 80), (20, 60, 60, 40, 80, 80), (60, 60, 60, 80, 80, 80)])
        for O_i in Obstacles:
            obs = go.Mesh3d(
                x=[O_i[0], O_i[0], O_i[3], O_i[3], O_i[0], O_i[0], O_i[3], O_i[3]],
                y=[O_i[1], O_i[4], O_i[4], O_i[1], O_i[1], O_i[4], O_i[4], O_i[1]],
                z=[O_i[2], O_i[2], O_i[2], O_i[2], O_i[5], O_i[5], O_i[5], O_i[5]],
                i=[7, 0, 0, 0, 4, 4, 6, 6, 4, 0, 3, 2],
                j=[3, 4, 1, 2, 5, 6, 5, 2, 0, 1, 6, 3],
                k=[0, 7, 2, 3, 6, 7, 1, 1, 5, 5, 7, 6],
                color='#FFB6C1',
                opacity=1
            )
            data.append(obs)
        layout = go.Layout(
            margin=dict(
                l=0,
                r=0,
                b=0,
                t=0
            )
        )
        fig = go.Figure(data=data, layout=layout)
        plotly.plotly.plot(fig, filename='3d_cube',auto_open=True)

    def main(self):
        while True:
            q_rand = Node(np.random.uniform(0,X_RANGE),np.random.uniform(0,Y_RANGE),np.random.uniform(0,Z_RANGE))
            q_near = self.getNearest(q_rand)
            # print(q_near.x,q_near.y,q_near.z)
            q_new = self.getNew(q_rand,q_near)
            # print(q_new.x,q_new.y,q_new.z)
            if 0.1 and random.random() < 0.1:
                if self.check_done(q_new):
                    print(len(self.points))
                    return self.points 
            if not self.check_line(q_near,q_new):
                continue
            if self.check_obstacle(q_new):
                continue
            self.points.append(q_new)
            self.parent.append(q_near)
        print(len(self.points))
        # self.plot()

if __name__=="__main__":
    rrt = RRT()
    rrt.main()