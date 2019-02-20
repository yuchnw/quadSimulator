#!/usr/bin/env python

import numpy as np
import random
import math
import plotly
import plotly.graph_objs as go
import uuid
from rtree import index

class Node(object):
    
    def __init__(self,x,y,z):
        self.x = x
        self.y = y
        self.z = z
        self.parent = None

class RRT(object):

    def __init__(self,obstacle,q_init,q_goal,xrange,yrange,zrange):
        self.q_init = q_init
        self.q_init.parent = None
        self.q_goal = q_goal
        self.delta_q = 1.0
        self.points = []
        self.path = []
        self.optimal = []
        self.checkNum = 0.1
        self.Obstacles = obstacle
        p = index.Property()
        p.dimension = 3
        self.obs = index.Index(self.getObstacle(), interleaved=True, properties=p)
        self.points.append(self.q_init)
        self.X_RANGE = xrange
        self.Y_RANGE = yrange
        self.Z_RANGE = zrange
    
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
        if point.x > self.X_RANGE:
            point.x = self.X_RANGE
        if point.y > self.Y_RANGE:
            point.y = self.Y_RANGE
        if point.z > self.Z_RANGE:
            point.z = self.Z_RANGE
        return point

    def check_line(self,p1,p2):
        # Return true if line NOT intersect with obstacle, otherwise false.
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
            self.q_goal.parent = point
            return True
        else:
            return False

    def getObstacle(self):
        for obs in self.Obstacles:
            yield (uuid.uuid4(), obs, obs)

    def getPath(self):
        self.path.append(self.q_goal)
        while(self.path[-1].parent!=self.q_init):
            self.path.append(self.path[-1].parent)
        self.path.append(self.q_init)
        return self.path

    def plot(self):
        plotly.tools.set_credentials_file(username='yuchnw', api_key='U4zDLjU3ftIprgEzyXTz')
        # Plot initial andg goal points
        trace1 = go.Scatter3d(
            x=[self.q_init.x,self.q_goal.x],
            y=[self.q_init.y,self.q_goal.y],
            z=[self.q_init.z,self.q_goal.z],
            mode='markers',
            marker=dict(
                size = 10,
                color = 'red',
                opacity = 1
            )
        )
        data = [trace1]
        # Plot all rrt trees
        for k in range(len(self.points)-1):
            trace = go.Scatter3d(
                x = [self.points[k+1].x,self.points[k+1].parent.x],
                y = [self.points[k+1].y,self.points[k+1].parent.y],
                z = [self.points[k+1].z,self.points[k+1].parent.z],
                mode = 'lines',
                line = dict(
                    color='black',
                    width = 5
                )
            )
            data.append(trace)
        # Plot feasible path
        for j in range(len(self.path)-1):
            path = go.Scatter3d(
                x = [self.path[j].x,self.path[j+1].x],
                y = [self.path[j].y,self.path[j+1].y],
                z = [self.path[j].z,self.path[j+1].z],
                mode = 'lines',
                line = dict(
                    color='green',
                    width = 5
                )
            )
            data.append(path)
        # Plot optimized path
        for m in range(len(self.optimal)-1):
            final = go.Scatter3d(
                x = [self.optimal[m].x,self.optimal[m+1].x],
                y = [self.optimal[m].y,self.optimal[m+1].y],
                z = [self.optimal[m].z,self.optimal[m+1].z],
                mode = 'lines',
                line = dict(
                    color='blue',
                    width = 5
                )
            )
            data.append(final)
        # Plot obstacles
        for O in self.Obstacles:
            obs = go.Mesh3d(
                x=[O[0], O[0], O[3], O[3], O[0], O[0], O[3], O[3]],
                y=[O[1], O[4], O[4], O[1], O[1], O[4], O[4], O[1]],
                z=[O[2], O[2], O[2], O[2], O[5], O[5], O[5], O[5]],
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
            ),
            showlegend = False
        )
        fig = go.Figure(data=data,layout=layout)
        plotly.plotly.plot(fig, filename='bars_gazebo_map',auto_open=False)

    def getOptimal(self):
        self.optimal = self.path[:]
        for i in range(100):
            minLength = len(self.optimal)
            pick1 = random.randint(1,minLength-1)
            pick2 = random.randint(1,minLength-1)
            if (pick1 != pick2) and (self.check_line(self.optimal[pick1],self.optimal[pick2])):
                if pick1 > pick2:
                    # self.optimal[pick2].parent = self.optimal[pick1]
                    del self.optimal[pick2+1:pick1]
                elif pick1 < pick2:
                    # self.optimal[pick1].parent = self.optimal[pick2]
                    del self.optimal[pick1+1:pick2]
        return self.optimal



    def main(self):
        while True:
            q_rand = Node(np.random.uniform(-self.X_RANGE,self.X_RANGE),np.random.uniform(-self.Y_RANGE,self.Y_RANGE),np.random.uniform(0,self.Z_RANGE))
            q_near = self.getNearest(q_rand)
            q_new = self.getNew(q_rand,q_near)
            if self.check_line(q_new,self.q_goal):
                self.points.append(q_new)
                q_new.parent = q_near
                self.points.append(self.q_goal)
                self.q_goal.parent = q_new
                break
            if not self.check_line(q_near,q_new):
                continue
            if not self.check_obstacle(q_new):
                continue
            q_new.parent = q_near
            self.points.append(q_new)
        print(len(self.points))
        self.getPath()
        print(len(self.path))
        self.getOptimal()
        print(len(self.optimal))
        self.plot()
        return self.optimal