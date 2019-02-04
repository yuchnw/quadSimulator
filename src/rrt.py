#!/usr/bin/env python

import numpy as np

X_RANGE = 100
Y_RANGE = 100
Z_RANGE = 100

class Node(object):
    
    def __init__(self,x,y,z):
        self.x = x
        self.y = y
        self.z = z
    
    def getNear(self):
        return 0

    def getNew(self):
        q_rand = Node(np.random.randint(0,X_RANGE),np.random.randint(0,Y_RANGE),np.random.randint(0,Z_RANGE))