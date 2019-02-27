#!/usr/bin/env python
# import rospy
import numpy as np
from std_msgs.msg import String
import sys
import numpy 
from std_msgs.msg import String
# import roslib
import cv2
import imutils
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from std_msgs.msg import String

def drawRectangle(self, frame):  
    rows,cols,_ = frame.shape

    self.hand_row_nw = np.array([6*rows/20,6*rows/20,6*rows/20,10*rows/20,10*rows/20,10*rows/20,14*rows/20,14*rows/20,14*rows/20])
    self.hand_col_nw = np.array([9*cols/20,10*cols/20,11*cols/20,9*cols/20,10*cols/20,11*cols/20,9*cols/20,10*cols/20,11*cols/20])

    self.hand_row_se = self.hand_row_nw + 10
    self.hand_col_se = self.hand_col_nw + 10

    size = self.hand_row_nw.size

    for i in xrange(size):
        cv2.rectangle(frame,(self.hand_col_nw[i],self.hand_row_nw[i]),(self.hand_col_se[i],self.hand_row_se[i]),(0,255,0),1)
        black = np.zeros(frame.shape, dtype=frame.dtype)
        frame_final = np.vstack([black, frame])
        return frame_final

def handHistogram(self, frame):  
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    roi = np.zeros([90,10,3], dtype=hsv.dtype)

    size = self.hand_row_nw.size
    for i in xrange(size):
        roi[i*10:i*10+10,0:10] = hsv[self.hand_row_nw[i]:self.hand_row_nw[i]+10, self.hand_col_nw[i]:self.hand_col_nw[i]+10]

    self.hand_hist = cv2.calcHist([roi],[0, 1], None, [180, 256], [0, 180, 0, 256])
    cv2.normalize(self.hand_hist, self.hand_hist, 0, 255, cv2.NORM_MINMAX)

def histogramMask(frame, hist):  
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    dst = cv2.calcBackProject([hsv], [0,1], hist, [0,180,0,256], 1)

    disc = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11,11))
    cv2.filter2D(dst, -1, disc, dst)

    ret, thresh = cv2.threshold(dst, 100, 255, 0)
    thresh = cv2.merge((thresh,thresh, thresh))

    cv2.GaussianBlur(dst, (3,3), 0, dst)

    res = cv2.bitwise_and(frame, thresh)
    return res

def maxContour(frame):
    max_i = 0
    max_area = 0

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret,thresh = cv2.threshold(gray, 0, 255, 0)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)	
    
    for i in range(len(contours)):
        cnt = contours[i]
        area = cv2.contourArea(cnt)
        if area > max_area:
            max_area = area
            max_i = i
    
    contour = contours[max_i]
    return contour

if __name__ == '__main__':
    capture = cv2.VideoCapture(0)
    height = capture.get(cv2.CAP_PROP_FRAME_HEIGHT)
    width = capture.get(cv2.CAP_PROP_FRAME_WIDTH)
    while(capture.isOpened()):
        ret, frame_raw = capture.read()
        while not ret:
            ret,frame_raw = capture.read()
        frame_raw = cv2.flip(frame_raw,1)
        frame = frame_raw[:int(height),:int(width)]
        print("!")
        cv2.imshow('frame', frame_raw)
    capture.release()
    cv2.destroyAllWindows()