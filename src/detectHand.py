#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String
import sys
import numpy 
from std_msgs.msg import String
import roslib
import cv2
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from std_msgs.msg import String

class DetectHand:

    def __init__(self):
        self.hand_row_nw = None
        self.hand_row_se = None
        self.hand_col_nw = None
        self.hand_col_se = None
        self.hand_hist = None
        self.set_hist = False
        self.defects = None
        self.centroid = None

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
        self.set_hist = True

    def histogramMask(self,frame, hist):  
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        dst = cv2.calcBackProject([hsv], [0,1], hist, [0,180,0,256], 1)

        disc = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11,11))
        cv2.filter2D(dst, -1, disc, dst)

        ret, thresh = cv2.threshold(dst, 100, 255, 0)
        thresh = cv2.merge((thresh,thresh, thresh))

        cv2.GaussianBlur(dst, (3,3), 0, dst)

        res = cv2.bitwise_and(frame, thresh)
        return res

    def maxContour(self,frame):
        max_i = 0
        max_area = 0

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(gray, 0, 255, 0)
        # contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        _,contours,_= cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)	
        
        for i in range(len(contours)):
            cnt = contours[i]
            area = cv2.contourArea(cnt)
            if area > max_area:
                max_area = area
                max_i = i
        
        contour = contours[max_i]        
        return contour
    
    def getCentroidandFingertip(self,contour,frame):
        if contour is not None:
            hull = cv2.convexHull(contour,returnPoints=False)
            self.defects = cv2.convexityDefects(contour, hull)

        moments = cv2.moments(contour)
        if moments['m00'] != 0:
            cx = int(moments['m10']/moments['m00'])
            cy = int(moments['m01']/moments['m00'])
            self.centroid = cx,cy
            # print(centroid)
            cv2.circle(frame, self.centroid, 5, [255, 0, 255], -1)
            
        if self.centroid is not None and self.defects is not None:
            shape = self.defects[:,0][:,0]
            cx,cy = self.centroid

            x = np.array(contour[shape][:, 0][:, 0], dtype=np.float)
            y = np.array(contour[shape][:, 0][:, 1], dtype=np.float)

            pointX = cv2.pow(cv2.subtract(x, cx), 2)
            pointY = cv2.pow(cv2.subtract(y, cy), 2)
            distance = cv2.sqrt(cv2.add(pointX, pointY))
            maxDistanceIndex = np.argmax(distance)

            if maxDistanceIndex < len(shape):
                far_defect = shape[maxDistanceIndex]
                far_point = tuple(contour[far_defect][0])
                cv2.circle(frame, far_point, 5, [0, 0, 255], -1)
                return far_point
            else:
                return None
        

    def main(self):
        capture = cv2.VideoCapture(0)
        height = capture.get(cv2.CAP_PROP_FRAME_HEIGHT)
        width = capture.get(cv2.CAP_PROP_FRAME_WIDTH)
        while(capture.isOpened()):
            ret, frame_raw = capture.read()
            while not ret:
                ret,frame_raw = capture.read()
            frame_raw = cv2.flip(frame_raw,1)
            frame = frame_raw[:int(height),:int(width)]
            if cv2.waitKey(1) == ord('z') & 0xFF:
            #     if self.set_hist == False:
                self.handHistogram(frame)
            if self.set_hist == True:
                frame = self.histogramMask(frame,self.hand_hist)
                contour = self.maxContour(frame)
                farthest = self.getCentroidandFingertip(contour,frame)
            else:
                frame = self.drawRectangle(frame)
            
            # frame = np.vstack([hand_frame, frame])

            cv2.imshow('frame',frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        capture.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    DetectHand().main()