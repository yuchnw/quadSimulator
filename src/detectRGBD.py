#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String
import sys
from geometry_msgs.msg import PoseStamped
import cv2
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
import message_filters

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
        self.path = []
        self.start = False
        self.ok = False
        self.prevAlt = 3
        self.x = 0
        self.y = 0
        self.z = 3
        self.farthest = None

    def drawRectangle(self, frame):  
        rows,cols,_ = frame.shape

        self.hand_row_nw = np.array([6*rows/20,6*rows/20,6*rows/20,10*rows/20,10*rows/20,10*rows/20,14*rows/20,14*rows/20,14*rows/20])
        self.hand_col_nw = np.array([9*cols/20,10*cols/20,11*cols/20,9*cols/20,10*cols/20,11*cols/20,9*cols/20,10*cols/20,11*cols/20])

        self.hand_row_se = self.hand_row_nw + 10
        self.hand_col_se = self.hand_col_nw + 10

        size = self.hand_row_nw.size
        for i in xrange(size):
            # cv2.rectangle(frame,(self.hand_col_nw[i],self.hand_row_nw[i]),(self.hand_col_se[i],self.hand_row_se[i]),(0,255,0),1)
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
        _,contours,_= cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)	
        
        for i in range(len(contours)):
            cnt = contours[i]
            area = cv2.contourArea(cnt)
            if area > max_area:
                max_area = area
                max_i = i
        
        contour = contours[max_i]
        cv2.drawContours(frame,[contour],0,(255,0,0),2)
        # res = cv2.bitwise_and(frame, frame, mask = contour)
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

    def depthCallback(self,image):
        from cv_bridge import CvBridge, CvBridgeError
        bridge = CvBridge()
        depth_image = bridge.imgmsg_to_cv2(image, desired_encoding="32FC1")
        # z = depth_image.distance
        # print(z)
        # depth_array = np.array(depth_image, dtype=np.float32)
        # cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
        # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    def getDepth(self,rgb,depth):
        from cv_bridge import CvBridge, CvBridgeError
        bridge = CvBridge()
        try:
            image = bridge.imgmsg_to_cv2(rgb, "bgr8")
            depth_image = bridge.imgmsg_to_cv2(depth, "32FC1")
        except CvBridgeError, e:
            print e
        depth_array = np.array(depth_image, dtype=np.float32)
        # cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
        # print(depth_array[20][319])
        self.operate(image,depth_array)
        # cv2.imshow('Body Recognition', depth_array)
        cv2.waitKey(3)
        return
    
    def imageCallback(self,image):
        from cv_bridge import CvBridge, CvBridgeError
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
        new_image = cv2.resize(cv_image,(640,480))
        cv2.imshow("iris",new_image)

    def main(self):
        # rospy.Subscriber("/iris/camera_red_iris/image_raw", Image, self.imageCallback)
    #     rospy.Subscriber("/camera/depth/image_raw", Image, self.depthCallback)
        image_sub = message_filters.Subscriber("/camera/rgb/image_color", Image)
        depth_sub = message_filters.Subscriber("/camera/depth/image_raw", Image)
        # depth_sub = message_filters.Subscriber("/camera/depth_registered/image_raw", Image)
        both = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub], 10, 0.5)
        both.registerCallback(self.getDepth)
        loop_rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            hand_publisher = rospy.Publisher('/iris/hand', PoseStamped, queue_size=10)
            handPos = PoseStamped()
            handPos.pose.position.x = self.x
            handPos.pose.position.y = self.y
            handPos.pose.position.z = self.z
            hand_publisher.publish(handPos)

            loop_rate.sleep()
            # rospy.spin()

    def operate(self,cv_image,depth_array):
        
        if cv2.waitKey(1) == ord('z') & 0xFF:
            self.handHistogram(cv_image)
        if cv2.waitKey(1) == ord('a') & 0xFF:
            self.ok = True
        if self.set_hist == True:
            cv_image = self.histogramMask(cv_image,self.hand_hist)
            contour = self.maxContour(cv_image)
            self.farthest = self.getCentroidandFingertip(contour,cv_image)
            if self.ok == True:
                if self.start == False:
                    self.path.append(self.farthest)
                    self.start = True
                else:
                    dx = self.path[-1][0] - self.farthest[0]
                    dy = self.path[-1][1] - self.farthest[1]
                    dist = np.sqrt(dx*dx + dy*dy)
                    if dist > 10 and dist < 50:
                        # print(self.farthest[1])
                        height = depth_array[self.centroid[1]][self.centroid[0]]
                        print(height)
                        self.path.append(self.farthest)
                        self.x = self.farthest[0]/640.0*100 -50
                        self.y = 70 - self.farthest[1]/480.0*120
                        if height == 0.0 or height > 1000.0:
                            self.z = self.prevAlt
                        else:
                            self.prevAlt = height/100.0
                            self.z = self.prevAlt
                if len(self.path) > 20:
                    self.path.pop(0)
                if self.path is not None:
                    for i in range(len(self.path)-1):
                        cv2.circle(cv_image, self.path[i], 4, [0, 0, 255], -1)
        if self.ok == False:
            cv_image = self.drawRectangle(cv_image)
        
        cv2.imshow('frame',cv_image)
        
        return

if __name__ == '__main__':
    rospy.init_node('detectHand_node',anonymous=True)
    
    DetectHand().main()