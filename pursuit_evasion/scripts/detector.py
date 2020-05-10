#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, RegionOfInterest
import imutils
from imutils.object_detection import non_max_suppression
from imutils import paths
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import message_filters

class image_converter:

    def __init__(self):
        self.frame_count = 0
        #self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size = 10)
        self.roi_pub = rospy.Publisher('roi', String, queue_size = 20)
        self.detection_pub = rospy.Publisher('human_detection', Image, queue_size = 20)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/tb3_0/camera/rgb/image_raw",Image,self.callback)
        #self.depth_image_sub = message_filters.Subscriber("/tb3_0/camera/depth/image_raw",Image)
        #self.roi_sub= message_filters.Subscriber("roi",String)
        #self.ts=message_filters.ApproximateTimeSynchronizer([self.depth_image_sub,self.roi_sub],10, slop = 0.1, allow_headerless = True)
        #self.ts.registerCallback(self.depthcallback)


    def callback(self,data):
        if self.frame_count % 10 == 0:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
                #cv2.imshow("depth",cv_image)
            except CvBridgeError as e:
                rospy.loginfo(e)

            self.frame_count += 1
            hog = cv2.HOGDescriptor()
            hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
            image = imutils.resize(cv_image, width=min(400, cv_image.shape[1]))
            orig = cv_image.copy()
            (rects, weights) = hog.detectMultiScale(image, winStride=(4, 4),padding=(8, 8), scale=1.05)
            for (x, y, w, h) in rects:
                cv2.rectangle(orig, (x, y), (x + w, y + h), (0, 0, 255), 2)
            rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])

            if len(rects) > 0:
                pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)
                for (xA, yA, xB, yB) in pick:
                    cv2.rectangle(image, (xA, yA), (xB, yB), (0, 255, 0), 2)

                xA, yA, xB, yB = pick[0]
                centroid_x = (xA + xB) / 2.0
                centroid_y = (yA + yB) / 2.0
                height = yB - yA
                width = xB - xA
                #rospy.init_node('roi_publisher', anonymous = True)
                #rate = rospy.rate(1)
                #while not rospy.is_shutdown():
                roi_msg = "" + str(xA) + " " + str(yA) + " " + str(height) + " " + str(width)
                self.roi_pub.publish(roi_msg)
                try:
                    self.detection_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
                except CvBridgeError as e:
                    print(e)

                #self.detection_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
                # cv2.imshow("After NMS", image)
                # cv2.waitKey(1000)
                # cv2.destroyAllWindows()  
            
                # rospy.sleep(1.0)
            else:
                roi_msg = "" + str(0) + " " + str(0) + " " + str(0) + " " + str(0)
                self.roi_pub.publish(roi_msg)
                
            
        else:
            self.frame_count += 1


    
is_flength_set = False
def main():
        ic = image_converter()
        rospy.init_node('image_converter', anonymous=True)
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
        main()
 
