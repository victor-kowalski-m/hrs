#!/usr/bin/env python

#Group D: Selin Kesler, Victor Kowalski and Miriam Senne

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class Central:


    def __init__(self):
        # initialize class variables
        pass

    def image_cb(self,data, cam):
        bridge_instance = CvBridge()
        try:
            cv_image = bridge_instance.imgmsg_to_cv2(data,"bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        
        if cam == 'top':
            # Extract red
            # we transform it into HSV
            image_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            lower = np.array([161,155,84])
            upper = np.array([179,255,255])
            mask1 = cv2.inRange(image_hsv, lower, upper)
            mask2 = cv2.inRange(image_hsv, np.array([0,155,84]), np.array([20,255,255]))

            mask = cv2.bitwise_or(mask1, mask2)

            erosion_shape = cv2.MORPH_ELLIPSE
            erosion_size = 4
            element_erosion = cv2.getStructuringElement(erosion_shape, (2 * erosion_size + 1, 2 * erosion_size + 1),
                                        (erosion_size, erosion_size))
            erosion_dst = cv2.erode(mask, element_erosion)

            dilation_shape = cv2.MORPH_ELLIPSE
            dilation_size = 1                            
            element_dilation = cv2.getStructuringElement(dilation_shape, (2 * dilation_size + 1, 2 * dilation_size + 1),
                                        (dilation_size, dilation_size))
            dilation_image = cv2.dilate(erosion_dst, element_dilation)


            new_rgb = np.zeros((240,320,3), dtype="uint8")
            new_rgb[:,:,0] = dilation_image # for red
            new_rgb[:,:,1] = dilation_image # for green
            new_rgb[:,:,2] = dilation_image # for blue

            # we find countours
            im2, contours, hierarchy = cv2.findContours(dilation_image,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            areas_list = []
            for c in contours:
                area = cv2.contourArea(c)
                areas_list.append(area)

            cv2.imshow("Top camera",cv_image)
            cv2.imshow("Original image",cv_image)
            
            # Find the largest colour blob and calculate the position of its center in pixel coordinates 
            # Index of largest blob
            if len(areas_list) != 0:
                idx = np.argmax(areas_list)     

                # Calculate the moment of largest blob   
                M = cv2.moments(contours[idx])

                # Get x,y coordinate of centroid
                try:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    self.BlobX = cx
                    self.BlobY = cy

                    # Show centroid in image
                    cv2.circle(new_rgb, (cx,cy), 5, (255, 0, 0), -1)

                except ZeroDivisionError:
                    pass           
        
            cv2.imshow("Color extraction", mask)

            cv2.imshow("Color extraction", dilation_image)
            cv2.imshow("Blob extraction", new_rgb)

        # bottom camera
        else: 
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            gray = cv2.medianBlur(gray, 5)
    
            rows = gray.shape[0]
            circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, rows / 8,
                                    param1=100, param2=30,
                                    minRadius=1, maxRadius=30)
    
    
            cv2.imshow("Bottom camera", cv_image)

            if circles is not None:
                circles = np.uint16(np.around(circles))
                for i in circles[0, :]:
                    center = (i[0], i[1])
                    # circle center
                    cv2.circle(cv_image, center, 1, (0, 100, 100), 3)
                    # circle outline
                    radius = i[2]
                    cv2.circle(cv_image, center, radius, (255, 0, 255), 3)
    
    
            cv2.imshow("Blurred grayscale", gray)
            cv2.imshow("Circular shapes", cv_image)

        cv2.waitKey(3) # a small wait time is needed for the image to be displayed correctly

        
    def central_execute(self):
        rospy.init_node('tutorial_2',anonymous=True) #initilizes node, sets name

        rospy.Subscriber("/nao_robot/camera/top/camera/image_raw",Image,self.image_cb, 'top')
        rospy.Subscriber("/nao_robot/camera/bottom/camera/image_raw",Image,self.image_cb, 'bottom')
        rate = rospy.Rate(3)  # sets the sleep time to 3ms
        while not rospy.is_shutdown():

            rate.sleep()

if __name__=='__main__':
    # instantiate class and start loop function
    central_instance = Central()
    central_instance.central_execute()
