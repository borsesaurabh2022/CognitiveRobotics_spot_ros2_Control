#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
from rclpy.node import Node

left_camera_mode = True
right_camera_mode = False

class SpotLinefollowerClass(Node):

    def __init__(self):
        super().__init__('spot_linefollower')
        self.rightheadcam_subscription = self.create_subscription(Image, '/Spot/left_head_camera/image_color', self.ImgCallback_LeftHead, 1)
        self.leftheadcam_subscription = self.create_subscription(Image, '/Spot/right_head_camera/image_color', self.ImgCallback_RightHead, 1)  
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        
        
    def ImgCallback_RightHead(self, msg):
        global cx1,cy1, IMAGE_H1, IMAGE_W1, left_camera_mode, right_camera_mode

        cv_bridge = CvBridge()
        cv_frame = cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        cv_image= cv2.resize(cv_frame, (320,190), interpolation = cv2.INTER_AREA)
        dimension = cv_image.shape
        IMAGE_H1, IMAGE_W1 = dimension[0], dimension[1]

        cv2.imshow('original_rightheadcam', cv_image)

        #defining point array for perspective transformation
        pt1 = [200,60]
        pt2 = [320,60]
        pt3 = [200,190]
        pt4 = [320,190]
        src = np.float32([pt1, pt2, pt3, pt4])
        dst = np.float32([[0, 0],[IMAGE_W1, 0],[0,IMAGE_H1],[IMAGE_W1, IMAGE_H1]])
        
        # The transformation matrix to get perspective transformation of the image from right head camera
        M = cv2.getPerspectiveTransform(src, dst) 
        cv_image = cv2.warpPerspective(cv_image, M, (IMAGE_W1, IMAGE_H1)) 
        dimension_1 = cv_image.shape
        Bird_eye_processed_height,Bird_eye_processed_width = dimension_1[0], dimension_1[1]


        perspective_proj = cv_image.copy()
        cv_image = cv2.GaussianBlur(cv_image,(5,5),0)
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        lower1 = np.array([0, 100, 110])
        upper1 = np.array([10, 255, 255])
        lower2 = np.array([160,50,50])
        upper2 = np.array([180,255,255])
        lower_mask = cv2.inRange(cv_image, lower1, upper1)
        upper_mask = cv2.inRange(cv_image, lower2, upper2)
        full_mask = lower_mask + upper_mask
        
        # Finding contour of the image
        cnts,hierarchy = cv2.findContours(full_mask, 1, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(cnts)>0:
            c = max(cnts, key=cv2.contourArea)

            M = cv2.moments(c)
            try:
                cx1, cy1 = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
                #print("cx co-ordinate = ",cx1,"cy co-ordinate = ", cy1)
            except ZeroDivisionError:
                print("Zero division region")
                cx1,cy1 = IMAGE_H1/2, IMAGE_W1/2
            cv2.drawContours(perspective_proj, cnts, -1, (0,255,0), 3)
            cv2.line(perspective_proj,(cx1,0),(cx1,720),(0,255,0),1)
            cv2.line(perspective_proj,(0,cy1),(1280,cy1),(0,255,0),1)
            
            if type(cx1) == int:
                if cx1<=150 and cy1>=130:
                    left_camera_mode = False
                    right_camera_mode = True
                    self.CmdPubCallback(cx1,cy1)
                else:
                    left_camera_mode = True
                    right_camera_mode = False
        else:
            cx1 = 0
            cy1 = 0
            self.CmdPubCallback(cx1,cy1)

        cv2.imshow('perspective_projection_right', perspective_proj)
        if cv2.waitKey(1) == ord('q'):
            cv2.destroyAllWindows()
            
    def ImgCallback_LeftHead(self, msg):
        global cx,cy, IMAGE_H, IMAGE_W

        cv_bridge = CvBridge()
        cv_frame = cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        cv_image= cv2.resize(cv_frame, (320,190), interpolation = cv2.INTER_AREA)
        dimension = cv_image.shape
        IMAGE_H, IMAGE_W = dimension[0], dimension[1]
        cv2.imshow('original_leftheadcam', cv_image)
        
        #defining point array for perspective transformation
        pt1 = [0,60]
        pt2 = [140,60]
        pt3 = [0,190]
        pt4 = [140,190]
        src = np.float32([pt1, pt2, pt3, pt4])
        dst = np.float32([[0, 0],[IMAGE_W, 0],[0,IMAGE_H],[IMAGE_W, IMAGE_H]])

        # The transformation matrix to get perspective transformation of the image from left head camera
        M = cv2.getPerspectiveTransform(src, dst) 
        cv_image = cv2.warpPerspective(cv_image, M, (IMAGE_W, IMAGE_H)) 
        dimension_1 = cv_image.shape
        Bird_eye_processed_height,Bird_eye_processed_width = dimension_1[0], dimension_1[1]

        perspective_proj = cv_image.copy()
        cv_image = cv2.GaussianBlur(cv_image,(5,5),0)
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        lower1 = np.array([0, 100, 110])
        upper1 = np.array([10, 255, 255])
        lower2 = np.array([160,100,20])
        upper2 = np.array([180,255,255])
        lower_mask = cv2.inRange(cv_image, lower1, upper1)
        upper_mask = cv2.inRange(cv_image, lower2, upper2)
        full_mask = lower_mask + upper_mask
        
        # Finding contour of the image
        cnts,hierarchy = cv2.findContours(full_mask, 1, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(cnts)>0:
            c = max(cnts, key=cv2.contourArea)

            M = cv2.moments(c)
            try:
                cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
                #print("cx co-ordinate = ",cx,"cy co-ordinate = ", cy)
            except ZeroDivisionError:
                print("Zero division region")
                cx,cy = IMAGE_H/2, IMAGE_W/2
            cv2.drawContours(perspective_proj, cnts, -1, (0,255,0), 3)
            cv2.line(perspective_proj,(cx,0),(cx,720),(0,255,0),1)
            cv2.line(perspective_proj,(0,cy),(1280,cy),(0,255,0),1)
            
            if type(cx) == int:
                self.CmdPubCallback(cx,cy)    
        else:
            cx = 0
            cy = 0
            self.CmdPubCallback(cx,cy)

        cv2.imshow('perspective_projection_left', perspective_proj)
        if cv2.waitKey(1) == ord('q'):
            cv2.destroyAllWindows()
      
    def CmdPubCallback(self, x, y):
        global left_camera_mode, right_camera_mode
        cmd = Twist()

        if left_camera_mode == True:
            error_x = x - IMAGE_W/3 

            if x == 0 and y == 0:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                print("Spot movement stopped")
            elif x>= 150.75 and 130<=y<=190:
                print("Spot turning in right direction to follow the ribbon")
                cmd.linear.x = 0.1
                cmd.angular.z = -error_x/100
            else:
                print("Spot heading straight to follow the ribbon") 
                cmd.linear.x = 0.35
                cmd.angular.z = -error_x/100

        elif right_camera_mode == True:
            error_x = x - IMAGE_W1/3

            if x<= 150 and 130<=y:
                print("Spot turning in left direction to follow the ribbon")
                cmd.linear.x = 0.1
                cmd.angular.z = -error_x/100
            else:
                print("Spot heading straight to follow the ribbon") 
                cmd.linear.x = 0.35
                cmd.angular.z = -error_x/100
            
        self.vel_publisher.publish(cmd)

def main():
    rclpy.init()
    followernode = SpotLinefollowerClass()
    try:
        rclpy.spin(followernode)
    except KeyboardInterrupt:
        pass
    followernode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


