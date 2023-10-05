# #!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
from geometry_msgs.msg import Twist

left_camera_mode = False
right_camera_mode = True

class SpotLinefollowerClass(Node):

    def __init__(self):
        super().__init__('spot_linefollower')
        self.rightheadcam_subscription = self.create_subscription(Image, '/Spot/left_head_camera/image_color', self.ImgCallback_LeftHead, 1)
        self.leftheadcam_subscription = self.create_subscription(Image, '/Spot/right_head_camera/image_color', self.ImgCallback_RightHead, 1)  
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        
    def ImgCallback_RightHead(self, msg):
        global right_camera_mode, left_camera_mode
        
        cv_bridge = CvBridge()
        cv_frame = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv_frame = cv_frame [260:720, 680:1080]
        original = cv_frame.copy()
        
        # HSV filter application to original image
        hsv = cv2.cvtColor(cv_frame, cv2.COLOR_BGR2HSV)
        lower = np.array([0, 135, 135], dtype="uint8")
        upper = np.array([10, 255, 255], dtype="uint8")
        mask = cv2.inRange(hsv, lower, upper)
        
        #Contour finding and contour drawing on original image
        cnts, hirarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(original, cnts, -1, (0,255,0), 3)       
        
        #Calculate centroide co-ordinates of the area of interest and mark the co-ordinates in image
        if len(cnts)>0: 
            
            c = max(cnts, key=cv2.contourArea)
            M = cv2.moments(c)
            try:
                cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
                print("cx_r co-ordinate = ",cx,"cy_r co-ordinate = ", cy)
            except ZeroDivisionError:
                print("Zero division region")
            cv2.drawMarker(original, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 200, 1)
            if right_camera_mode == True:
                if cy <= 375 and 300<= cx <=400:    
                    self.CmdPubCallback(cx, cy)
                elif cy > 375 and cx > 300: 
                    right_camera_mode = False
                    left_camera_mode = True    
        else:
            cx = 0
            cy = 0
            if right_camera_mode == True:
                self.CmdPubCallback(cx, cy)

        cv2.imshow('original_right_head', original)
        if cv2.waitKey(1) == ord('q'):
            cv2.destroyAllWindows()
            
    def ImgCallback_LeftHead(self, msg):
        global left_camera_mode
        
        cv_bridge = CvBridge()
        cv_frame = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv_frame = cv_frame [260:720, 0:400]
        original = cv_frame.copy()
        
        # HSV filter application to original image
        hsv = cv2.cvtColor(cv_frame, cv2.COLOR_BGR2HSV)
        lower = np.array([0, 135, 135], dtype="uint8")
        upper = np.array([10, 255, 255], dtype="uint8")
        mask = cv2.inRange(hsv, lower, upper)
        
        #Contour finding and contour drawing on original image
        cnts, hirarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(original, cnts, -1, (0,255,0), 3)       
        
        #Calculate centroide co-ordinates of the area of interest and mark the co-ordinates in image
        if len(cnts)>0: 
            
            c = max(cnts, key=cv2.contourArea)
            M = cv2.moments(c)
            try:
                cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
                print("cx_l co-ordinate = ",cx,"cy_l co-ordinate = ", cy)
            except ZeroDivisionError:
                print("Zero division region")
            cv2.drawMarker(original, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 200, 1)
            if left_camera_mode == True:
                if cx > 225 and cy > 300 : 
                    self.CmdPubCallback(cx, cy)    
        else:
            cx = 0
            cy = 0
            if left_camera_mode == True:
                self.CmdPubCallback(cx, cy)
            
        cv2.imshow('original_left_head', original)
        if cv2.waitKey(1) == ord('q'):
            cv2.destroyAllWindows()
            
    def CmdPubCallback(self, x, y):
        
        global left_camera_mode, right_camera_mode
        cmd = Twist()
        
        if x == 0 and y == 0:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            print("Spot movement stopped")

        elif left_camera_mode == False and right_camera_mode == True:
            print("Spot heading straight to follow the ribbon") 
            cmd.linear.x = 0.5
            cmd.angular.z = 0.0

        elif left_camera_mode == True and right_camera_mode == False:
            error = x - 400/3
            print("Spot heading straight to follow the ribbon") 
            cmd.linear.x = 0.6
            cmd.angular.z = - error/100
            
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




