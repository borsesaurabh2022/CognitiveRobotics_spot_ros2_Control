#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from webots_spot_msgs.srv import SpotMotion
from spot_interface.srv import SpotStop

spot_standup = True 
spot_sitdown = False
spot_stop = False

class SpotteleopClass(Node):

    def __init__(self):
        super().__init__('tb3_teleop')
        self.subscription = self.create_subscription(Joy, '/joy', self.MotionCallback, 10)
        self.publisher= self.create_publisher(Twist, '/cmd_vel', 10)
        self.spotsitdown_service = self.create_client(SpotMotion, '/Spot/sit_down')
        self.spotstandup_service = self.create_client(SpotMotion, '/Spot/stand_up')
        self.spotstop_service = self.create_service(SpotStop, '/Spot/stop', self.SpotStopsrvCallback)
        self.declare_parameter('forward_axis_scalling', 1.0)
        self.declare_parameter('angular_axis_scalling', 1.0)

    # callback function to publish data over cmd_vel topic based on joy node activation
    def MotionCallback(self, data):
        global spot_standup, spot_sitdown, spot_stop
        if spot_standup:
            if not spot_sitdown:
                f_scale = self.get_parameter('forward_axis_scalling').value
                z_scale = self.get_parameter('forward_axis_scalling').value
                p_msg = Twist()
                if not spot_stop:
                    p_msg.linear.x = f_scale*data.axes[1]
                    p_msg.linear.y = 0.0
                    p_msg.linear.z = 0.0
                else:
                    p_msg.linear.x = 0.0
                    p_msg.linear.y = 0.0
                    p_msg.linear.z = 0.0
                p_msg.angular.x = 0.0
                p_msg.angular.y = 0.0
                p_msg.angular.z = z_scale*data.axes[3]
                if data.buttons[0] == 1:
                    print('please wait, spot sitdown service is being executed !')
                    self.SitdownSrvCallback()
                    spot_sitdown = True
                    spot_standup = False
                self.publisher.publish(p_msg)

        elif spot_sitdown:
            if not spot_standup:                
                if data.buttons[3] == 1:
                    print('please wait, spot standup service is being executed !')
                    self.StandupSrvCallback()
                    spot_standup = True
                    spot_sitdown = False

    # spot stop telop service callback function
    def SpotStopsrvCallback(self, request, response):
        global spot_stop 
        if request.data:
            spot_stop = True
            response.success = True
            response.message = 'Spot motion stop service called by the client and succeded'
        else:
            spot_stop = False
            response.success = True
            response.message = 'Spot motion resume service called by the client and succeded'
        return response

    # spot sit_down service callback function          
    def SitdownSrvCallback(self):
        while not self.spotsitdown_service.wait_for_service(timeout_sec=1.0):
            print("Service not available, trying again...")
        sitdown_srv = SpotMotion.Request()
        sitdown_srv.override = True
        sitdown_srv_future = self.spotsitdown_service.call_async(sitdown_srv)
        sitdown_srv_future.add_done_callback(self.sitdown_srv_callback)
        
    def sitdown_srv_callback(self, future):
        sitdown_srv_result = future.result().answer
        print('result:', sitdown_srv_result)

    # spot stand_up service callback function  
    def StandupSrvCallback(self):
        while not self.spotstandup_service.wait_for_service(timeout_sec=1.0):
            print("Service not available, trying again...")
        standup_srv = SpotMotion.Request()
        standup_srv.override = True
        standup_srv_future = self.spotstandup_service.call_async(standup_srv)
        standup_srv_future.add_done_callback(self.standup_srv_callback)

    def standup_srv_callback(self, future):
        standup_srv_result = future.result().answer
        print('result:', standup_srv_result)

def main():
    rclpy.init()
    subscribernode = SpotteleopClass()
    try:
        rclpy.spin(subscribernode)
    except KeyboardInterrupt:
        pass
    subscribernode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
