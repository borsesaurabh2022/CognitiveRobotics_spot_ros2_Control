#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from webots_spot_msgs.srv import SpotMotion
from sensor_msgs.msg import Range
from spot_interface.srv import SpotStop

spot_standup = True 
spot_sitdown = False
prev_range = 0.0
spot_stopsrvcall = False
spot_resumesrv = True

class SpotsensorClass(Node):

    def __init__(self):
        super().__init__('spot_range_sensor')
        self.subscription = self.create_subscription(Range, '/Spot/distance_sensor', self.SensorCallback, 10)
        self.publisher= self.create_publisher(Twist, '/cmd_vel', 10)
        self.spotstop_service = self.create_client(SpotStop, '/Spot/stop')
        self.prev_range = 0.0

    # callback function to publish data over cmd_vel topic based on joy node activation
    def SensorCallback(self, data):
        global spot_stopsrvcall, spot_resumesrv, prev_range
        current_range = data.range
        if spot_stopsrvcall == False and spot_resumesrv == True:
            if current_range < 1.0 and prev_range >1.0:
                self.StopServiceCallback(True)
                spot_stopsrvcall = True
                spot_resumesrv = False
            prev_range = current_range
        elif spot_stopsrvcall == True and spot_resumesrv == False:
            if current_range > 1.0 and prev_range <1.0:    
                self.StopServiceCallback(False)
                spot_resumesrv = True
                spot_stopsrvcall = False
            prev_range = current_range
        
    # spot stop and resume service callback function          
    def StopServiceCallback(self,state):
        while not self.spotstop_service.wait_for_service(timeout_sec=1.0):
            print("Service not available, trying again...")
        stop_srv = SpotStop.Request()
        stop_srv.data = state
        stop_srv_future = self.spotstop_service.call_async(stop_srv)
        stop_srv_future.add_done_callback(self.spotstop_srv_callback)
        
    def spotstop_srv_callback(self, future):
        global spot_stopsrvcall
        stop_srv_result = future.result()
        print('result:', stop_srv_result.message)

def main():
    rclpy.init()
    subscribernode = SpotsensorClass()
    try:
        rclpy.spin(subscribernode)
    except KeyboardInterrupt:
        pass
    subscribernode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
