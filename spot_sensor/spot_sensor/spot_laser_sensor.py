#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from spot_interface.srv import SpotStop
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy, QoSLivelinessPolicy

spot_stopsrvcall = False
spot_resumesrv = True
counter = 0
prev_scan = LaserScan()

class SpotsensorClass(Node):

    def __init__(self):
        super().__init__('spot_range_sensor')
        #making a separate QOS Profile to get LaserScan msgs.
        self.Qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                durability=QoSDurabilityPolicy.VOLATILE,
                liveliness=QoSLivelinessPolicy.AUTOMATIC,
                depth=1
            )
        self.subscription = self.create_subscription(LaserScan, '/scan', self.SensorCallback, qos_profile=self.Qos_profile )
        self.publisher = self.create_publisher(LaserScan, '/near_field_scan',10 )
        self.spotstop_service = self.create_client(SpotStop, '/Spot/stop')

    # callback function to publish data over cmd_vel topic based on joy node activation
    def SensorCallback(self, msg):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = msg.header.frame_id
        scan.angle_min = (msg.angle_min)
        scan.angle_max = (msg.angle_max)
        scan.angle_increment = msg.angle_increment
        scan.time_increment = msg.time_increment
        scan.scan_time = msg.scan_time
        scan.range_min = msg.range_min -0.35
        scan.range_max = (msg.range_max -98.5)
        scan.ranges = msg.ranges
        var_len = len(scan.ranges)
        for i in range(var_len):
            if i < 320 or i > 400:
                scan.ranges[i] = 0.0
        self.publisher.publish(scan)
        global spot_stopsrvcall, spot_resumesrv, counter, prev_scan
      
        if spot_stopsrvcall == False and spot_resumesrv == True:
            for i in range(var_len):
                if 0.25 < scan.ranges[i] < 1.0:
                    self.StopServiceCallback(True)
                    spot_stopsrvcall = True
                    spot_resumesrv = False
                    break
            prev_scan = scan
        temp_len = len(scan.ranges)
        
        if spot_stopsrvcall == True and spot_resumesrv == False:
            for i in range(temp_len):
                if scan.ranges[i] > 1.0:
                    counter = counter+1
                    print(counter)
                    if counter == 81:
                        self.StopServiceCallback(False)
                        spot_resumesrv = True
                        spot_stopsrvcall = False     
                        counter = 0  
                        break
            prev_scan = scan

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
