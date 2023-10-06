import math

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from turtlesim.srv import Spawn
from spot_interface.srv import CanTransform
from tf2_ros import TransformBroadcaster
import numpy as np
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion


init_tfbroadcast = False

class TfListener(Node):

    def __init__(self):
        super().__init__('spotworld_cantflistener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.tf_srvser = self.create_service(CanTransform, '/tf_request', self.tf_requestcallback)
        self.timer = self.create_timer(0.1, self.tf_timercallback)

    def tf_requestcallback(self, request, response):
        global init_tfbroadcast
        if request.request == True:
            self.target_frame = request.obj_id
            init_tfbroadcast = True
            response.response = True
            return response
            
    def tf_timercallback(self):
        global init_tfbroadcast
        if init_tfbroadcast: 
            self.source_frame = 'map'
            try:
                t = self.tf_buffer.lookup_transform(self.source_frame,self.target_frame,rclpy.time.Time())
            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform {self.target_frame} to {self.source_frame}: {ex}')
                return
            pos = t.transform.translation
            rot = t.transform.rotation
            self.tf_broadcast(pos,rot)
    
    def tf_broadcast(self, pos, rot):
        t1 = TransformStamped()
        t1.header.stamp = self.get_clock().now().to_msg()
        t1.header.frame_id = 'map'
        t1.child_frame_id = self.target_frame+'_psudomrk_spot'
        t1.transform.translation.x = pos.x
        t1.transform.translation.y = pos.y -1.1 
        t1.transform.translation.z = pos.z
        q_rot = self.quaternion_from_euler(0, 0, 3.14159/2)
        t1.transform.rotation = q_rot
        self.tf_broadcaster.sendTransform(t1)
        
        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = 'map'
        t2.child_frame_id = self.target_frame+'_psudomrk_gripper'
        t2.transform.translation.x = pos.x
        t2.transform.translation.y = pos.y -0.25
        t2.transform.translation.z = pos.z
        t2.transform.rotation = rot
        self.tf_broadcaster.sendTransform(t2)
        
    def quaternion_from_euler(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        quat = Quaternion()
        quat.w = qw
        quat.x = qx
        quat.y = qy
        quat.z = qz
        return quat  
 
    def euler_from_quaternion(self,w, x, y, z):
        ysqr = y * y
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + ysqr)
        X = math.degrees(math.atan2(t0, t1))
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.degrees(math.asin(t2))
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        Z = math.degrees(math.atan2(t3, t4))
        return X, Y, Z   
    
def main():
    rclpy.init()
    node = TfListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()