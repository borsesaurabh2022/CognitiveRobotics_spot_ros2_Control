#!/usr/bin/env python

from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Point, Quaternion
from rclpy.node import Node
import rclpy
import time
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped 
from spot_interface.srv import CanTransform
from spot_interface.msg import TrigManireq, TrigManiresp

tfbroadcast_stopped =  False
tfbroadcast_started = False
tf_received = False
response_state = False
compute_transformation = False

list_targetgoals = ["can4", "yellow_bin", "can3", "yellow_bin", "can2", "yellow_bin", "can5", "yellow_bin", "can1", "yellow_bin"]
list_goalsachived = []

class AutonomousNavigation(Node):

    def __init__(self):
        super().__init__('Spot_Autonavigation')
        global manipose_initialized, compute_transformation, tf_received
        
        self.tf_Buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_Buffer, self)
        self.tf_timer  = self.create_timer(0.25, self.TransformationCallback)
        self.create_subscription(TrigManiresp, '/manipulator_resp', self.ManiResponseCallback, 0)
        self.trigmani_publisher =  self.create_publisher(TrigManireq, '/manipulator_req', 0)
        self.tf_srvcli = self.create_client(CanTransform, '/tf_request') 
        self.navtopose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.SendManipulatorRequest(True, "init_pose")
        self.goal_index = 0
        self.current_goal = list_targetgoals[self.goal_index]
        
    def send_tfrequest(self, tf_goal):
        while not self.tf_srvcli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')  
        self.tfreq = CanTransform.Request()
        self.tfreq.request = True
        self.tfreq.obj_id = tf_goal
        self.tf_future = self.tf_srvcli.call_async(self.tfreq)
        self.tf_future.add_done_callback(self.tfsrvfuture_callback)
        
    def tfsrvfuture_callback(self, future):
        global tfbroadcast_started
        self.tfsrv_result = future.result().response
        print('result:', self.tfsrv_result)
        if self.tfsrv_result:
            print('tf_broadcast successful, proceed for autonav')
            tfbroadcast_started = True

    def ManiResponseCallback(self, data):
        global manipose_initialized, compute_transformation, list_goalsachived, list_targetgoals, tfbroadcast_stopped, response_state
        if not response_state:
            if data.result:
                print(data.message)
                if data.message == "initial_pose_set":
                    response_state = True
                    manipose_initialized = True
                    self.send_tfrequest(self.current_goal)
                    compute_transformation = True
                if data.message == self.current_goal+"_gripped":
                    response_state = True
                    list_goalsachived.append(self.current_goal)
                    self.goal_index = self.goal_index + 1
                    self.next_goal = list_targetgoals[self.goal_index]
                    self.send_tfrequest(self.next_goal)
                    compute_transformation = True
                if data.message == self.current_goal+"_ungripped":
                    response_state = True
                    list_goalsachived.append(self.current_goal)
                    self.goal_index = self.goal_index + 1
                    self.next_goal = list_targetgoals[self.goal_index]
                    self.send_tfrequest(self.next_goal)
                    compute_transformation = True
            
    def SendManipulatorRequest(self, request, objectid):
        self.mani_request = TrigManireq()
        self.mani_request.request = request
        self.mani_request.objectid = objectid
        for i in range(15):
            self.trigmani_publisher.publish(self.mani_request)
            time.sleep(0.1)
            print("publishing the goal for manipulator")

    def TransformationCallback(self,):
        global tf_received, tfbroadcast_started, compute_transformation
        if compute_transformation:
            if tfbroadcast_started:
                if not tf_received:
                    try:
                        tf_transform = self.tf_Buffer.lookup_transform("map",self.current_goal+"_psudomrk_spot",rclpy.time.Time()) 
                        print("transform:", tf_transform)
                        tf_received = True
                        tf_response_pos = tf_transform.transform.translation
                        tf_response_rot = tf_transform.transform.rotation
                        self.ExecuteAutoNavigation(tf_response_pos, tf_response_rot)
                    except:
                        print("Overall lookup transfrom failed")

    def ExecuteAutoNavigation(self, tf_position, tf_rotation):
        while not self.navtopose_client.wait_for_server(timeout_sec=2.0):
            print("Server still not available; waiting...")
        result = self.generatePositionandOrientation(tf_position, tf_rotation)
        position_ = result[0]
        orientation_= result[1]
        self.SendAutonavGoal(position_, orientation_)

    def SendAutonavGoal(self, position, orientation):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position = position
        goal.pose.pose.orientation = orientation
        print("Received new goal => X: " + str(goal.pose.pose.position.x) + " Y: " + str(goal.pose.pose.position.y))
        self.send_goal_future = self.navtopose_client.send_goal_async(goal)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, send_goal_future):
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, get_result_future):
        global tf_received, tfbroadcast_started, compute_transformation, response_state
        status = get_result_future.result().status
        print("status", status)
        if status == GoalStatus.STATUS_SUCCEEDED:
            print("Reached to the goal position")
            self.SendManipulatorRequest(True, self.current_goal)
            compute_transformation = False
            #tfbroadcast_started= False
            tf_received = False
            response_state = False
        elif status == 6:
            print("Retrying navigation to the goal position")
            tf_received = False

    def generatePositionandOrientation(self, pose, rot):
        self.position = Point()
        self.quat = Quaternion()
        self.position.x = pose.x
        self.position.y = pose.y
        self.position.z = pose.z
        self.quat.x = rot.x
        self.quat.y = rot.y
        self.quat.z = rot.z
        self.quat.w = rot.w
        print("Positon",self.position)
        print("Qaternion",self.quat)
        return [self.position, self.quat]

def main(args=None):
  rclpy.init(args=args)
  auto_navigate  = AutonomousNavigation()
  try:
    rclpy.spin(auto_navigate)
  except KeyboardInterrupt:
      pass
  auto_navigate.destroy_node()
  rclpy.shutdown()
  rclpy.spin(auto_navigate)

if __name__ == '__main__':
    main()
