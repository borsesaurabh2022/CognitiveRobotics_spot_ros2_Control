import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Quaternion
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.msg import MotionPlanRequest
from moveit_msgs.msg import JointConstraint
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import PlanningOptions
from moveit_msgs.msg import RobotState
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest
from copy import deepcopy
from spot_interface.msg import TrigManireq, TrigManiresp
from webots_spot_msgs.srv import SpotMotion
import time
import numpy as np

global_joint_states = None
motion_plan_request_action_ready =  True
get_joint_angles_request_ready = True
previousstate_request = False
currentstate_request = False
compute_jointangle = False
initial_pose = False
tf_calculated  = False
gripping_completed = False
ungripping_completed = False
first_action =  False
second_action = False
counter = 0
bin_identifier = False

class MoveGroupActionClient(Node):
    def __init__(self):
        super().__init__('moveit_plan_execute_python')

        # declre parameters for tf2 ros function
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(JointState, '/joint_states', self.joint_states_cb, 10)
        self.create_subscription(TrigManireq, '/manipulator_req', self.ManipulatoreReq_cb,1)
        self.maniresp_publisher = self.create_publisher(TrigManiresp,'/manipulator_resp',0)
        self._action_client = ActionClient(self, MoveGroup, '/move_action')
        self.gripopen_client = self.create_client(SpotMotion, '/Spot/open_gripper')
        self.gripclose_client = self.create_client(SpotMotion, '/Spot/close_gripper')
        self.getjointangles_cli = self.create_client(GetPositionIK, '/compute_ik')
        while not self.getjointangles_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('joint agle finding service not available, waiting again...')
        # Call on_timer function every second
        self.timer = self.create_timer(1.0, self.transform_callback)

    def ManipulatoreReq_cb(self, data):
        global compute_jointangle, previousstate_request, initial_pose, counter, bin_identifier
        if not previousstate_request:
            if data.request:
                print("Request to execute manipulator action received")
                previousstate_request = True
                self.frame_id = data.objectid
                if self.frame_id == 'init_pose':
                    initial_pose = True
                    compute_jointangle = False
                    bin_identifier = False
                    self.send_goal()
                if self.frame_id in ["can1", "can2", "can3", "can4", "can5"]:
                    compute_jointangle = True
                    initial_pose = False
                    bin_identifier = False
                if self.frame_id in ["yellow_bin", "red_bin", "green_bin"]:
                    bin_identifier = True
                    compute_jointangle = False
                    initial_pose = False
                    self.send_goal()
        # elif previousstate_request:
        #     print("Action to pick/drop the object is already being executed")
        
    def joint_states_cb(self, joint_state):
        global global_joint_states
        global_joint_states = joint_state
        self.joint_state = joint_state

    def transform_callback(self):
        global compute_jointangle, motion_plan_request_action_ready, get_joint_angles_request_ready, tf_calculated
        global counter, first_action, second_action, bin_identifier
        if compute_jointangle:
            if motion_plan_request_action_ready:
                if get_joint_angles_request_ready:
                    if not tf_calculated:
                        if counter == 0:
                            first_action = True
                            self.obj_frame_id = self.frame_id+"_reftagpoint"
                        if counter == 1:
                            second_action = True
                            self.obj_frame_id = self.frame_id+"_psudocanmarker"
                        # if bin_identifier:
                        #     self.obj_frame_id = self.frame_id+"_psudobinmarker"
                        try:
                            t = self.tf_buffer.lookup_transform('UR3e',self.obj_frame_id, rclpy.time.Time()) 
                            print("transform:", t)
                            tf_calculated = True
                            self.GetJointangles(t) 
                        except TransformException as ex:
                            self.get_logger().info(f'Could not transform {self.frame_id} to UR3e: {ex}')
        #             else:
        #                 print("We are in thr else loop as the tf is already calculated")
        #         else:
        #             print("We are in else loop as the joint agle service is busy")
        #     else:
        #         print("we are else loop as the motion plan action service is busy")
        # else:
        #     print("here we are in else loop as compute jointangle request not received yet")
    
    def SendGripcloseReq(self):
        while not self.gripclose_client.wait_for_service(timeout_sec=1.0):
            print('service not available, trying again...')
        self.gripclose_request = SpotMotion.Request()
        self.gripclose_request.override = False
        print("sending a grip close request")
        self.gripclose_future = self.gripclose_client.call_async(self.gripclose_request)
        self.gripclose_future.add_done_callback(self.GripCloseSrv_cb)

    def GripCloseSrv_cb(self, gripclose_future):
        global initial_pose, gripping_completed, compute_jointangle
        self.gripclose_result = self.gripclose_future.result().answer
        if self.gripclose_result == 'closing gripper':
            initial_pose = True
            compute_jointangle = False
            gripping_completed = True
            self.send_goal()

    def SendGripopenReq(self):
        while not self.gripopen_client.wait_for_service(timeout_sec=1.0):
            print('service not available, trying again...')
        self.gripopen_request = SpotMotion.Request()
        self.gripopen_request.override = False
        print("sending a grip open request")
        self.gripopen_future = self.gripopen_client.call_async(self.gripopen_request)
        self.gripopen_future.add_done_callback(self.GripOpenSrv_cb)

    def GripOpenSrv_cb(self, gripopen_future):
        global initial_pose, ungripping_completed, compute_jointangle
        self.gripopen_result = self.gripopen_future.result().answer
        if self.gripopen_result == 'opening gripper':
            initial_pose = True
            compute_jointangle = False
            ungripping_completed = True
            self.send_goal()
    
    def SendManiresp(self, result, msg):
        self.mani_resp = TrigManiresp()
        self.mani_resp.result = result
        self.mani_resp.message = msg
        for i in range(10):
            self.maniresp_publisher.publish(self.mani_resp)
            time.sleep(0.1)

    def GetJointangles(self, t):
        global get_joint_angles_request_ready, global_joint_states
        try:
            get_joint_angles_request_ready = False
            get_joint_angle_request = PositionIKRequest()
            get_joint_angle_request.group_name = 'spot_ur3e_arm'
            get_joint_angle_request.avoid_collisions = True

            robot_state = RobotState()
            robot_state.joint_state = global_joint_states
            robot_state.is_diff = True
            get_joint_angle_request.robot_state = robot_state
         
            posestamped = PoseStamped()
            posestamped.header.stamp = self.get_clock().now().to_msg()
            posestamped.header.frame_id = 'UR3e'
            posestamped.pose.position.x = t.transform.translation.x
            posestamped.pose.position.y = t.transform.translation.y
            posestamped.pose.position.z = t.transform.translation.z
            posestamped.pose.orientation.x = t.transform.rotation.x
            posestamped.pose.orientation.y = t.transform.rotation.y
            posestamped.pose.orientation.z = t.transform.rotation.z
            posestamped.pose.orientation.w = t.transform.rotation.w
            get_joint_angle_request.pose_stamped = posestamped
            
           
            get_joint_angle_request.timeout.nanosec = 100

            get_poistionik = GetPositionIK.Request()
            get_poistionik.ik_request = get_joint_angle_request
            self.joint_angle_future = self.getjointangles_cli.call_async(get_poistionik)
            print("Joint angle calculation request sent")
            self.joint_angle_future.add_done_callback(self.joint_angle_calculation_cb)
        except Exception as e:
            print("Exception in Position IK claculation request:", e)

    def joint_angle_calculation_cb(self, joint_angle_future):
        global get_joint_angles_request_ready, tf_calculated, motion_plan_request_action_ready
        try:
            result1 = joint_angle_future.result()
            error_code = joint_angle_future.result().error_code.val
            self.joint_angle_results = result1.solution.joint_state.position
            print(error_code)
            print("joint angle result:", result1)
            print("joint angle result:", self.joint_angle_results)
            if (error_code == 1) and motion_plan_request_action_ready == True:
                print("we are here !!")
                self.send_goal()
            else:
                tf_calculated = False
                get_joint_angles_request_ready = True
        except Exception as e:
            print("There is exception in joint angle calculation:",e)

    def PlanningOptions(self):
        self.planning_option = PlanningOptions()
        self.planning_option.plan_only = False
        self.planning_option.look_around = False
        self.planning_option.look_around_attempts = 0
        self.planning_option.max_safe_execution_cost = 0.
        self.planning_option.replan = True
        self.planning_option.replan_attempts = 10
        self.planning_option.replan_delay = 0.1

    def MoveplanRequestmessage(self):
        global initial_pose, bin_identifier, compute_jointangle
        self.motion_plan_request = MotionPlanRequest()
        self.motion_plan_request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        self.motion_plan_request.workspace_parameters.header.frame_id = 'base_link'
        self.motion_plan_request.workspace_parameters.min_corner.x = -1.0
        self.motion_plan_request.workspace_parameters.min_corner.y = -1.0
        self.motion_plan_request.workspace_parameters.min_corner.z = -1.0
        self.motion_plan_request.workspace_parameters.max_corner.x = 1.0
        self.motion_plan_request.workspace_parameters.max_corner.y = 1.0
        self.motion_plan_request.workspace_parameters.max_corner.z = 1.0
        self.motion_plan_request.start_state.is_diff = True
        self.motion_plan_request.start_state.joint_state = self.joint_state

        jc = JointConstraint()
        jc.tolerance_above = 0.0001
        jc.tolerance_below = 0.0001
        jc.weight = 1.0

        if initial_pose:
            joints = {}
            joints['shoulder_pan_joint'] = -1.85
            joints['shoulder_lift_joint'] = 0.2
            joints['elbow_joint'] = -2.6
            joints['wrist_1_joint'] = -0.7512
            joints['wrist_2_joint'] = 1.5708
            joints['wrist_3_joint'] = 0.0

        elif bin_identifier:
            joints = {}
            joints['shoulder_pan_joint'] = np.deg2rad(58)
            joints['shoulder_lift_joint'] = np.deg2rad(0)
            joints['elbow_joint'] = np.deg2rad(-10)
            joints['wrist_1_joint'] = np.deg2rad(-169)
            joints['wrist_2_joint'] = np.deg2rad(-83)
            joints['wrist_3_joint'] = np.deg2rad(-179)

        elif compute_jointangle:
            joints = {}
            joints['shoulder_pan_joint'] = self.joint_angle_results[0]
            joints['shoulder_lift_joint'] = self.joint_angle_results[1]
            joints['elbow_joint'] = self.joint_angle_results[2]
            joints['wrist_1_joint'] = self.joint_angle_results[3]
            joints['wrist_2_joint'] = self.joint_angle_results[4]
            joints['wrist_3_joint'] = self.joint_angle_results[5]

        constraints = Constraints()
        for (joint, angle) in joints.items():
            jc.joint_name = joint
            jc.position = angle
            constraints.joint_constraints.append(deepcopy(jc))

        self.motion_plan_request.goal_constraints.append(constraints)
        self.motion_plan_request.pipeline_id = 'move_group'
        self.motion_plan_request.group_name = 'spot_ur3e_arm'
        self.motion_plan_request.num_planning_attempts = 10
        self.motion_plan_request.allowed_planning_time = 5.0
        self.motion_plan_request.max_velocity_scaling_factor = 0.1
        self.motion_plan_request.max_acceleration_scaling_factor = 0.1
        self.motion_plan_request.max_cartesian_speed = 0.0

    def send_goal(self):
        global motion_plan_request_action_ready
        try:
            motion_plan_request_action_ready = False
            self.MoveplanRequestmessage()
            self.PlanningOptions()
            goal_msg = MoveGroup.Goal()
            goal_msg.request = self.motion_plan_request
            goal_msg.planning_options = self.planning_option
            self._action_client.wait_for_server()
            self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
            self._send_goal_future.add_done_callback(self.goal_response_callback)
        except Exception as e:
            print("Exception in sending thr goal to motion plan request", e)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        global motion_plan_request_action_ready, get_joint_angles_request_ready, tf_calculated, gripping_completed
        global compute_jointangle, previousstate_request, bin_identifier, ungripping_completed, initial_pose, counter
        global first_action, second_action
        status = future.result().status
        print("status", status)
        if status == 4:
            print("reached goal")
            if initial_pose:
                if gripping_completed == True and counter == 2:
                    motion_plan_request_action_ready = True
                    get_joint_angles_request_ready = True
                    self.SendManiresp(True, self.frame_id+"_gripped")
                    counter = 0 
                    initial_pose = False
                    compute_jointangle = False
                    tf_calculated = False
                    previousstate_request = False
                elif ungripping_completed == True and bin_identifier:
                    self.SendManiresp(True, self.frame_id+"_ungripped")
                    counter = 0 
                    initial_pose = False
                    compute_jointangle = False
                    tf_calculated = False
                    previousstate_request = False
                    bin_identifier = False
                    motion_plan_request_action_ready = True
                    get_joint_angles_request_ready = True
                else:
                    initial_pose = False
                    motion_plan_request_action_ready = True
                    get_joint_angles_request_ready = True
                    previousstate_request = False
                    self.SendManiresp(True,"initial_pose_set")
            elif compute_jointangle == True:
                if counter == 0 and first_action == True:
                    counter = counter +1
                    first_action = False
                    motion_plan_request_action_ready = True
                    get_joint_angles_request_ready = True
                    tf_calculated = False
                elif counter == 1 and second_action == True:
                    counter = counter +1
                    second_action =  False
                    motion_plan_request_action_ready = True
                    get_joint_angles_request_ready = True
                    self.SendGripcloseReq()
            elif bin_identifier:
                self.SendGripopenReq()
        elif status == 6:
            if gripping_completed:
                initial_pose = True
                compute_jointangle = False
                self.send_goal()


    def feedback_callback(self, feedback_msg):
        self.get_logger().info(str(feedback_msg))

def main(args=None):
    rclpy.init(args=args)
    action_client = MoveGroupActionClient()
    while global_joint_states is None:
        rclpy.spin_once(action_client)
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()