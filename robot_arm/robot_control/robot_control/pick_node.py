#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from isaac_ros_moveit_goal_setter.move_group_client import MoveGroupClient
from moveit_msgs.msg import MoveItErrorCodes

from robot_interface.srv import Gripper


class PickNode(Node):
    def __init__(self):
        super().__init__('pick_node')
       
        self._world_frame = self.declare_parameter(
            'world_frame', 'arm_base_link').get_parameter_value().string_value

        self._target_frames = self.declare_parameter(
            'target_frames', ['fp_object']).get_parameter_value().string_array_value

        self._target_frame_idx = 0

        self._plan_timer_period = self.declare_parameter(
            'plan_timer_period', 2).get_parameter_value().double_value

        planner_group_name = self.declare_parameter(
            'planner_group_name', 'arm').get_parameter_value().string_value

        pipeline_id = self.declare_parameter(
            'pipeline_id', 'isaac_ros_cumotion').get_parameter_value().string_value

        planner_id = self.declare_parameter(
            'planner_id', 'cuMotion').get_parameter_value().string_value
 
        ee_link = self.declare_parameter(
            'end_effector_link', 'ee_link').get_parameter_value().string_value

        self.move_group_client = MoveGroupClient(
            self, planner_group_name, pipeline_id, planner_id, ee_link)

        self.move_group_client.max_velocity_scaling_factor = 0.1
        self.move_group_client.max_acceleration_scaling_factor = 0.1

        self.sub = self.create_subscription(
            Detection3DArray,
            '/output',
            self.detection_callback,
            10
        )
 
        self.busy = False

        self._tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=60.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)


        self.gripper_client = self.create_client(Gripper, '/grip')

        self.get_logger().info('pick_node started, waiting for FoundationPose /output')

    def detection_callback(self, msg: Detection3DArray):
        if self.busy:
            return

        if len(msg.detections) == 0:
            return

        self.busy = True

        try:
            detection = msg.detections[0]

            if len(detection.results) == 0:
                self.get_logger().warn('Detection has no pose result')
                self.busy = False
                return

            transform = self._tf_buffer.lookup_transform(
                self._world_frame,
                "fp_object",
                rclpy.time.Time()
            )

            object_pose = PoseStamped()
            object_pose.header.frame_id = self._world_frame

            object_pose.pose.position.x = transform.transform.translation.x
            object_pose.pose.position.y = transform.transform.translation.y
            object_pose.pose.position.z = transform.transform.translation.z

            self.get_logger().info(
                f'Object in base frame: '
                f'x={object_pose.pose.position.x:.3f}, '
                f'y={object_pose.pose.position.y:.3f}, '
                f'z={object_pose.pose.position.z:.3f}'
            )

            self.pick_object(object_pose)

        except Exception as e:
            self.get_logger().error(f'Pick failed: {e}')

        self.busy = False

    def pick_object(self, object_pose: PoseStamped):
        x = object_pose.pose.position.x
        y = 0.00#object_pose.pose.position.y 
        z = 0.0#object_pose.pose.position.z -0.088

        approach_pose = self.make_pose(x, y, z + 0.20)
        pick_pose = self.make_pose(x, y, z + 0.1)
        retreat_pose = self.make_pose(x, y, z + 0.15)

        #open gripper
        self.gripper_control(0)

        self.get_logger().info('Moving to approach pose')
        self.move_to_pose(approach_pose)

        self.get_logger().info('Moving to pick pose')
        self.move_to_pose(pick_pose)

        #close gripper
        self.gripper_control(1)

        self.get_logger().info('Retreating')
        self.move_to_pose(retreat_pose)

        self.get_logger().info('Pick sequence complete')

    def make_pose(self, x, y, z):
        pose = PoseStamped()
        pose.header.frame_id = self._world_frame

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        # Fixed gripper-down orientation. Tune if needed.
        pose.pose.orientation.x = 0.917#0.894
        pose.pose.orientation.y = 0.00#0.014
        pose.pose.orientation.z = 0.399#0.447
        pose.pose.orientation.w = 0.00#-0.002

        return pose

    def move_to_pose(self, pose: PoseStamped):
        
        response = self.move_group_client.send_goal_pose(pose)
        
        if response.error_code.val == MoveItErrorCodes.SUCCESS:
            self._target_frame_idx = (self._target_frame_idx + 1) % len(self._target_frames)
        else:
            self.get_logger().warning('target pose was not reachable by planner, trying again \
                                      on the next iteration')


    def gripper_control(self, grip):
        self.get_logger().info('gripper placeholder')

        req = Gripper.Request()
        req.grip = grip

        future = self.gripper_client.call_async(req)
        future.add_done_callback(self.gripper_response_callback)

    def gripper_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Gripper success: {response.success}')
        except Exception as e:
            self.get_logger().error(f'Gripper failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = PickNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
