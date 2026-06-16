import rclpy
from rclpy.node import Node
from .servo_control import *
from sensor_msgs.msg import JointState
from robot_interface.srv import GetAngleList, SetAngle, Home, Gripper
from trajectory_msgs.msg import JointTrajectory
import math

class HardwareNode(Node):
    def __init__(self):
        super().__init__("hardware_node")
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.1, self.publisher_joints)

        self.servo_control = ServoControl()

        self.srv_get_angle = self.create_service(GetAngleList, '/get_joint_angle', self.get_joint_angles_callback)
        self.srv_set_angle = self.create_service(SetAngle, '/set_angle', self.set_joint_angle_callback)
        self.srv_home = self.create_service(Home, '/home', self.home_pose)
        self.srv_gripper = self.create_service(Gripper, '/grip', self.grip)

        self.subscriber = self.create_subscription(JointTrajectory, '/joint_trajectory', self.joint_trajectory_callback, 10)
        
    def publisher_joints(self):
        msg = JointState()

        #the joint names
        msg.name = ['joint_0', 'joint_1', 'joint_2', 'joint_3', 'joint_4',
            'left_gripper_joint', 'right_gripper_joint']

        self.servo_control.update_joint_angles()


        msg.position = self.servo_control.get_joint_angles()
        #self.get_logger().info(f"pos : {msg.position}")
        msg.position.append(self.servo_control.get_gripper_angle())
        msg.position.append(self.servo_control.get_gripper_angle())


        #convert angle to radians
        for i in range(7):

            msg.position[i] = msg.position[i]*math.pi/180 
            #self.get_logger().info(f"pos{i} is {msg.position[i]}")

        #add time stamp
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)

    def get_joint_angles_callback(self, request, response):
        response.current_angles = self.servo_control.get_joint_angles()
        return response

    def set_joint_angle_callback(self, request, response):
        response.success = self.servo_control.set_joint_angle(request.index, request.angle)

        return response
    
    def home_pose(self, request, response):
        self.servo_control.set_joint_angles_list(HOME)
        return response

    def grip(self, request, response):
        self.servo_control.set_gripper(request.grip)
        return response

    def joint_trajectory_callback(self, msg):
        for point in msg.points:
            deg_pos = [math.degrees(p) for p in point.positions]
            self.servo_control.interval_control(deg_pos)

            #self.get_clock().sleep_for(point.time_from_start.to_sec())

        
    

def main(args=None):
    rclpy.init(args=args)
    hardware_node = HardwareNode()
    rclpy.spin(hardware_node)
    hardware_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
