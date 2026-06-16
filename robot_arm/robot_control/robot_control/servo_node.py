import rclpy
import math
import threading

from rclpy.node import Node
from .servo_control import *
from sensor_msgs.msg import JointState
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.executors import MultiThreadedExecutor
from robot_interface.srv import GetAngleList, Gripper, Home


class ServoNode(Node):
    def __init__(self):
        super().__init__("servo_node")
        self.servo_control = ServoControl()

        self.goal_handle = None
        self.goal_lock = threading.Lock()

        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.1, self.publisher_joints)

        self._action_server = ActionServer(self, FollowJointTrajectory, 
                                           'arm_controller/follow_joint_trajectory', 
                                           self.execute_trajectory)

        self._action_server = ActionServer(self, GripperCommand, 
                                           'gripper/gripper_cmd', 
                                           self.execute_gripper)
        
        self.srv_get_angle = self.create_service(GetAngleList, '/get_joint_angle', self.get_joint_angles_callback)

        self.srv_home = self.create_service(Home, '/home', self.home_pose)
        self.srv_gripper = self.create_service(Gripper, '/grip', self.grip)

        self.joint_names = self.servo_control.get_joint_names()
        self.joint_names.append("left_gripper_joint")
        #self.joint_names.append("right_gripper_joint")


    def publisher_joints(self):
        msg = JointState()
		
        msg.name = self.joint_names
        #the joint names

        msg.position = self.servo_control.get_joint_angles()
        msg.position.append(self.servo_control.get_gripper_angle())

        msg.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        #self.get_logger().info(f"publish for")

        for i in range(len(msg.position)):
            msg.position[i] = msg.position[i]*math.pi/180 
            #self.get_logger().info(f"pos is {msg.position}")

        #add time stamp
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)

    def execute_trajectory(self, goal_handle):
        trajectory = goal_handle.request.trajectory

        feedback_msg = FollowJointTrajectory.Feedback()

        #check all poistions are valid
        for point in trajectory.points:
            if not self.servo_control.angle_is_valid_list(self.rad_to_deg(point)):
                goal_handle.abort()
                self.get_logger().info(f"invalid position")

                return FollowJointTrajectory.Result()

        #inizialise values
        previous_time = 0.0
        target_time_sec = 0.0
        dt = 0.025
 
        prev_pos = self.servo_control.get_joint_angles()
        prev_velocity = [0, 0, 0, 0, 0]
        all_points = []

        # Send each point to ServoControl
        for i, point in enumerate(trajectory.points):
            target_time_sec = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9

            duration_s = target_time_sec - previous_time
            #duration_ms = int(duration_s*1000)

            #convert to degrees
            next_angles = [round(math.degrees(a), 1) for a in point.positions]
            next_velocity = [math.degrees(a) for a in point.velocities]
            #self.get_logger().info(f"angle: {next_angles}")
            #self.get_logger().info(f"time {target_time_sec}")

            #dt = duration_s            
            steps = max(int(duration_s/dt),1)
            if next_angles != prev_pos:
                for step in range(steps):
                    start_time = time.time()
                    t = (step+1)/steps
                    
                    prev = self.hermite_spline(prev_pos, next_angles, prev_velocity, next_velocity, t, dt*steps)
                    #if not all_points or prev != all_points[-1]: 
                    all_points.append(prev)
            prev_pos = next_angles
            prev_velocity = next_velocity
            previous_time = target_time_sec
	
        #for i, points in enumerate(all_points):
        for i, points in enumerate(all_points):	
            #start time
            start_time = time.monotonic()

            self.servo_control.moveit_control(points, interval = 0)

            if i%5 == 0 or i == len(all_points)-1:
                current_joints = self.servo_control.get_joint_angles()
                self.publisher_joints()
                
                #publish feedback
                feedback_msg.joint_names = trajectory.joint_names
                feedback_msg.actual.positions = [math.radians(a) for a in current_joints]# Current waypoint
                feedback_msg.header.stamp = self.get_clock().now().to_msg()
                goal_handle.publish_feedback(feedback_msg)   
			
            self.get_logger().info(f"angle {points}") 
			
            #time to publish and send commands
            end_time = time.monotonic()
            #self.get_logger().info(f"sleep {dt  - (end_time - start_time)} dt {dt} sleep {end_time - start_time}")
            sleep = end_time - start_time
            
            #time.sleep(sleep)

        goal_handle.succeed()
        return FollowJointTrajectory.Result()


    def execute_gripper(self, goal_handle):
        gripper_cmd = goal_handle.request.command

        #check if not null
        if gripper_cmd == None:
            self.get_logger().info(f"gripper empty: {gripper_cmd}")

            goal_handle.abort()
            return pos.Result()

        self.get_logger().info(f"gripper{gripper_cmd}")
        grip = 0

        #open
        if gripper_cmd.position >= 1.57:
            pos  = 0
        #clode
        elif gripper_cmd.position <= .2:
            pos = 1
        
        self.servo_control.set_gripper(pos)

        result = GripperCommand.Result()
        goal_handle.succeed()
        return result
        

    def get_joint_angles_callback(self, request, response):
        response.current_angles = self.servo_control.get_joint_angles()
        return response
    
    def rad_to_deg(self, points):
        return [math.degrees(a) for a in points.positions]

    def hermite_spline(self, current_angles, target_angles, current_velocity, target_velocity, t, duration):
        h00 = 2*t**3-3*t**2+1
        h10 = t**3-2*t**2 +t
        h01 = -2*t**3+3*t**2
        h11 = t**3-t**2

        pos = []

        for p0, p1, v0, v1 in zip(current_angles, target_angles, current_velocity, target_velocity):
            pos.append(round(h00*p0 +h10*v0*duration + h01*p1 + h11*v1*duration,1))

        return pos

    def home_pose(self, request, response):
        response.success = self.servo_control.set_joint_angles_list(HOME)
        return response

    def grip(self, request, response):
        #self.get_logger().info("grip")
        response.success = self.servo_control.set_gripper(request.grip)
        return response

def main(args=None):
    rclpy.init(args=args)
    servo_node = ServoNode()
    #multithreading
    rclpy.spin(servo_node)
    servo_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
