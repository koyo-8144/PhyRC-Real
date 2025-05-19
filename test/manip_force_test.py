#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState, CameraInfo, Image, PointCloud2
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import cv2
from sensor_msgs import point_cloud2
# import pcl
import open3d as o3d
import numpy as np
import sys
import tf
from std_msgs.msg import Bool
import time
from std_msgs.msg import Int64

class StretchController:
    def __init__(self):
        rospy.init_node('stretch_arm_controller', anonymous=True)

        self.bridge = CvBridge()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        self.intrinsics = None  # Store camera intrinsics here
        self.rate = rospy.Rate(0.1)  # 1 Hz (1 message per second)

        # Publishers
        self.arm_command_pub = rospy.Publisher('/stretch_arm_controller/command', JointTrajectory, queue_size=10)
        self.base_command_pub = rospy.Publisher('/stretch_diff_drive_controller/cmd_vel', Twist, queue_size=10)
        self.gripper_command_pub = rospy.Publisher('/stretch_gripper_controller/command', JointTrajectory, queue_size=10)
        self.head_command_pub = rospy.Publisher('/stretch_head_controller/command', JointTrajectory, queue_size=10)

        # Subscribers
        rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)
        rospy.Subscriber('/force_sensor', Int64, self.force_callback)



        self.current_joint_lift = 0.5
        self.force_flag = False
        self.initial_force_value = 68
        self.force_value = self.initial_force_value


    ##### callbacks #####
    def joint_states_callback(self, data):
        name = data.name
        position = data.position
        velocity = data.velocity
        effort = data.effort

        # print("=== Joint States ===")
        # print(f"Joint Name: {name}")
        # print(f"  Position: {position:}")
        # print(f"  Velocity: {velocity:}")
        # print(f"  Effort: {effort:}")

        self.current_joint_lift = position[9]

    def force_callback(self, data):
        # print("force data: ", data)
        self.force_value = data.data
        print("force value: ", self.force_value)
        
        if self.force_value > self.initial_force_value:
            self.force_flag = True

    ##### control #####
    def init_position(self):
        print("Initialising position")
        for _ in range(50):
            self.send_head_command(pan=-1.5, tilt=-0.7)
            self.send_arm_command({
                "joint_arm_l0": 0.0,
                "joint_arm_l1": 0.0,
                "joint_arm_l2": 0.0,
                "joint_arm_l3": 0.0,
                # "joint_left_wheel": 0.5,
                "joint_lift": 0.7,
                "joint_wrist_yaw": 0.0
            })

            self.rate.sleep()


    def send_arm_command(self, joint_positions, duration=2.0):
        # Create a JointTrajectory message
        traj_msg = JointTrajectory()
        traj_msg.joint_names = list(joint_positions.keys())

        # Create a single trajectory point
        point = JointTrajectoryPoint()
        point.positions = [joint_positions[joint] for joint in traj_msg.joint_names]
        point.time_from_start = rospy.Duration(duration)

        # Add the point to the trajectory message
        traj_msg.points = [point]

        # Publish the command
        self.arm_command_pub.publish(traj_msg)
        # rospy.loginfo(f"Sent arm command: {joint_positions}")

    def send_base_command(self, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        # Create a Twist message for base control
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.linear.y = linear_y
        twist_msg.angular.z = angular_z

        # Publish the command
        self.base_command_pub.publish(twist_msg)
        # rospy.loginfo(f"Sent base command: linear_x={linear_x}, linear_y={linear_y}, angular_z={angular_z}")
    
    def send_gripper_command(self, open=True, duration=2.0):
        # Create a JointTrajectory message for the gripper
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ["joint_gripper_finger_left", "joint_gripper_finger_right"]

        # Create a single trajectory point
        point = JointTrajectoryPoint()

        # Set the gripper position
        if open:
            point.positions = [0.5, 0.5]  # Open the gripper
        else:
            point.positions = [0.0, 0.0]  # Close the gripper

        point.time_from_start = rospy.Duration(duration)

        # Add the point to the trajectory message
        traj_msg.points = [point]

        # Publish the command
        self.gripper_command_pub.publish(traj_msg)
        # rospy.loginfo(f"Sent gripper command: {'open' if open else 'close'}")

    def send_head_command(self, pan=0.0, tilt=0.0, duration=2.0):
        # Create a JointTrajectory message for the head
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ["joint_head_pan", "joint_head_tilt"]

        # Create a single trajectory point
        point = JointTrajectoryPoint()
        point.positions = [pan, tilt]
        point.time_from_start = rospy.Duration(duration)

        # Add the point to the trajectory message
        traj_msg.points = [point]

        # Publish the command
        self.head_command_pub.publish(traj_msg)
        # rospy.loginfo(f"Sent head command: pan={pan}, tilt={tilt}")



    def manip_force(self):
        while not self.force_flag and not rospy.is_shutdown():
            print("Lowering arm")
            self.send_arm_command({
                "joint_arm_l0": 0.5,
                "joint_lift": 0.2,
                "joint_wrist_yaw": -0.0
            })
            # self.rate.sleep()

        print("FORCE DETECTED")

        while not rospy.is_shutdown():
            print("Stopping arm")
            self.send_arm_command({
                "joint_arm_l0": 0.5,
                "joint_lift": self.current_joint_lift,
                "joint_wrist_yaw": -0.0
            })
    
    

    def run(self):

        self.manip_force()


if __name__ == "__main__":
    try:
        controller = StretchController()
        controller.run()
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass





# self.send_head_command(pan=-1.5, tilt=-0.7)
            # self.send_arm_command({
            #     "joint_arm_l0": 0.0,
            #     "joint_arm_l1": 0.0,
            #     "joint_arm_l2": 0.0,
            #     "joint_arm_l3": 0.0,
            #     # "joint_left_wheel": 0.5,
            #     "joint_lift": 0.5,
            #     "joint_wrist_yaw": -0.0
            # })
            # self.send_gripper_command(open=True)
            # self.send_base_command(linear_x=0.5, angular_z=0.0)

