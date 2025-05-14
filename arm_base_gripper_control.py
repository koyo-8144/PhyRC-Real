#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist

class StretchController:
    def __init__(self):
        rospy.init_node('stretch_arm_controller', anonymous=True)

        # Subscribers
        rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)

        # Publishers
        self.arm_command_pub = rospy.Publisher('/stretch_arm_controller/command', JointTrajectory, queue_size=10)
        self.base_command_pub = rospy.Publisher('/stretch_diff_drive_controller/cmd_vel', Twist, queue_size=10)
        self.gripper_command_pub = rospy.Publisher('/stretch_gripper_controller/command', JointTrajectory, queue_size=10)

        print("Initialization done")

    def joint_states_callback(self, data):
        print("=== Joint States ===")
        name = data.name
        position = data.position
        velocity = data.velocity
        effort = data.effort

        print(f"Joint Name: {name}")
        print(f"  Position: {position:}")
        print(f"  Velocity: {velocity:}")
        print(f"  Effort: {effort:}")

        # Example: Move the arm to a new position
        self.send_arm_command({
            "joint_lift": 0.3,
            "joint_wrist_yaw": 0.5,
            "joint_arm_l0": 0.2
        })

        self.send_base_command(linear_x=0.0, angular_z=0.0)

        self.send_gripper_command(open=True)

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
        rospy.loginfo(f"Sent arm command: {joint_positions}")

    def send_base_command(self, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        # Create a Twist message for base control
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.linear.y = linear_y
        twist_msg.angular.z = angular_z

        # Publish the command
        self.base_command_pub.publish(twist_msg)
        rospy.loginfo(f"Sent base command: linear_x={linear_x}, linear_y={linear_y}, angular_z={angular_z}")
    
    def send_gripper_command(self, open=True, duration=2.0):
        # Create a JointTrajectory message for the gripper
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ["joint_gripper_finger_left", "joint_gripper_finger_right"]

        # Create a single trajectory point
        point = JointTrajectoryPoint()

        # Set the gripper position
        if open:
            point.positions = [0.5, 0.05]  # Open the gripper
        else:
            point.positions = [0.0, 0.0]  # Close the gripper

        point.time_from_start = rospy.Duration(duration)

        # Add the point to the trajectory message
        traj_msg.points = [point]

        # Publish the command
        self.gripper_command_pub.publish(traj_msg)
        rospy.loginfo(f"Sent gripper command: {'open' if open else 'close'}")

    def run(self):
        rospy.spin()
    
    


if __name__ == "__main__":
    try:
        controller = StretchController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
