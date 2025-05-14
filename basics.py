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

class StretchController:
    def __init__(self):
        rospy.init_node('stretch_arm_controller', anonymous=True)

        self.bridge = CvBridge()

        # Subscribers
        rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)
        rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.color_camera_info_callback)
        rospy.Subscriber('/camera/color/image_raw', Image, self.color_image_callback)
        rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.depth_camera_info_callback)
        rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_image_callback)
        rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.depth_point_cloud_callback)

        # Publishers
        self.arm_command_pub = rospy.Publisher('/stretch_arm_controller/command', JointTrajectory, queue_size=10)
        self.base_command_pub = rospy.Publisher('/stretch_diff_drive_controller/cmd_vel', Twist, queue_size=10)
        self.gripper_command_pub = rospy.Publisher('/stretch_gripper_controller/command', JointTrajectory, queue_size=10)
        self.head_command_pub = rospy.Publisher('/stretch_head_controller/command', JointTrajectory, queue_size=10)
        self.point_cloud_pub = rospy.Publisher('/processed_depth_points', PointCloud2, queue_size=1)


         # Initialize Open3D visualizer
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window("Depth Point Cloud")
        
        # Initialize empty point cloud
        self.pcd = o3d.geometry.PointCloud()
        self.vis.add_geometry(self.pcd)

        # Add a coordinate frame to help with orientation
        self.coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3, origin=[0, 0, 0])
        self.vis.add_geometry(self.coordinate_frame)

        # Set the camera parameters
        self.reset_view()


    ##### joint info #####
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


    ##### control #####
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
            point.positions = [0.5, 0.05]  # Open the gripper
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

    ##### color and depth camera #####
    def color_camera_info_callback(self, data):
        print("=== Color Camera Info ===")
        print(f"  Frame ID: {data.header.frame_id}")
        print(f"  Width: {data.width}")
        print(f"  Height: {data.height}")
        print(f"  Distortion Model: {data.distortion_model}")
        print(f"  D Coefficients: {data.D}")
        print(f"  K Matrix (Intrinsic):\n    {data.K[0:3]}\n    {data.K[3:6]}\n    {data.K[6:9]}")
        # print(f"  R Matrix (Rectification):\n    {data.R[0:3]}\n    {data.R[3:6]}\n    {data.R[6:9]}")
        # print(f"  P Matrix (Projection):\n    {data.P[0:4]}\n    {data.P[4:8]}\n    {data.P[8:12]}")
        # print(f"  Binning (x, y): ({data.binning_x}, {data.binning_y})")
        # print(f"  ROI: (x_offset={data.roi.x_offset}, y_offset={data.roi.y_offset}, width={data.roi.width}, height={data.roi.height}, do_rectify={data.roi.do_rectify})")

    def depth_camera_info_callback(self, data):
        print("=== Depth Camera Info ===")
        print(f"  Frame ID: {data.header.frame_id}")
        print(f"  Width: {data.width}")
        print(f"  Height: {data.height}")
        print(f"  Distortion Model: {data.distortion_model}")
        print(f"  D Coefficients: {data.D}")
        print(f"  K Matrix (Intrinsic):\n    {data.K[0:3]}\n    {data.K[3:6]}\n    {data.K[6:9]}")
        # print(f"  R Matrix (Rectification):\n    {data.R[0:3]}\n    {data.R[3:6]}\n    {data.R[6:9]}")
        # print(f"  P Matrix (Projection):\n    {data.P[0:4]}\n    {data.P[4:8]}\n    {data.P[8:12]}")
        # print(f"  Binning (x, y): ({data.binning_x}, {data.binning_y})")
        # print(f"  ROI: (x_offset={data.roi.x_offset}, y_offset={data.roi.y_offset}, width={data.roi.width}, height={data.roi.height}, do_rectify={data.roi.do_rectify})")

    def reset_view(self):
        # Set a reasonable view point
        view_control = self.vis.get_view_control()
        view_control.set_front([0, 0, -1])
        view_control.set_lookat([0, 0, 0])
        view_control.set_up([0, -1, 0])
        view_control.set_zoom(0.8)

    def depth_point_cloud_callback(self, data):
        # Convert ROS PointCloud2 message to numpy array
        points = np.array(list(point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)))

        # Set the color to red for all points
        colors = np.ones((points.shape[0], 3)) * [1.0, 0.0, 0.0]  # Red color

        # Update point cloud
        self.pcd.points = o3d.utility.Vector3dVector(points)
        self.pcd.colors = o3d.utility.Vector3dVector(colors)

        # Update the visualizer
        self.vis.update_geometry(self.pcd)
        self.vis.poll_events()
        self.vis.update_renderer()

        self.point_cloud_pub.publish(data)

    def color_image_callback(self, data):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            # print("=== Color Image ===")
            # print(f"  Shape: {cv_image.shape}")
            # print(f"  Data Type: {cv_image.dtype}")

            # Display the image
            cv2.imshow("Color Image", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(f"Failed to convert color image: {e}")

    def depth_image_callback(self, data):
        try:
            # Convert ROS Image message to OpenCV image (16-bit grayscale)
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
            # print("=== Depth Image ===")
            # print(f"  Shape: {cv_image.shape}")
            # print(f"  Data Type: {cv_image.dtype}")

            # Normalize for visualization
            cv_normalized = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX)
            cv_normalized = cv2.convertScaleAbs(cv_normalized)

            # Display the image
            cv2.imshow("Depth Image", cv_normalized)
            cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(f"Failed to convert depth image: {e}")

    def run(self):
        # Set the ROS loop rate
        rate = rospy.Rate(1)  # 1 Hz (1 message per second)

        while not rospy.is_shutdown():
            # Continuously send commands
            self.send_head_command(pan=0.0, tilt=-0.5)
            self.send_arm_command({
                "joint_arm_l0": 0.5,
                "joint_arm_l1": 0.5,
                "joint_arm_l2": 0.5,
                "joint_arm_l3": 0.5,
                # "joint_left_wheel": 0.5,
                "joint_lift": 0.5,
                "joint_wrist_yaw": 1.0
            })
            self.send_gripper_command(open=True)
            self.send_base_command(linear_x=0.5, angular_z=0.0)

            # Sleep to maintain the loop rate
            rate.sleep()

        
        rospy.spin()

if __name__ == "__main__":
    try:
        controller = StretchController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
