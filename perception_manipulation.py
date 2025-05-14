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
import mediapipe as mp
import sys
import tf

class StretchController:
    def __init__(self):
        rospy.init_node('stretch_arm_controller', anonymous=True)

        self.bridge = CvBridge()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.intrinsics = None  # Store camera intrinsics here

        # Subscribers
        rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)
        rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.color_camera_info_callback)
        rospy.Subscriber('/camera/color/image_raw', Image, self.color_image_callback)
        # rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.depth_camera_info_callback)
        rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_image_callback)
        # rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.depth_point_cloud_callback)

        # Publishers
        self.arm_command_pub = rospy.Publisher('/stretch_arm_controller/command', JointTrajectory, queue_size=10)
        self.base_command_pub = rospy.Publisher('/stretch_diff_drive_controller/cmd_vel', Twist, queue_size=10)
        self.gripper_command_pub = rospy.Publisher('/stretch_gripper_controller/command', JointTrajectory, queue_size=10)
        self.head_command_pub = rospy.Publisher('/stretch_head_controller/command', JointTrajectory, queue_size=10)
        self.point_cloud_pub = rospy.Publisher('/processed_depth_points', PointCloud2, queue_size=1)

        # Mediapipe pose
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)
        self.mp_drawing = mp.solutions.drawing_utils

        # Depth image buffer
        self.depth_image = None

         # Initialize Open3D visualizer
        # self.vis = o3d.visualization.Visualizer()
        # self.vis.create_window("Depth Point Cloud")
        
        # # Initialize empty point cloud
        # self.pcd = o3d.geometry.PointCloud()
        # self.vis.add_geometry(self.pcd)

        # # Add a coordinate frame to help with orientation
        # self.coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3, origin=[0, 0, 0])
        # self.vis.add_geometry(self.coordinate_frame)

        # # Set the camera parameters
        # self.reset_view()

    

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

    ##### color and depth camera #####
    def color_camera_info_callback(self, data):
        # print("=== Color Camera Info ===")
        # print(f"  Frame ID: {data.header.frame_id}")
        # print(f"  Width: {data.width}")
        # print(f"  Height: {data.height}")
        # print(f"  Distortion Model: {data.distortion_model}")
        # print(f"  D Coefficients: {data.D}")
        # print(f"  K Matrix (Intrinsic):\n    {data.K[0:3]}\n    {data.K[3:6]}\n    {data.K[6:9]}")
        # print(f"  R Matrix (Rectification):\n    {data.R[0:3]}\n    {data.R[3:6]}\n    {data.R[6:9]}")
        # print(f"  P Matrix (Projection):\n    {data.P[0:4]}\n    {data.P[4:8]}\n    {data.P[8:12]}")
        # print(f"  Binning (x, y): ({data.binning_x}, {data.binning_y})")
        # print(f"  ROI: (x_offset={data.roi.x_offset}, y_offset={data.roi.y_offset}, width={data.roi.width}, height={data.roi.height}, do_rectify={data.roi.do_rectify})")

        # Extract the camera intrinsic matrix
        self.intrinsics = np.array(data.K).reshape(3, 3)
        print("Camera Intrinsics:", self.intrinsics)

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

             # Process the image with MediaPipe
            results = self.pose.process(cv_image)

            # Draw the landmarks on the image
            if results.pose_landmarks:
                self.mp_drawing.draw_landmarks(
                    cv_image,
                    results.pose_landmarks,
                    self.mp_pose.POSE_CONNECTIONS
                )

                # Extract left wrist
                left_wrist = results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.LEFT_WRIST]
                height, width, _ = cv_image.shape
                pixel_x = int(left_wrist.x * width)
                pixel_y = int(left_wrist.y * height)

                # Get the depth value
                if self.depth_image is not None:
                    depth_value = self.depth_image[pixel_y, pixel_x]
                    if depth_value == 0:
                        print("Invalid depth at this point")
                        return
                    print(f"Left Wrist Depth (m): {depth_value}")

                    # Get 3D coordinates
                    if depth_value > 0:
                        world_coords = self.pixel_to_world(pixel_x, pixel_y, depth_value)
                        print(f"Left Wrist World Coordinates (m): {world_coords}")
                
                T = self.get_translation_matrix(world_coords[0], world_coords[1], world_coords[2])
                print("Translation Matrix (Camera to Wrist):")
                print(T)

                self.broadcast_tf(T, "camera_color_optical_frame", "left_wrist")

            else:
                print("Landmarks are not detected")

            # Display the processed image
            cv2.imshow("Pose Detection", cv_image)
            cv2.waitKey(1)

            # # Display the image
            # cv2.imshow("Color Image", cv_image)
            # cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(f"Failed to convert color image: {e}")

    def depth_image_callback(self, data):
        try:
            # Convert ROS Image message to OpenCV image (16-bit grayscale)
            depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

            # Convert to meters if it's in millimeters
            if depth_image.dtype == np.uint16:
                self.depth_image = depth_image.astype(np.float32) / 1000.0  # Convert mm to meters
                print("Convert mm to meters")
            else:
                self.depth_image = depth_image.astype(np.float32)  # Already in meters
        except Exception as e:
            rospy.logerr(f"Failed to convert depth image: {e}")
    

    def pixel_to_world(self, u, v, z):
        # Use the camera intrinsics to get the 3D point
        if self.intrinsics is None:
            rospy.logwarn("No camera intrinsics available, skipping 3D point calculation.")
            return None
        
        # Invert the camera intrinsic matrix
        fx, fy = self.intrinsics[0, 0], self.intrinsics[1, 1]
        cx, cy = self.intrinsics[0, 2], self.intrinsics[1, 2]

        # Compute the 3D world coordinates
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        return (x, y, z)

    def get_translation_matrix(self, x, y, z):
        # Create a 4x4 identity matrix
        T = np.eye(4)

        # Set the translation part
        T[0, 3] = x
        T[1, 3] = y
        T[2, 3] = z

        return T

    def broadcast_tf(self, T, parent_frame, child_frame):
        translation = T[:3, 3]
        rotation_matrix = T[:3, :3]
        quaternion = tf.transformations.quaternion_from_matrix(T)

        # Broadcast the transformation
        self.tf_broadcaster.sendTransform(
            translation,
            quaternion,
            rospy.Time.now(),
            child_frame,
            parent_frame
        )
        print(f"Broadcasting TF: {parent_frame} → {child_frame}")


    def run(self):
        # Set the ROS loop rate
        rate = rospy.Rate(1)  # 1 Hz (1 message per second)

        while not rospy.is_shutdown():
            # Continuously send commands
            self.send_head_command(pan=-1.5, tilt=-0.5)
            self.send_arm_command({
                "joint_arm_l0": 0.0,
                "joint_arm_l1": 0.0,
                "joint_arm_l2": 0.0,
                "joint_arm_l3": 0.0,
                # "joint_left_wheel": 0.5,
                "joint_lift": 0.5,
                "joint_wrist_yaw": -0.0
            })
            self.send_gripper_command(open=True)
            # self.send_base_command(linear_x=0.5, angular_z=0.0)

            # Sleep to maintain the loop rate
            rate.sleep()

        # for i in range(3):
        #     print(f"--------------- {i} ---------------")
        #     self.send_head_command(pan=0.1, tilt=0.1)
        #     self.send_arm_command({
        #         "joint_lift": 0.3,
        #         "joint_wrist_yaw": 0.5,
        #         "joint_arm_l0": 0.1
        #     })
        #     self.send_gripper_command(open=False)
        #     self.send_base_command(linear_x=0.5, angular_z=0.0)

        #     # Sleep to maintain the loop rate
        #     rate.sleep()
        
        rospy.spin()

if __name__ == "__main__":
    try:
        controller = StretchController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
