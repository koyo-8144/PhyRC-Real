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
from ultralytics import YOLO
from std_msgs.msg import Bool
import time

class StretchController:
    def __init__(self):
        rospy.init_node('stretch_arm_controller', anonymous=True)

        self.bridge = CvBridge()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        self.intrinsics = None  # Store camera intrinsics here
        self.rate = rospy.Rate(0.1)  # 1 Hz (1 message per second)

        self.yolo_model = YOLO("yolov8n.pt")  # Use a small model for fast inference

        # Publishers
        self.arm_command_pub = rospy.Publisher('/stretch_arm_controller/command', JointTrajectory, queue_size=10)
        self.base_command_pub = rospy.Publisher('/stretch_diff_drive_controller/cmd_vel', Twist, queue_size=10)
        self.gripper_command_pub = rospy.Publisher('/stretch_gripper_controller/command', JointTrajectory, queue_size=10)
        self.head_command_pub = rospy.Publisher('/stretch_head_controller/command', JointTrajectory, queue_size=10)
        self.point_cloud_pub = rospy.Publisher('/processed_depth_points', PointCloud2, queue_size=1)
        self.manip_done_pub = rospy.Publisher("/manipulation_done", Bool, queue_size=10)

        # Subscribers
        rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)
        rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.color_camera_info_callback)
        rospy.Subscriber('/camera/color/image_raw', Image, self.color_image_callback)
        # rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.depth_camera_info_callback)
        rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_image_callback)
        # rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.depth_point_cloud_callback)
        rospy.Subscriber('/manipulation_start', Bool, self.manip_start_callback)

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

        self.move_x = False
        self.y_done_1 = False
        self.z_done_1 = False
        self.y_done_2 = False
        self.z_done_2 = False
        self.done_1 = False
        self.done_2 = False
        self.y_threshold = 0.2
        self.z_threshold = 0.1
        self.filtered_delta_y = 0.5
        self.filtered_delta_z = 0.5
        self.y_smoothing_factor = 0.8  # Adjust this for smoother or more responsive filtering
        self.z_smoothing_factor = 0.8

        self.manip_start_flag = False
        self.percep_done_flag = False
        self.percep_count = 0

        self.current_joint_lift = 0.5


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

    def manip_start_callback(self, msg):
        if msg.data:
            # rospy.loginfo("Manipulation start signal received.")
            self.manip_start_flag = True

           

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
        # print("Camera Intrinsics:", self.intrinsics)

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

    def grab_sponge(self):
        if self.percep_done_flag:
            print("Start grab_sponge")
            T_base_human = self.listen_parent_to_child("base_link", "detected_human")
            T_base_ee = self.listen_parent_to_child("base_link", "link_grasp_center")

            delta_x, delta_y, delta_z = self.compare_translation(T_base_human, T_base_ee)

            # Apply low-pass filtering
            self.filtered_delta_y = self.apply_low_pass_filter(delta_y, self.filtered_delta_y, self.y_smoothing_factor)
            self.filtered_delta_z = self.apply_low_pass_filter(delta_z, self.filtered_delta_z, self.z_smoothing_factor)


            print("Δy (filtered): ", self.filtered_delta_y)
            self.send_head_command(pan=-1.5, tilt=-0.7)

            #### l0 ####
            if not self.y_done_1:
                for i in range(10):
                    print(f"-----1st l0 {i+1}-----")
                    self.send_arm_command({
                        "joint_arm_l0": 0.5,
                        "joint_lift": 0.5,
                        "joint_wrist_yaw": -0.0
                    })
                    # self.rate.sleep()

                _, delta_y, _ = self.compare_translation(T_base_human, T_base_ee)
                print("Δy (filtered): ", self.filtered_delta_y)
                if abs(self.filtered_delta_y) < self.y_threshold:
                    print("Y is close enough")
                    self.y_done_1 = True

                if not self.y_done_1:
                    for i in range(10):
                        print(f"-----2nd l0 {i+1}-----")
                        self.send_arm_command({
                            "joint_arm_l0": 1.0,
                            "joint_lift": 0.5,
                            "joint_wrist_yaw": -0.0
                        })
                        # self.rate.sleep()

                    _, delta_y, _ = self.compare_translation(T_base_human, T_base_ee)
                    print("Δy (filtered): ", self.filtered_delta_y)
                    if abs(self.filtered_delta_y) < self.y_threshold:
                        print("Y is close enough")
                        self.y_done_1 = True

                    if not self.y_done_1:
                        for i in range(10):
                            print(f"-----3rd l0 {i+1}-----")
                            self.send_arm_command({
                                "joint_arm_l0": 1.5,
                                "joint_lift": 0.5,
                                "joint_wrist_yaw": -0.0
                            })
                            # self.rate.sleep()

                        _, delta_y, _ = self.compare_translation(T_base_human, T_base_ee)
                        print("Δy (filtered): ", self.filtered_delta_y)
                        if abs(self.filtered_delta_y) < self.y_threshold:
                            print("Y is close enough")
                            self.y_done_1 = True
                
                        if not self.y_done_1:
                            for i in range(10):
                                print(f"-----4th l0 {i+1}-----")
                                self.send_arm_command({
                                    "joint_arm_l0": 2.0,
                                })
                                # self.rate.sleep()

                            _, delta_y, _ = self.compare_translation(T_base_human, T_base_ee)
                            print("Δy (filtered): ", self.filtered_delta_y)
                            if abs(self.filtered_delta_y) < self.y_threshold:
                                print("Y is close enough")
                                self.y_done_1 = True
                
                            #### l1 ####
                            if not self.y_done_1:
                                for i in range(10):
                                    print(f"-----1st l1 {i+1}-----")
                                    self.send_arm_command({
                                        "joint_arm_l1": 0.5,
                                        "joint_lift": 0.5,
                                        "joint_wrist_yaw": -0.0
                                    })
                                    # self.rate.sleep()

                                _, delta_y, _ = self.compare_translation(T_base_human, T_base_ee)
                                print("Δy (filtered): ", self.filtered_delta_y)
                                if abs(self.filtered_delta_y) < self.y_threshold:
                                    print("Y is close enough")
                                    self.y_done_1 = True
                
                                if not self.y_done_1:
                                    for i in range(10):
                                        print(f"-----2nd l1 {i+1}-----")
                                        self.send_arm_command({
                                            "joint_arm_l1": 1.0,
                                            "joint_lift": 0.5,
                                            "joint_wrist_yaw": -0.0
                                        })
                                        # self.rate.sleep()

                                    _, delta_y, _ = self.compare_translation(T_base_human, T_base_ee)
                                    print("Δy (filtered): ", self.filtered_delta_y)
                                    if abs(self.filtered_delta_y) < self.y_threshold:
                                        print("Y is close enough")
                                        self.y_done_1 = True
                
                                    if not self.y_done_1:
                                        for i in range(10):
                                            print(f"-----3rd l1 {i+1}-----")
                                            self.send_arm_command({
                                                "joint_arm_l1": 1.5,
                                                "joint_lift": 0.5,
                                                "joint_wrist_yaw": -0.0
                                            })
                                            # self.rate.sleep()

                                        _, delta_y, _ = self.compare_translation(T_base_human, T_base_ee)
                                        print("Δy (filtered): ", self.filtered_delta_y)
                                        if abs(self.filtered_delta_y) < self.y_threshold:
                                            print("Y is close enough")
                                            self.y_done_1 = True
                
                                        if not self.y_done_1:
                                            for i in range(10):
                                                print(f"-----4th l1 {i+1}-----")
                                                self.send_arm_command({
                                                    "joint_arm_l1": 2.0,
                                                    "joint_lift": 0.5,
                                                    "joint_wrist_yaw": -0.0
                                                })
                                                # self.rate.sleep()

                                            _, delta_y, _ = self.compare_translation(T_base_human, T_base_ee)
                                            print("Δy (filtered): ", self.filtered_delta_y)
                                            if abs(self.filtered_delta_y) < self.y_threshold:
                                                print("Y is close enough")
                                                self.y_done_1 = True

                                            #### l2 ####
                                            if not self.y_done_1:
                                                for i in range(10):
                                                    print(f"-----1st l2 {i+1}-----")
                                                    self.send_arm_command({
                                                        "joint_arm_l2": 0.5,
                                                        "joint_lift": 0.5,
                                                        "joint_wrist_yaw": -0.0
                                                    })
                                                    # self.rate.sleep()

                                                _, delta_y, _ = self.compare_translation(T_base_human, T_base_ee)
                                                print("Δy (filtered): ", self.filtered_delta_y)
                                                if abs(self.filtered_delta_y) < self.y_threshold:
                                                    print("Y is close enough")
                                                    self.y_done_1 = True
                
                                                if not self.y_done_1:
                                                    for i in range(10):
                                                        print(f"-----2nd l2 {i+1}-----")
                                                        self.send_arm_command({
                                                            "joint_arm_l2": 1.0,
                                                            "joint_lift": 0.5,
                                                            "joint_wrist_yaw": -0.0
                                                        })
                                                        # self.rate.sleep()

                                                    _, delta_y, _ = self.compare_translation(T_base_human, T_base_ee)
                                                    print("Δy (filtered): ", self.filtered_delta_y)
                                                    if abs(self.filtered_delta_y) < self.y_threshold:
                                                        print("Y is close enough")
                                                        self.y_done_1 = True
                
                                                    if not self.y_done_1:
                                                        for i in range(10):
                                                            print(f"-----3rd l2 {i+1}-----")
                                                            self.send_arm_command({
                                                                "joint_arm_l2": 1.5,
                                                                "joint_lift": 0.5,
                                                                "joint_wrist_yaw": -0.0
                                                            })
                                                            # self.rate.sleep()

                                                        _, delta_y, _ = self.compare_translation(T_base_human, T_base_ee)
                                                        print("Δy (filtered): ", self.filtered_delta_y)
                                                        if abs(self.filtered_delta_y) < self.y_threshold:
                                                            print("Y is close enough")
                                                            self.y_done_1 = True
                
                                                        if not self.y_done_1:
                                                            for i in range(10):
                                                                print(f"-----4th l2 {i+1}-----")
                                                                self.send_arm_command({
                                                                    "joint_arm_l2": 2.0,
                                                                    "joint_lift": 0.5,
                                                                    "joint_wrist_yaw": -0.0
                                                                })
                                                                # self.rate.sleep()

                                                            _, delta_y, _ = self.compare_translation(T_base_human, T_base_ee)
                                                            print("Δy (filtered): ", self.filtered_delta_y)
                                                            if abs(self.filtered_delta_y) < self.y_threshold:
                                                                print("Y is close enough")
                                                                self.y_done_1 = True
                
                                                            #### l3 ####
                                                            if not self.y_done_1:
                                                                for i in range(10):
                                                                    print(f"-----1st l3 {i+1}-----")
                                                                    self.send_arm_command({
                                                                        "joint_arm_l3": 0.5,
                                                                    })
                                                                    # self.rate.sleep()

                                                                _, delta_y, _ = self.compare_translation(T_base_human, T_base_ee)
                                                                print("Δy (filtered): ", self.filtered_delta_y)
                                                                if abs(self.filtered_delta_y) < self.y_threshold:
                                                                    print("Y is close enough")
                                                                    self.y_done_1 = True
                
                                                                if not self.y_done_1:
                                                                    for i in range(10):
                                                                        print(f"-----2nd l3 {i+1}-----")
                                                                        self.send_arm_command({
                                                                            "joint_arm_l3": 1.0,
                                                                        })
                                                                        # self.rate.sleep()

                                                                    _, delta_y, _ = self.compare_translation(T_base_human, T_base_ee)
                                                                    print("Δy (filtered): ", self.filtered_delta_y)
                                                                    if abs(self.filtered_delta_y) < self.y_threshold:
                                                                        print("Y is close enough")
                                                                        self.y_done_1 = True
                
                                                                    if not self.y_done_1:
                                                                        for i in range(10):
                                                                            print(f"-----3rd l3 {i+1}-----")
                                                                            self.send_arm_command({
                                                                                "joint_arm_l3": 1.5,
                                                                            })
                                                                            # self.rate.sleep()

                                                                        _, delta_y, _ = self.compare_translation(T_base_human, T_base_ee)
                                                                        print("Δy (filtered): ", self.filtered_delta_y)
                                                                        if abs(self.filtered_delta_y) < self.y_threshold:
                                                                            print("Y is close enough")
                                                                            self.y_done_1 = True
                
                                                                        if not self.y_done_1:
                                                                            for i in range(10):
                                                                                print(f"-----4th l3 {i+1}-----")
                                                                                self.send_arm_command({
                                                                                    "joint_arm_l3": 2.0,
                                                                                })
                                                                                # self.rate.sleep()

                                                                            _, delta_y, _ = self.compare_translation(T_base_human, T_base_ee)
                                                                            print("Δy (filtered): ", self.filtered_delta_y)
                                                                            if abs(self.filtered_delta_y) < self.y_threshold:
                                                                                print("Y is close enough")
                                                                                self.y_done_1 = True
            
            else:
                print("Y is close enough")
                
                print("Δz (filtered): ", self.filtered_delta_z)

                for i in range(30):
                    print("Current lift: ", self.current_joint_lift)
                    self.send_arm_command({
                        "joint_lift": 0.48,
                    })

                if not self.z_done_1:
                    for i in range(10):
                        print(f"-----1st lift down {i+1}-----")
                        self.send_arm_command({
                            "joint_lift": 0.4,
                        })
                        # self.rate.sleep()

                    _, _, delta_z = self.compare_translation(T_base_human, T_base_ee)
                    print("Δz (filtered): ", self.filtered_delta_z)
                    if abs(self.filtered_delta_z) < self.z_threshold:
                        print("Z is close enough")
                        self.z_done_1 = True
                    
                    for i in range(10):
                        print(f"-----2nd lift down {i+1}-----")
                        self.send_arm_command({
                            "joint_lift": 0.3,
                        })
                        # self.rate.sleep()

                    _, _, delta_z = self.compare_translation(T_base_human, T_base_ee)
                    print("Δz (filtered): ", self.filtered_delta_z)
                    if abs(self.filtered_delta_z) < self.z_threshold:
                        print("Z is close enough")
                        self.z_done_1 = True
                    
                    for i in range(10):
                        print(f"-----3rd lift down {i+1}-----")
                        self.send_arm_command({
                            "joint_lift": 0.2,
                        })
                        # self.rate.sleep()

                    _, _, delta_z = self.compare_translation(T_base_human, T_base_ee)
                    print("Δz (filtered): ", self.filtered_delta_z)
                    if abs(self.filtered_delta_z) < self.z_threshold:
                        print("Z is close enough")
                        self.z_done_1 = True
                
                else:
                    print("Z is close enough")
                    for i in range(20):
                        self.send_arm_command({
                            "joint_lift": self.current_joint_lift,
                        })




    # def color_image_callback(self, data):
    #     try:
    #         # Convert ROS Image message to OpenCV image
    #         cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            
    #         # Run YOLOv8 inference
    #         results = self.yolo_model(cv_image)

    #         # Iterate over detected objects
    #         for result in results:  # Each frame has its own result
    #             for box in result.boxes:  # Get bounding boxes
    #                 x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
    #                 conf = box.conf[0].item()
    #                 cls = int(box.cls[0].item())
                    
    #                 # Check if the detected object is a person (class 0)
    #                 if cls == 0:
    #                     # Draw the bounding box
    #                     cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
    #                     cv2.putText(cv_image, f"Person {conf:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    #                     # Get the center of the bounding box
    #                     center_x = int((x1 + x2) / 2)
    #                     center_y = int((y1 + y2) / 2)

    #                     # Get the depth value
    #                     if self.depth_image is not None:
    #                         depth_value = self.depth_image[center_y, center_x]
    #                         if depth_value == 0:
    #                             print("Invalid depth at this point")
    #                             continue

    #                         # Convert to world coordinates
    #                         if depth_value > 0:
    #                             world_coords = self.pixel_to_world(center_x, center_y, depth_value)
    #                             # print(f"Human World Coordinates (m): {world_coords}")

    #                             # Broadcast the transform
    #                             T_camera_human = self.get_translation_matrix(world_coords[0], world_coords[1], world_coords[2])
    #                             self.broadcast_tf(T_camera_human, "camera_color_optical_frame", "detected_human")

    #                             T_base_human = self.listen_parent_to_child("base_link", "detected_human")
    #                             T_base_ee = self.listen_parent_to_child("base_link", "link_grasp_center")

    #                             delta_x, delta_y, delta_z = self.compare_translation(T_base_human, T_base_ee)
    #                             self.filtered_delta_y = self.apply_low_pass_filter(delta_y, self.filtered_delta_y, self.y_smoothing_factor)
    #                             self.filtered_delta_z = self.apply_low_pass_filter(delta_z, self.filtered_delta_z, self.z_smoothing_factor)

    #                             if self.move_x:
    #                                 print("Δx: ", delta_x)
    #                             else:
    #                                 print("Δy (filtered): ", self.filtered_delta_y)
    #                                 self.send_head_command(pan=-1.5, tilt=-0.7)

    #                                 #### l0 ####
    #                                 if not self.y_done_1:
    #                                     for i in range(10):
    #                                         print(f"-----1st l0 {i+1}-----")
    #                                         self.send_arm_command({
    #                                             "joint_arm_l0": 0.5,
    #                                             "joint_lift": 0.5,
    #                                             "joint_wrist_yaw": -0.0
    #                                         })
    #                                         # self.rate.sleep()

    #                                     _, delta_y, _ = self.compare_translation(T_base_human, T_base_ee)
    #                                     print("Δy (filtered): ", self.filtered_delta_y)
    #                                     if abs(self.filtered_delta_y) < self.y_threshold:
    #                                         print("Y is close enough")
    #                                         self.y_done_1 = True

    #                                     if not self.y_done_1:
    #                                         for i in range(10):
    #                                             print(f"-----2nd l0 {i+1}-----")
    #                                             self.send_arm_command({
    #                                                 "joint_arm_l0": 1.0,
    #                                                 "joint_lift": 0.5,
    #                                                 "joint_wrist_yaw": -0.0
    #                                             })
    #                                             # self.rate.sleep()

    #                                         _, delta_y, _ = self.compare_translation(T_base_human, T_base_ee)
    #                                         print("Δy (filtered): ", self.filtered_delta_y)
    #                                         if abs(self.filtered_delta_y) < self.y_threshold:
    #                                             print("Y is close enough")
    #                                             self.y_done_1 = True

    #                                         if not self.y_done_1:
    #                                             for i in range(10):
    #                                                 print(f"-----3rd l0 {i+1}-----")
    #                                                 self.send_arm_command({
    #                                                     "joint_arm_l0": 1.5,
    #                                                     "joint_lift": 0.5,
    #                                                     "joint_wrist_yaw": -0.0
    #                                                 })
    #                                                 # self.rate.sleep()

    #                                             _, delta_y, _ = self.compare_translation(T_base_human, T_base_ee)
    #                                             print("Δy (filtered): ", self.filtered_delta_y)
    #                                             if abs(self.filtered_delta_y) < self.y_threshold:
    #                                                 print("Y is close enough")
    #                                                 self.y_done_1 = True
                                        
    #                                             if not self.y_done_1:
    #                                                 for i in range(10):
    #                                                     print(f"-----4th l0 {i+1}-----")
    #                                                     self.send_arm_command({
    #                                                         "joint_arm_l0": 2.0,
    #                                                     })
    #                                                     # self.rate.sleep()

    #                                                 _, delta_y, _ = self.compare_translation(T_base_human, T_base_ee)
    #                                                 print("Δy (filtered): ", self.filtered_delta_y)
    #                                                 if abs(self.filtered_delta_y) < self.y_threshold:
    #                                                     print("Y is close enough")
    #                                                     self.y_done_1 = True
                                        
    #                                                 #### l1 ####
    #                                                 if not self.y_done_1:
    #                                                     for i in range(10):
    #                                                         print(f"-----1st l1 {i+1}-----")
    #                                                         self.send_arm_command({
    #                                                             "joint_arm_l1": 0.5,
    #                                                             "joint_lift": 0.5,
    #                                                             "joint_wrist_yaw": -0.0
    #                                                         })
    #                                                         # self.rate.sleep()

    #                                                     _, delta_y, _ = self.compare_translation(T_base_human, T_base_ee)
    #                                                     print("Δy (filtered): ", self.filtered_delta_y)
    #                                                     if abs(self.filtered_delta_y) < self.y_threshold:
    #                                                         print("Y is close enough")
    #                                                         self.y_done_1 = True
                                        
    #                                                     if not self.y_done_1:
    #                                                         for i in range(10):
    #                                                             print(f"-----2nd l1 {i+1}-----")
    #                                                             self.send_arm_command({
    #                                                                 "joint_arm_l1": 1.0,
    #                                                                 "joint_lift": 0.5,
    #                                                                 "joint_wrist_yaw": -0.0
    #                                                             })
    #                                                             # self.rate.sleep()

    #                                                         _, delta_y, _ = self.compare_translation(T_base_human, T_base_ee)
    #                                                         print("Δy (filtered): ", self.filtered_delta_y)
    #                                                         if abs(self.filtered_delta_y) < self.y_threshold:
    #                                                             print("Y is close enough")
    #                                                             self.y_done_1 = True
                                        
    #                                                         if not self.y_done_1:
    #                                                             for i in range(10):
    #                                                                 print(f"-----3rd l1 {i+1}-----")
    #                                                                 self.send_arm_command({
    #                                                                     "joint_arm_l1": 1.5,
    #                                                                     "joint_lift": 0.5,
    #                                                                     "joint_wrist_yaw": -0.0
    #                                                                 })
    #                                                                 # self.rate.sleep()

    #                                                             _, delta_y, _ = self.compare_translation(T_base_human, T_base_ee)
    #                                                             print("Δy (filtered): ", self.filtered_delta_y)
    #                                                             if abs(self.filtered_delta_y) < self.y_threshold:
    #                                                                 print("Y is close enough")
    #                                                                 self.y_done_1 = True
                                        
    #                                                             if not self.y_done_1:
    #                                                                 for i in range(10):
    #                                                                     print(f"-----4th l1 {i+1}-----")
    #                                                                     self.send_arm_command({
    #                                                                         "joint_arm_l1": 2.0,
    #                                                                         "joint_lift": 0.5,
    #                                                                         "joint_wrist_yaw": -0.0
    #                                                                     })
    #                                                                     # self.rate.sleep()

    #                                                                 _, delta_y, _ = self.compare_translation(T_base_human, T_base_ee)
    #                                                                 print("Δy (filtered): ", self.filtered_delta_y)
    #                                                                 if abs(self.filtered_delta_y) < self.y_threshold:
    #                                                                     print("Y is close enough")
    #                                                                     self.y_done_1 = True

    #                                                                 #### l2 ####
    #                                                                 if not self.y_done_1:
    #                                                                     for i in range(10):
    #                                                                         print(f"-----1st l2 {i+1}-----")
    #                                                                         self.send_arm_command({
    #                                                                             "joint_arm_l2": 0.5,
    #                                                                             "joint_lift": 0.5,
    #                                                                             "joint_wrist_yaw": -0.0
    #                                                                         })
    #                                                                         # self.rate.sleep()

    #                                                                     _, delta_y, _ = self.compare_translation(T_base_human, T_base_ee)
    #                                                                     print("Δy (filtered): ", self.filtered_delta_y)
    #                                                                     if abs(self.filtered_delta_y) < self.y_threshold:
    #                                                                         print("Y is close enough")
    #                                                                         self.y_done_1 = True
                                        
    #                                                                     if not self.y_done_1:
    #                                                                         for i in range(10):
    #                                                                             print(f"-----2nd l2 {i+1}-----")
    #                                                                             self.send_arm_command({
    #                                                                                 "joint_arm_l2": 1.0,
    #                                                                                 "joint_lift": 0.5,
    #                                                                                 "joint_wrist_yaw": -0.0
    #                                                                             })
    #                                                                             # self.rate.sleep()

    #                                                                         _, delta_y, _ = self.compare_translation(T_base_human, T_base_ee)
    #                                                                         print("Δy (filtered): ", self.filtered_delta_y)
    #                                                                         if abs(self.filtered_delta_y) < self.y_threshold:
    #                                                                             print("Y is close enough")
    #                                                                             self.y_done_1 = True
                                        
    #                                                                         if not self.y_done_1:
    #                                                                             for i in range(10):
    #                                                                                 print(f"-----3rd l2 {i+1}-----")
    #                                                                                 self.send_arm_command({
    #                                                                                     "joint_arm_l2": 1.5,
    #                                                                                     "joint_lift": 0.5,
    #                                                                                     "joint_wrist_yaw": -0.0
    #                                                                                 })
    #                                                                                 # self.rate.sleep()

    #                                                                             _, delta_y, _ = self.compare_translation(T_base_human, T_base_ee)
    #                                                                             print("Δy (filtered): ", self.filtered_delta_y)
    #                                                                             if abs(self.filtered_delta_y) < self.y_threshold:
    #                                                                                 print("Y is close enough")
    #                                                                                 self.y_done_1 = True
                                        
    #                                                                             if not self.y_done_1:
    #                                                                                 for i in range(10):
    #                                                                                     print(f"-----4th l2 {i+1}-----")
    #                                                                                     self.send_arm_command({
    #                                                                                         "joint_arm_l2": 2.0,
    #                                                                                         "joint_lift": 0.5,
    #                                                                                         "joint_wrist_yaw": -0.0
    #                                                                                     })
    #                                                                                     # self.rate.sleep()

    #                                                                                 _, delta_y, _ = self.compare_translation(T_base_human, T_base_ee)
    #                                                                                 print("Δy (filtered): ", self.filtered_delta_y)
    #                                                                                 if abs(self.filtered_delta_y) < self.y_threshold:
    #                                                                                     print("Y is close enough")
    #                                                                                     self.y_done_1 = True
                                        
    #                                                                                 #### l3 ####
    #                                                                                 if not self.y_done_1:
    #                                                                                     for i in range(10):
    #                                                                                         print(f"-----1st l3 {i+1}-----")
    #                                                                                         self.send_arm_command({
    #                                                                                             "joint_arm_l3": 0.5,
    #                                                                                         })
    #                                                                                         # self.rate.sleep()

    #                                                                                     _, delta_y, _ = self.compare_translation(T_base_human, T_base_ee)
    #                                                                                     print("Δy (filtered): ", self.filtered_delta_y)
    #                                                                                     if abs(self.filtered_delta_y) < self.y_threshold:
    #                                                                                         print("Y is close enough")
    #                                                                                         self.y_done_1 = True
                                        
    #                                                                                     if not self.y_done_1:
    #                                                                                         for i in range(10):
    #                                                                                             print(f"-----2nd l3 {i+1}-----")
    #                                                                                             self.send_arm_command({
    #                                                                                                 "joint_arm_l3": 1.0,
    #                                                                                             })
    #                                                                                             # self.rate.sleep()

    #                                                                                         _, delta_y, _ = self.compare_translation(T_base_human, T_base_ee)
    #                                                                                         print("Δy (filtered): ", self.filtered_delta_y)
    #                                                                                         if abs(self.filtered_delta_y) < self.y_threshold:
    #                                                                                             print("Y is close enough")
    #                                                                                             self.y_done_1 = True
                                        
    #                                                                                         if not self.y_done_1:
    #                                                                                             for i in range(10):
    #                                                                                                 print(f"-----3rd l3 {i+1}-----")
    #                                                                                                 self.send_arm_command({
    #                                                                                                     "joint_arm_l3": 1.5,
    #                                                                                                 })
    #                                                                                                 # self.rate.sleep()

    #                                                                                             _, delta_y, _ = self.compare_translation(T_base_human, T_base_ee)
    #                                                                                             print("Δy (filtered): ", self.filtered_delta_y)
    #                                                                                             if abs(self.filtered_delta_y) < self.y_threshold:
    #                                                                                                 print("Y is close enough")
    #                                                                                                 self.y_done_1 = True
                                        
    #                                                                                             if not self.y_done_1:
    #                                                                                                 for i in range(10):
    #                                                                                                     print(f"-----4th l3 {i+1}-----")
    #                                                                                                     self.send_arm_command({
    #                                                                                                         "joint_arm_l3": 2.0,
    #                                                                                                     })
    #                                                                                                     # self.rate.sleep()

    #                                                                                                 _, delta_y, _ = self.compare_translation(T_base_human, T_base_ee)
    #                                                                                                 print("Δy (filtered): ", self.filtered_delta_y)
    #                                                                                                 if abs(self.filtered_delta_y) < self.y_threshold:
    #                                                                                                     print("Y is close enough")
    #                                                                                                     self.y_done_1 = True
                                    
    #                                 else:
    #                                     print("Y is close enough")
                                        
    #                                     print("Δz (filtered): ", self.filtered_delta_z)

    #                                     for i in range(30):
    #                                         print("Current lift: ", self.current_joint_lift)
    #                                         self.send_arm_command({
    #                                             "joint_lift": 0.48,
    #                                         })

    #                                     # if not self.z_done_1:
    #                                     #     for i in range(10):
    #                                     #         print(f"-----1st lift down {i+1}-----")
    #                                     #         self.send_arm_command({
    #                                     #             "joint_lift": 0.4,
    #                                     #         })
    #                                     #         # self.rate.sleep()

    #                                     #     _, _, delta_z = self.compare_translation(T_base_human, T_base_ee)
    #                                     #     print("Δz (filtered): ", self.filtered_delta_z)
    #                                     #     if abs(self.filtered_delta_z) < self.z_threshold:
    #                                     #         print("Z is close enough")
    #                                     #         self.z_done_1 = True
                                            
    #                                     #     for i in range(10):
    #                                     #         print(f"-----2nd lift down {i+1}-----")
    #                                     #         self.send_arm_command({
    #                                     #             "joint_lift": 0.3,
    #                                     #         })
    #                                     #         # self.rate.sleep()

    #                                     #     _, _, delta_z = self.compare_translation(T_base_human, T_base_ee)
    #                                     #     print("Δz (filtered): ", self.filtered_delta_z)
    #                                     #     if abs(self.filtered_delta_z) < self.z_threshold:
    #                                     #         print("Z is close enough")
    #                                     #         self.z_done_1 = True
                                            
    #                                     #     for i in range(10):
    #                                     #         print(f"-----3rd lift down {i+1}-----")
    #                                     #         self.send_arm_command({
    #                                     #             "joint_lift": 0.2,
    #                                     #         })
    #                                     #         # self.rate.sleep()

    #                                     #     _, _, delta_z = self.compare_translation(T_base_human, T_base_ee)
    #                                     #     print("Δz (filtered): ", self.filtered_delta_z)
    #                                     #     if abs(self.filtered_delta_z) < self.z_threshold:
    #                                     #         print("Z is close enough")
    #                                     #         self.z_done_1 = True
                                        
    #                                     # else:
    #                                     #     print("Z is close enough")
    #                                     #     for i in range(20):
    #                                     #         self.send_arm_command({
    #                                     #             "joint_lift": self.current_joint_lift,
    #                                     #         })


    #         # Display the processed image
    #         cv2.imshow("YOLO Detection", cv_image)
    #         cv2.waitKey(1)

    #     except Exception as e:
    #         rospy.logerr(f"Failed to process color image: {e}")

    def color_image_callback(self, data):
        if self.manip_start_flag:
            try:
                # Convert ROS Image message to OpenCV image
                cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
                
                # Run YOLOv8 inference
                results = self.yolo_model(cv_image)

                # Process only the first detected person (more efficient)
                for result in results:
                    for box in result.boxes:
                        cls = int(box.cls[0].item())
                        if cls == 0:  # Person class
                            # Extract bounding box
                            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                            center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2

                            # Get depth
                            if self.depth_image is None:
                                continue
                            
                            depth_value = self.depth_image[center_y, center_x]
                            if depth_value <= 0:
                                print("Invalid depth at this point")
                                continue

                            # Convert to world coordinates
                            world_coords = self.pixel_to_world(center_x, center_y, depth_value)
                            T_camera_human = self.get_translation_matrix(*world_coords)
                            self.broadcast_tf(T_camera_human, "camera_color_optical_frame", "detected_human")

                            # T_base_human = self.listen_parent_to_child("base_link", "detected_human")
                            # T_base_ee = self.listen_parent_to_child("base_link", "link_grasp_center")

                            # delta_x, delta_y, delta_z = self.compare_translation(T_base_human, T_base_ee)

                            # # Apply low-pass filtering
                            # self.filtered_delta_y = self.apply_low_pass_filter(delta_y, self.filtered_delta_y, self.y_smoothing_factor)
                            # self.filtered_delta_z = self.apply_low_pass_filter(delta_z, self.filtered_delta_z, self.z_smoothing_factor)


                            # Draw the bounding box
                            conf = box.conf[0].item()
                            cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                            cv2.putText(cv_image, f"Person {conf:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                            # Break after first person detection for efficiency
                            break

                # Display the processed image
                if not self.percep_done_flag:
                    cv2.imshow("YOLO Detection", cv_image)
                    cv2.waitKey(1)

                self.percep_count += 1
                if self.percep_count % 100 == 0:
                    self.percep_done_flag = True
                    self.manip_start_flag = False  # Reset for the next cycle
                    rospy.loginfo("Perception complete. Closing display...")
                    cv2.destroyAllWindows()

            except Exception as e:
                rospy.logerr(f"Failed to process color image: {e}")


    def depth_image_callback(self, data):
        if self.manip_start_flag:
            try:
                # Convert ROS Image message to OpenCV image (16-bit grayscale)
                depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

                # Convert to meters if it's in millimeters
                if depth_image.dtype == np.uint16:
                    self.depth_image = depth_image.astype(np.float32) / 1000.0  # Convert mm to meters
                    # print("Convert mm to meters")
                else:
                    self.depth_image = depth_image.astype(np.float32)  # Already in meters
            except Exception as e:
                rospy.logerr(f"Failed to convert depth image: {e}")
    

    ##### camera #####
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


    ##### tf #####
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
        # print(f"Broadcasting TF: {parent_frame} → {child_frame}")

    def listen_parent_to_child(self, parent_frame, child_frame):
        while not rospy.is_shutdown():
            try:
                # Get transformation from base_link to end_effector_link
                (trans, rot) = self.listener.lookupTransform(parent_frame, child_frame, rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        T = self.construct_rot_matrix_homogeneous_transform(trans, rot)

        return T
    
    def construct_rot_matrix_homogeneous_transform(self, translation, quaternion):
        # Convert quaternion to rotation matrix
        rotation_matrix = tf.transformations.quaternion_matrix(quaternion)[:3, :3]
        # print("Rotation Matrix:")
        # print(rotation_matrix)

        # Construct the 4x4 homogeneous transformation matrix
        T = np.identity(4)
        T[:3, :3] = rotation_matrix
        T[:3, 3] = translation

        return T

    def compare_translation(self, T_1, T_2):
        # Extract the translation components
        trans_1 = T_1[:3, 3]
        trans_2 = T_2[:3, 3]

        # Calculate the differences
        delta_x = trans_1[0] - trans_2[0]
        delta_y = trans_1[1] - trans_2[1]
        delta_z = trans_1[2] - trans_2[2]

        # print(f"Translation Differences (1 vs 2):")
        # print(f"  Δx: {delta_x:.3f} m")
        # print(f"  Δy: {delta_y:.3f} m")
        # print(f"  Δz: {delta_z:.3f} m")

        return delta_x, delta_y, delta_z

    def apply_low_pass_filter(self, raw_value, filtered_value, smoothing_factor):
        # Low-pass filter for reducing noise
        return smoothing_factor * filtered_value + (1 - smoothing_factor) * raw_value

    def set_starting_arm_position(self):
        while not rospy.is_shutdown():
            self.send_arm_command({
                    "joint_arm_l0": 0.0,
                    "joint_arm_l1": 0.0,
                    "joint_arm_l2": 0.0,
                    "joint_arm_l3": 0.0,
                    # "joint_left_wheel": 0.5,
                    "joint_lift": 0.5,
                    "joint_wrist_yaw": -0.0
                })
        
            self.rate.sleep()

    

    def run(self):
        for _ in range(2):  # Run exactly twice
            # Wait for manip_start signal
            while not self.manip_start_flag and not rospy.is_shutdown():
                print("Waiting for manip_start signal...")
                rospy.sleep(0.1)

            # time.sleep(10)  # Replace with actual task logic
            while not self.percep_done_flag and not rospy.is_shutdown():
                print("Waiting for percep_done signal...")
                # self.manip_start_flag = False  # Reset for the next cycle

            rospy.loginfo("Performing manipulation task...")
            # self.grab_sponge()

            # Send manip_done signal
            rospy.loginfo("Manipulation done! Sending manip_done signal...")
            
            # Publish manip_done for 5 seconds
            publish_duration = rospy.Duration(3)
            start_time = rospy.Time.now()
            while rospy.Time.now() - start_time < publish_duration and not rospy.is_shutdown():
                self.manip_done_pub.publish(Bool(data=True))
                self.manip_start_flag = False  # Reset for the next cycle
                self.percep_done_flag = False
                rospy.sleep(0.1)

        rospy.loginfo("Both manipulation tasks complete!")


if __name__ == "__main__":
    try:
        controller = StretchController()
        controller.run()
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass
