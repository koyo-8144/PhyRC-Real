#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

def pose_callback(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    quat = msg.pose.pose.orientation
    (_, _, yaw_rad) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
    yaw_deg = math.degrees(yaw_rad)

    # Wrap yaw to [0, 360)
    yaw_deg = yaw_deg % 360

    print(f"WAYPOINT = ({x:.2f}, {y:.2f}, {yaw_deg:.1f})")

rospy.init_node('pose_logger')
rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_callback)
rospy.loginfo("Move the robot in RViz using '2D Pose Estimate' and check terminal for waypoints.")
rospy.spin()

