#!/usr/bin/env python3
import rospy
import math
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

WAYPOINTS = [
    # (x, y, yaw_in_degrees) - Converted to radians automatically
    (-1.68, 0.27, 168.1),
    (-3.18, 0.30, 234.7),
    (-3.08, -0.59, 325.3),

    # (-2.06, -0.21, 15.2),
    # (-1.34, 1.09, 94.0),
    # (-1.26, 2.45, 141.8),
]

MOVE_SPEED = 1  # Change speed 
TIMEOUT = 30.0    # Max time per waypoint (seconds)

class StretchNavigator:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Connecting to move_base...")
        self.client.wait_for_server()
        rospy.loginfo("Connected!")
        
    def navigate_to(self, x, y, yaw_degrees):
        """Move to position (x,y) and rotate to face yaw_degrees"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        # Set position
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        
        # Convert yaw (degrees) to quaternion
        yaw_rad = math.radians(yaw_degrees)
        q = quaternion_from_euler(0, 0, yaw_rad)
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        
        rospy.loginfo(f"\n=== New Goal ===\nPosition: ({x:.2f}, {y:.2f})\nFacing: {yaw_degrees}°")
        self.client.send_goal(goal)
        
        # Wait with timeout
        finished = self.client.wait_for_result(rospy.Duration(TIMEOUT))
        state = self.client.get_state()
        if finished and state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Reached goal successfully!")
        else:
            rospy.logwarn(f"Navigation failed or timed out. State: {state}")
            self.client.cancel_goal()

    

# ========== MAIN ==========
if __name__ == '__main__':
    try:
        rospy.init_node('stretch_waypoint_navigator')
        navigator = StretchNavigator()
        
        rospy.loginfo(f"Starting navigation with {len(WAYPOINTS)} waypoints...")
        for x, y, yaw in WAYPOINTS:
            navigator.navigate_to(x, y, yaw)
            #rospy.sleep(1)  # Brief pause between waypoints
            
        rospy.loginfo("All waypoints completed!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation stopped by user")
