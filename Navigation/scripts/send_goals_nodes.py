#!/usr/bin/env python3
import rospy
import math
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from std_msgs.msg import Bool

WAYPOINTS = [
    # (x, y, yaw_in_degrees) - Converted to radians automatically
    (1.02, -0.01, 360),    # Position A 
    (1.05, -1.02, 270), # Position B
    (0.01, -0.94, 180)  # Position C
    #(-0.12, -0.18, 270)  # Position D 
]

MOVE_SPEED = 1  # Change speed 
TIMEOUT = 5.0    # Max time per waypoint (seconds)

class StretchNavigator:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Connecting to move_base...")
        self.client.wait_for_server()
        rospy.loginfo("Connected!")
        
        # Publishers and subscribers for manipulation 
        self.manip_start_pub = rospy.Publisher('/manipulation_start', Bool, queue_size=10)
        self.manip_done_sub = rospy.Subscriber('/manipulation_done', Bool, self.manip_done_callback)
        self.manip_done_flag = False
        
    def manip_done_callback(self, msg):
        if msg.data:
            rospy.loginfo("Manipulation done signal received.")
            self.manip_done_flag = True
            
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

    def run(self):
        for idx, (x, y, yaw) in enumerate(WAYPOINTS):
            rospy.loginfo(f"\n=== Waypoint {idx+1}/{len(WAYPOINTS)} ===")
            self.navigate_to(x, y, yaw)

            # Publish start signal for manipulation
            self.manip_done_flag = False
            rospy.sleep(0.5)  # Short buffer to ensure subscribers connect
            rospy.loginfo("Publishing start signal for manipulation.")
            self.manip_start_pub.publish(Bool(data=True))

            # Wait for manipulation to complete
            rospy.loginfo("Waiting for manipulation to finish...")
            timeout_time = rospy.Time.now() + rospy.Duration(30)  # max wait time
            while not self.manip_done_flag and rospy.Time.now() < timeout_time and not rospy.is_shutdown():
                rospy.sleep(0.1)

            if not self.manip_done_flag:
                rospy.logwarn("Manipulation timeout or no response.")

        rospy.loginfo("\nAll waypoints processed. Mission complete!")
        
# ========== MAIN ==========
if __name__ == '__main__':
    try:
        rospy.init_node('stretch_waypoint_navigator')
        nav = StretchNavigator()
        nav.run()
	
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation stopped")
