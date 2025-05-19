#!/usr/bin/env python3
import rospy
import math
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from std_msgs.msg import Bool

WAYPOINTS_1 = [
    (-1.68, 0.27, 168.1),
    (-3.18, 0.30, 234.7),
    (-3.08, -0.59, 325.3),
]

WAYPOINTS_2 = [
    # (-2.06, -0.21, 15.2),
    (-1.34, 1.09, 94.0),
    (-1.26, 2.45, 141.8),
]

MOVE_SPEED = 1  # Change speed
TIMEOUT = 30.0  # Max time per waypoint (seconds)

class StretchNavigator:
    def __init__(self):
        rospy.init_node("stretch_waypoint_navigator")

        # Action client for navigation
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Connecting to move_base...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base!")

        # Publisher for manip_start
        self.manip_start_pub = rospy.Publisher("/manipulation_start", Bool, queue_size=10)
        
        # Subscriber for manip_done
        rospy.Subscriber("/manipulation_done", Bool, self.manip_done_callback)
        self.manip_done_flag = False

    def manip_done_callback(self, msg):
        if msg.data:
            # rospy.loginfo("Received manip_done signal.")
            self.manip_done_flag = True
            
    def navigate_to(self, x, y, yaw_degrees):
        """Move to position (x,y) and rotate to face yaw_degrees."""
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

        # Wait for navigation to complete
        finished = self.client.wait_for_result(rospy.Duration(TIMEOUT))
        state = self.client.get_state()
        if finished and state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Reached goal successfully!")
            return True
        else:
            rospy.logwarn(f"Navigation failed or timed out. State: {state}")
            self.client.cancel_goal()
            return False


    def run(self):
        # for waypoints in [WAYPOINTS_1, WAYPOINTS_2]:
        for waypoints in [WAYPOINTS_2]:
            for idx, (x, y, yaw) in enumerate(waypoints):
                rospy.loginfo(f"\n=== Waypoint {idx+1}/{len(waypoints)} ===")
                # Navigate to the waypoint
                self.navigate_to(x, y, yaw)

            # Send manip_start signal
            rospy.loginfo("Sending manip_start signal...")
            self.manip_done_flag = False
            
            # Publish the manip_start signal for a fixed duration
            publish_duration = rospy.Duration(3)  # 5 seconds
            start_time = rospy.Time.now()
            while rospy.Time.now() - start_time < publish_duration and not rospy.is_shutdown():
                self.manip_start_pub.publish(Bool(data=True))
                rospy.sleep(0.1) 

            # Wait for the manip_done signal with a timeout
            # timeout_time = rospy.Time.now() + rospy.Duration(30)  # Max wait time
            # while not self.manip_done_flag and rospy.Time.now() < timeout_time and not rospy.is_shutdown():
            while not self.manip_done_flag and not rospy.is_shutdown():
                rospy.loginfo("Waiting for manip_done signal...")
                rospy.sleep(0.1)

            if self.manip_done_flag:
                rospy.loginfo("Received manip_done signal. Moving to next waypoint.")
            else:
                rospy.logwarn("Manipulation timed out or no response.")

        rospy.loginfo("\nAll waypoints processed. Mission complete!")


        
if __name__ == "__main__":
    nav = StretchNavigator()
    nav.run()