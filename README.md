From Gazebo GUI, you can spawn "walking person" model as shown in the following image.
![image](https://github.com/user-attachments/assets/fa5eb445-1b0f-48dd-a7c2-b9f8992bc962)

After creating the world, you can save it by selection "File" -> "Save World As"
![image](https://github.com/user-attachments/assets/63c95f1b-ac60-4e57-b425-357e152c5fa6)

To upload the world next time you launch gazebo, you can specify the saved world in launch file (gazebo.launch in this case) as follows:
![image](https://github.com/user-attachments/assets/5ae549b5-3931-4d8f-b6f3-cc32d035ce5a)

Creating the map in simulation:
To launch the simulation with an environment:
roslaunch stretch_navigation mapping_gazebo.launch gazebo_world:=worlds/willowgarage.world

Drive the robot around following instructions in stretch_navigation to have a simple map:
roslaunch stretch_core teleop_twist.launch twist_topic:=/stretch_diff_drive_controller/cmd_vel linear:=1.0 angular:=2.0 teleop_type:=keyboard # or use teleop_type:=joystick if you have a controller

Save map:
mkdir -p ~/stretch_user/maps
rosrun map_server map_saver -f ${HELLO_FLEET_PATH}/maps/<map_name>

After having the map, run:
roslaunch stretch_navigation navigation_gazebo.launch gazebo_world:=worlds/willowgarage.world map_yaml:=$(rospack find stretch_navigation)/maps/<map_name>

get_pose.py will return current position(x, y, yaw), In rviz, using "2D Pose Estimate" to the position and rotation specified, send_goals.py will return the next position. 
rosrun stretch_navigation get_pose.py

We can update send_goals.py with the position from get_pose.py. Before running send_goals.py, restart the gazebo, as Rviz and gazebo will not map using "2D Pose Estimate" as discussed.
rosrun stretch_navigation send_goals.py

send_goals_nodes.py has one more section of publishers and subscribers for manipulation
stretch_navigation send_goals_nodes.py 
