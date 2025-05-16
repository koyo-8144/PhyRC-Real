From Gazebo GUI, you can spawn "walking person" model as shown in the following image.
![image](https://github.com/user-attachments/assets/fa5eb445-1b0f-48dd-a7c2-b9f8992bc962)

After creating the world, you can save it by selection "File" -> "Save World As"
![image](https://github.com/user-attachments/assets/63c95f1b-ac60-4e57-b425-357e152c5fa6)

To upload the world next time you launch gazebo, you can specify the saved world in launch file (gazebo.launch in this case) as follows:
![image](https://github.com/user-attachments/assets/5ae549b5-3931-4d8f-b6f3-cc32d035ce5a)

Launch Simulation with a World
'roslaunch stretch_navigation mapping_gazebo.launch gazebo_world:=worlds/willowgarage.world'

Creating the map in simulation:
To launch the simulation with an environment:

'roslaunch stretch_navigation mapping_gazebo.launch gazebo_world:=worlds/willowgarage.world'


Drive the robot around following instructions in stretch_navigation to have a simple map:

'roslaunch stretch_core teleop_twist.launch twist_topic:=/stretch_diff_drive_controller/cmd_vel linear:=1.0 angular:=2.0 teleop_type:=keyboard # or use teleop_type:=joystick if you have a controller'


