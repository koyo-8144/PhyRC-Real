From Gazebo GUI, you can spawn "walking person" model as shown in the following image.
![image](https://github.com/user-attachments/assets/fa5eb445-1b0f-48dd-a7c2-b9f8992bc962)

After creating the world, you can save it by selection "File" -> "Save World As"
![image](https://github.com/user-attachments/assets/63c95f1b-ac60-4e57-b425-357e152c5fa6)

To upload the world next time you launch gazebo, you can specify the saved world in launch file (gazebo.launch in this case) as follows:
<!-- <arg name="world" default="worlds/empty.world"/> -->
<arg name="world" default="/home/sandisk/koyo_ws/stretch_ws/src/stretch_ros/phyrc/scripts/world/human_sleep"/>
