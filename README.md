# Inventory performing robotont 

Thesis by Robina Zvirgzdina on controling robotont to perform inventory in the Univeristy of Tartu library sing using a rfid reader, ROS2 platform and a height adjustment system. 
 
## How to get started

### First mapping needs to be done

On the robot run:

```
ros2 launch robotont_driver example_launch.py

ros2 launch slam_toolbox online_async_launch.py
```

Save and serialize the map using the slam toolbox plugin 

### Navigate using the generated map

On the robot run:

```
source env/bin/activate

ros2 launch robina_thesis combined_launch.py

ros2 launch nav2_bringup navigation_launch.py
```

Using SSH connect remotely to the robot: 

```
ssh -X user@ip
```

remotely open rviz2 and add map as fixed frame.For displaying map choose topic --> Durability Policy --> Transient local
At first set initial pose, then can set goal pose or use Nav2 plugin for multiple points.

# Result

After generating map, running the navigation, the robot will drive to the set destination and on its way scan the rfid tags. It will put the tag number and time when the tag was scanned in a file. 

Simulteniously, rosbag will record robots location so that afterwards it will be possible to see where robot was located when it scanned the book. 
