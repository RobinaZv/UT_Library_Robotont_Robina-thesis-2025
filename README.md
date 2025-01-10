# Inventory performing robotont 

Thesis by Robina Zvirgzdina controling robotont to perform inventory in the library of Univeristy of Tartu using rfid reader, nav2 for navigation and a height adjustment system

## How to get started

### First mapping needs to be done

On the robot run:

```
ros2 launch robotont_driver example_launch.py

ros2 launch slam_toolbox online_async_launch.py
```

Save and serialize the map using the slam toolbox plugin 

### Using generated map can navigate

On the robot run:

```
source env/bin/activate

ros2 launch robina_thesis combined_launch.py

ros2 launch nav2_bringup navigation_launch.py
```

At first set initial pose, then can set goal pose or use Nav2 plugin for multiple points.

# Result

After generating map, running the navigation, the robot will drive to the set destination and on its way scan the rfid tags. It will put the tag number and time when the tag was scanned in a file. 

Simulteniously, rosbag will record robots location so that afterwards it will be possible to see where robot was located when it scanned the book. 
