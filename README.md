# Inventory performing robotont 

Thesis by Robina Zvirgzdina on controling robotont to perform inventory in the Univeristy of Tartu library sing using a rfid reader, ROS2 platform and a height adjustment system. 
 
## How to get started

### Setting up

To get the IP on the robot run: 

```
ifconfig
```

Then for visualizing purpose remotely run:

```
ssh -X user@IP
```

For running the rest of the code, either a simple ssh connection can be formed, or VScode remote connection can be used.

### Mapping 

in the terminal for visualization from the colcon_ws run:

```
rviz2 navrviz.rviz
```

in other terminal / remotely connected VScode:
```
ros2 launch robina_thesis mapping_launch.py

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Using keyboard navigate through the area to generate a map. Save and serialize the map using the slam toolbox plugin under the name "map"

### Navigate using the generated map

Sometimes the ports differ from the ones shown in the code. To ckeck this run: 
```
ls /dev/tty*
```
to show which device corresponds to which port. Then change the ports in hieght_adjuster_node.py and rfid_reader_node.py to the correct ports.


Another easier solution is to disconnect everything and connect in the following order
1. lidar
2. RFID reader
3. Stepper motor

Now start the navigation node and RFID reader:
```
ros2 launch robina_thesis all_systems_launch.py

ros2 run robina_rfid big_node
```

and in the visualization terminal run:
```
rviz2 navrviz.rviz
```

Give 4 point along the shelf using clicked point in RViz2.


# Result

After generating map, running the navigation and giving the navigation points, the robot will drive trhought the points and on its way scan the RFID tags. It will then create a heat-map for each of the tags to display their most probable location and height. It is possible to also later visualize these tags on the map using:

```
ros2 run robina_rfid marker_publisher
```
