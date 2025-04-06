# Inventory performing robotont 

Thesis by Robina Zvirgzdina on controling robotont to perform inventory in the Univeristy of Tartu library sing using a rfid reader, ROS2 platform and a height adjustment system. 
 
## How to get started

### Setting up

To get the IP on the robot run 

```
ifconfig
```

then remotely run

```
ssh -X user@IP
```
### Mapping 

Remotely run the following commands and using keyboard navigate through the area to generate a map:

```
ros2 launch robina_thesis mapping_launch.py

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Save and serialize the map using the slam toolbox plugin 

### Navigate using the generated map

run to check which device corresponds to which port 
```
ls /dev/tty*
```
In the file _/src/robina__rfid/robina__rfid/rfid__reader.py_ add the correct port for the RFID reader. In the file _/src/robina__rfid/robina__rfid/height__changer__node.py_ change the port to correspond to the stepper motor.

Now start the navigation node and RFID reader:
```
ros2 launch robina_thesis navigation_launch.py

ros2 run robina_rfid big_node

ros2 launch robina_thesis navigation_launch.py
```

Give waypoints to the robot using Nav2 Goal plugin. 


# Result

After generating map, running the navigation and giving the waypoints, the robot will drive to the set destination and on its way scan the RFID tags. It will then create a heat-map for each of the tags to display their most probable location. It is possible to visualize these tags on the map using:

```
ros2 run robina_rfid marker_publisher
```
