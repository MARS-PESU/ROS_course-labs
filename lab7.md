
# Setting up a robot simulation (Advanced)[](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Advanced.html#setting-up-a-robot-simulation-advanced "Link to this heading")

**Goal:** Extend a robot simulation with an obstacle avoider node.

**Tutorial level:** Advanced

**Time:** 20 minutes

Contents

- [Background](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Advanced.html#background)
    
- [Prerequisites](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Advanced.html#prerequisites)
    
- [Tasks](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Advanced.html#tasks)
    
    - [1 Updating `my_robot.urdf`](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Advanced.html#updating-my-robot-urdf)
        
    - [2 Creating a ROS node to avoid obstacles](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Advanced.html#creating-a-ros-node-to-avoid-obstacles)
        
    - [3 Updating additional files](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Advanced.html#updating-additional-files)
        
    - [4 Test the obstacle avoidance code](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Advanced.html#test-the-obstacle-avoidance-code)
        
- [Summary](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Advanced.html#summary)
    
- [Next steps](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Advanced.html#next-steps)
    

## [Background](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Advanced.html#id1)[](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Advanced.html#background "Link to this heading")

In this tutorial you will extend the package created in the first part of the tutorial: [Setting up a robot simulation (Basic)](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html). The aim is to implement a ROS 2 node that avoids obstacles using the robot’s distance sensors. This tutorial focuses on using robot devices with the `webots_ros2_driver` interface.

## [Prerequisites](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Advanced.html#id2)[](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Advanced.html#prerequisites "Link to this heading")

This is a continuation of the first part of the tutorial: [Setting up a robot simulation (Basic)](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html). It is mandatory to start with the first part to set up the custom packages and necessary files.

This tutorial is compatible with version 2023.1.0 of `webots_ros2` and Webots R2023b, as well as upcoming versions.

## [Tasks](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Advanced.html#id3)[](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Advanced.html#tasks "Link to this heading")

### [1 Updating `my_robot.urdf`](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Advanced.html#id4)[](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Advanced.html#updating-my-robot-urdf "Link to this heading")

As mentioned in [Setting up a robot simulation (Basic)](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html), `webots_ros2_driver` contains plugins to interface most of Webots devices with ROS 2 directly. These plugins can be loaded using the `<device>` tag in the URDF file of the robot. The `reference` attribute should match the Webots device `name` parameter. The list of all existing interfaces and the corresponding parameters can be found [on the devices reference page](https://github.com/cyberbotics/webots_ros2/wiki/References-Devices). For available devices that are not configured in the URDF file, the interface will be automatically created and default values will be used for ROS parameters (e.g. `update rate`, `topic name`, and `frame name`).

In `my_robot.urdf` replace the whole contents with:

Python
```xml
<?xml version="1.0" ?>
<robot name="My robot">
    <webots>
        <device reference="ds0" type="DistanceSensor">
            <ros>
                <topicName>/left_sensor</topicName>
                <alwaysOn>true</alwaysOn>
            </ros>
        </device>
        <device reference="ds1" type="DistanceSensor">
            <ros>
                <topicName>/right_sensor</topicName>
                <alwaysOn>true</alwaysOn>
            </ros>
        </device>
        <plugin type="my_package.my_robot_driver.MyRobotDriver" />
    </webots>
</robot>
```
In addition to your custom plugin, the `webots_ros2_driver` will parse the `<device>` tags referring to the **DistanceSensor** nodes and use the standard parameters in the `<ros>` tags to enable the sensors and name their topics.

### [2 Creating a ROS node to avoid obstacles](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Advanced.html#id5)[](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Advanced.html#creating-a-ros-node-to-avoid-obstacles "Link to this heading")

Python

The robot will use a standard ROS node to detect the wall and send motor commands to avoid it. In the `my_package/my_package/` folder, create a file named `obstacle_avoider.py` with this code:
``` python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

MAX_RANGE = 0.15

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')

        self.__publisher = self.create_publisher(Twist, 'cmd_vel', 1)

        self.create_subscription(Range, 'left_sensor', self.__left_sensor_callback, 1)
        self.create_subscription(Range, 'right_sensor', self.__right_sensor_callback, 1)

    def __left_sensor_callback(self, message):
        self.__left_sensor_value = message.range

    def __right_sensor_callback(self, message):
        self.__right_sensor_value = message.range

        command_message = Twist()

        command_message.linear.x = 0.1

        if self.__left_sensor_value < 0.9 * MAX_RANGE or self.__right_sensor_value < 0.9 * MAX_RANGE:
            command_message.angular.z = -2.0

        self.__publisher.publish(command_message)

def main(args=None):
    rclpy.init(args=args)
    avoider = ObstacleAvoider()
    rclpy.spin(avoider)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    avoider.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
```



This node will create a publisher for the command and subscribe to the sensors topics here:

```python
self.__publisher = self.create_publisher(Twist, 'cmd_vel', 1)

self.create_subscription(Range, 'left_sensor', self.__left_sensor_callback, 1)
self.create_subscription(Range, 'right_sensor', self.__right_sensor_callback, 1)

```

When a measurement is received from the left sensor it will be copied to a member field:
```python
def __left_sensor_callback(self, message):
    self.__left_sensor_value = message.range
```
Finally, a message will be sent to the `/cmd_vel` topic when a measurement from the right sensor is received. The `command_message` will register at least a forward speed in `linear.x` in order to make the robot move when no obstacle is detected. If any of the two sensors detect an obstacle, `command_message` will also register a rotational speed in `angular.z` in order to make the robot turn right.
```python
def __right_sensor_callback(self, message):
    self.__right_sensor_value = message.range

    command_message = Twist()

    command_message.linear.x = 0.1

    if self.__left_sensor_value < 0.9 * MAX_RANGE or self.__right_sensor_value < 0.9 * MAX_RANGE:
        command_message.angular.z = -2.0

    self.__publisher.publish(command_message)
```
### [3 Updating additional files](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Advanced.html#id6)[](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Advanced.html#updating-additional-files "Link to this heading")

You have to modify these two other files to launch your new node.

```Python

Edit `setup.py` and replace `'console_scripts'` with:

'console_scripts': [
    'my_robot_driver = my_package.my_robot_driver:main',
    'obstacle_avoider = my_package.obstacle_avoider:main'
],
```

This will add an entry point for the `obstacle_avoider` node.

Go to the file `robot_launch.py` and replace it with:
```python
import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

def generate_launch_description():
    package_dir = get_package_share_directory('my_package')
    robot_description_path = os.path.join(package_dir, 'resource', 'my_robot.urdf')

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'my_world.wbt')
    )

    my_robot_driver = WebotsController(
        robot_name='my_robot',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )

    obstacle_avoider = Node(
        package='my_package',
        executable='obstacle_avoider',
    )

    return LaunchDescription([
        webots,
        my_robot_driver,
        obstacle_avoider,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
```
This will create an `obstacle_avoider` node that will be included in the `LaunchDescription`.

### [4 Test the obstacle avoidance code](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Advanced.html#id7)[](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Advanced.html#test-the-obstacle-avoidance-code "Link to this heading")

Launch the simulation from a terminal in your ROS 2 workspace:

Linux

From a terminal in your ROS 2 workspace run:
```bash
colcon build
source install/local_setup.bash
ros2 launch my_package robot_launch.py
```
Your robot should go forward and before hitting the wall it should turn clockwise. You can press `Ctrl+F10` in Webots or go to the `View` menu, `Optional Rendering` and `Show DistanceSensor Rays` to display the range of the distance sensors of the robot.

![../../../../_images/Robot_turning_clockwise.png](https://docs.ros.org/en/humble/_images/Robot_turning_clockwise.png)

## [Summary](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Advanced.html#id8)[](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Advanced.html#summary "Link to this heading")

In this tutorial, you extended the basic simulation with a obstacle avoider ROS 2 node that publishes velocity commands based on the distance sensor values of the robot.
