
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
    
# Setting up a robot simulation (Basic)[](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html#setting-up-a-robot-simulation-basic "Link to this heading")

**Goal:** Setup a robot simulation and control it from ROS 2.

**Tutorial level:** Advanced

**Time:** 30 minutes

Contents

- [Background](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html#background)
    
- [Prerequisites](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html#prerequisites)
    
- [Tasks](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html#tasks)
    
    - [1 Create the package structure](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html#create-the-package-structure)
        
    - [2 Setup the simulation world](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html#setup-the-simulation-world)
        
    - [3 Edit the `my_robot_driver` plugin](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html#edit-the-my-robot-driver-plugin)
        
    - [4 Create the `my_robot.urdf` file](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html#create-the-my-robot-urdf-file)
        
    - [5 Create the launch file](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html#create-the-launch-file)
        
    - [6 Edit additional files](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html#edit-additional-files)
        
    - [7 Test the code](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html#test-the-code)
        
- [Summary](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html#summary)
    
- [Next steps](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html#next-steps)
    

## [Background](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html#id3)[](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html#background "Link to this heading")

In this tutorial, you are going to use the Webots robot simulator to set-up and run a very simple ROS 2 simulation scenario.

The `webots_ros2` package provides an interface between ROS 2 and Webots. It includes several sub-packages, but in this tutorial, you are going to use only the `webots_ros2_driver` sub-package to implement a Python or C++ plugin controlling a simulated robot. Some other sub-packages contain demos with different robots such as the TurtleBot3. They are documented in the [Webots ROS 2 examples](https://github.com/cyberbotics/webots_ros2/wiki/Examples) page.

## [Prerequisites](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html#id4)[](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html#prerequisites "Link to this heading")

It is recommended to understand basic ROS principles covered in the beginner [Tutorials](https://docs.ros.org/en/humble/Tutorials.html). In particular, [Using turtlesim, ros2, and rqt](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html), [Understanding topics](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html), [Creating a workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html), [Creating a package](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html) and [Creating a launch file](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html) are useful prerequisites.

Linux
The Linux and ROS commands of this tutorial can be run in a standard Linux terminal. The following page [Installation (Ubuntu)](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Installation-Ubuntu.html) explains how to install the `webots_ros2` package on Linux.

This tutorial is compatible with version 2023.1.0 of `webots_ros2` and Webots R2023b, as well as upcoming versions.

## [Tasks](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html#id5)[](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html#tasks "Link to this heading")

### [1 Create the package structure](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html#id6)[](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html#create-the-package-structure "Link to this heading")

Let’s organize the code in a custom ROS 2 package. Create a new package named `my_package` from the `src` folder of your ROS 2 workspace. Change the current directory of your terminal to `ros2_ws/src` and run:

```Python

ros2 pkg create --build-type ament_python --license Apache-2.0 --node-name my_robot_driver my_package --dependencies rclpy geometry_msgs webots_ros2_driver
```
The `--node-name my_robot_driver` option will create a `my_robot_driver.py` template Python plugin in the `my_package` subfolder that you will modify later. The `--dependencies rclpy geometry_msgs webots_ros2_driver` option specifies the packages needed by the `my_robot_driver.py` plugin in the `package.xml` file.

Let’s add a `launch` and a `worlds` folder inside the `my_package` folder.
```
cd my_package
mkdir launch
mkdir worlds
```
You should end up with the following folder structure:

src/
└── my_package/
    ├── launch/
    ├── my_package/
    │   ├── __init__.py
    │   └── my_robot_driver.py
    ├── resource/
    │   └── my_package
    ├── test/
    │   ├── test_copyright.py
    │   ├── test_flake8.py
    │   └── test_pep257.py
    ├── worlds/
    ├── package.xml
    ├── setup.cfg
    └── setup.py

### [2 Setup the simulation world](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html#id7)[](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html#setup-the-simulation-world "Link to this heading")

You will need a world file containing a robot to launch your simulation. [`Download this world file`](https://docs.ros.org/en/humble/_downloads/5ad123fc6a8f1ea79553d5039728084a/my_world.wbt) and move it inside `my_package/worlds/`.

This is actually a fairly simple text file you can visualize in a text editor. A simple robot is already included in this `my_world.wbt` world file.

Note

In case you want to learn how to create your own robot model in Webots, you can check this [tutorial](https://cyberbotics.com/doc/guide/tutorial-6-4-wheels-robot).

### [3 Edit the `my_robot_driver` plugin](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html#id8)[](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html#edit-the-my-robot-driver-plugin "Link to this heading")

The `webots_ros2_driver` sub-package automatically creates a ROS 2 interface for most sensors. More details on existing device interfaces and how to configure them is given in the second part of the tutorial: [Setting up a robot simulation (Advanced)](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Advanced.html). In this task, you will extend this interface by creating your own custom plugin. This custom plugin is a ROS node equivalent to a robot controller. You can use it to access the [Webots robot API](https://cyberbotics.com/doc/reference/robot?tab-language=python) and create your own topics and services to control your robot.

Note

The purpose of this tutorial is to show a basic example with a minimum number of dependencies. However, you could avoid the use of this plugin by using another `webots_ros2` sub-package named `webots_ros2_control`, introducing a new dependency. This other sub-package creates an interface with the `ros2_control` package that facilitates the control of a differential wheeled robot.


Open `my_package/my_package/my_robot_driver.py` in your favorite editor and replace its contents with the following:
```python
import rclpy
from geometry_msgs.msg import Twist

HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025

class MyRobotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__left_motor = self.__robot.getDevice('left wheel motor')
        self.__right_motor = self.__robot.getDevice('right wheel motor')

        self.__left_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0)

        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0)

        self.__target_twist = Twist()

        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_robot_driver')
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        forward_speed = self.__target_twist.linear.x
        angular_speed = self.__target_twist.angular.z

        command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        self.__left_motor.setVelocity(command_motor_left)
        self.__right_motor.setVelocity(command_motor_right)
```
As you can see, the `MyRobotDriver` class implements three methods.

The first method, named `init(self, ...)`, is actually the ROS node counterpart of the Python `__init__(self, ...)` constructor. The `init` method always takes two arguments:

- The `webots_node` argument contains a reference on the Webots instance.
    
- The `properties` argument is a dictionary created from the XML tags given in the URDF files ([4 Create the my_robot.urdf file](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html#create-the-my-robot-urdf-file)) and allows you to pass parameters to the controller.
    

The robot instance from the simulation `self.__robot` can be used to access the [Webots robot API](https://cyberbotics.com/doc/reference/robot?tab-language=python). Then, it gets the two motor instances and initializes them with a target position and a target velocity. Finally a ROS node is created and a callback method is registered for a ROS topic named `/cmd_vel` that will handle `Twist` messages.
```python
def init(self, webots_node, properties):
    self.__robot = webots_node.robot

    self.__left_motor = self.__robot.getDevice('left wheel motor')
    self.__right_motor = self.__robot.getDevice('right wheel motor')

    self.__left_motor.setPosition(float('inf'))
    self.__left_motor.setVelocity(0)

    self.__right_motor.setPosition(float('inf'))
    self.__right_motor.setVelocity(0)

    self.__target_twist = Twist()

    rclpy.init(args=None)
    self.__node = rclpy.create_node('my_robot_driver')
    self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)
```
Then comes the implementation of the `__cmd_vel_callback(self, twist)` callback private method that will be called for each `Twist` message received on the `/cmd_vel` topic and will save it in the `self.__target_twist` member variable.
```python
def __cmd_vel_callback(self, twist):
    self.__target_twist = twist
```
Finally, the `step(self)` method is called at every time step of the simulation. The call to `rclpy.spin_once()` is needed to keep the ROS node running smoothly. At each time step, the method will retrieve the desired `forward_speed` and `angular_speed` from `self.__target_twist`. As the motors are controlled with angular velocities, the method will then convert the `forward_speed` and `angular_speed` into individual commands for each wheel. This conversion depends on the structure of the robot, more specifically on the radius of the wheel and the distance between them.
```python
def step(self):
    rclpy.spin_once(self.__node, timeout_sec=0)

    forward_speed = self.__target_twist.linear.x
    angular_speed = self.__target_twist.angular.z

    command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
    command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

    self.__left_motor.setVelocity(command_motor_left)
    self.__right_motor.setVelocity(command_motor_right)
```


### [4 Create the `my_robot.urdf` file](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html#id9)[](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html#create-the-my-robot-urdf-file "Link to this heading")

You now have to create a URDF file to declare the `MyRobotDriver` plugin. This will allow the `webots_ros2_driver` ROS node to launch the plugin and connect it to the target robot.

In the `my_package/resource` folder create a text file named `my_robot.urdf` with this content:

```Python

<?xml version="1.0" ?>
<robot name="My robot">
    <webots>
        <plugin type="my_package.my_robot_driver.MyRobotDriver" />
    </webots>
</robot>
```
The `type` attribute specifies the path to the class given by the hierarchical structure of files. `webots_ros2_driver` is responsible for loading the class based on the specified package and modules.

Note

This simple URDF file doesn’t contain any link or joint information about the robot as it is not needed in this tutorial. However, URDF files usually contain much more information as explained in the [URDF](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html) tutorial.

Note

Here the plugin does not take any input parameter, but this can be achieved with a tag containing the parameter name.

```Python

<plugin type="my_package.my_robot_driver.MyRobotDriver">
    <parameterName>someValue</parameterName>
</plugin>
```
This is namely used to pass parameters to existing Webots device plugins (see [Setting up a robot simulation (Advanced)](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Advanced.html)).

### [5 Create the launch file](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html#id10)[](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html#create-the-launch-file "Link to this heading")

Let’s create the launch file to easily launch the simulation and the ROS controller with a single command. In the `my_package/launch` folder create a new text file named `robot_launch.py` with this code:
```python
import os
import launch
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

    return LaunchDescription([
        webots,
        my_robot_driver,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
```
The `WebotsLauncher` object is a custom action that allows you to start a Webots simulation instance. You have to specify in the constructor which world file the simulator will open.

```
webots = WebotsLauncher(
    world=os.path.join(package_dir, 'worlds', 'my_world.wbt')
)
```

Then, the ROS node interacting with the simulated robot is created. This node, named `WebotsController`, is located in the `webots_ros2_driver` package.

Linux

The node will be able to communicate with the simulated robot by using a custom protocol based on IPC and shared memory.

In your case, you need to run a single instance of this node, because you have a single robot in the simulation. But if you had more robots in the simulation, you would have to run one instance of this node per robot. The `robot_name` parameter is used to define the name of the robot the driver should connect to. The `robot_description` parameter holds the path to the URDF file which refers to the `MyRobotDriver` plugin. You can see the `WebotsController` node as the interface that connects your controller plugin to the target robot.
```python
my_robot_driver = WebotsController(
    robot_name='my_robot',
    parameters=[
        {'robot_description': robot_description_path},
    ]
)
```

After that, the two nodes are set to be launched in the `LaunchDescription` constructor:

```python
return LaunchDescription([
    webots,
    my_robot_driver,
```

Finally, an optional part is added in order to shutdown all the nodes once Webots terminates (e.g. when it gets closed from the graphical user interface).

```python
launch.actions.RegisterEventHandler(
    event_handler=launch.event_handlers.OnProcessExit(
        target_action=webots,
        on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
    )
)
```
Note

More details on `WebotsController` and `WebotsLauncher` arguments can be found [on the nodes reference page](https://github.com/cyberbotics/webots_ros2/wiki/References-Nodes).

### [6 Edit additional files](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html#id11)[](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html#edit-additional-files "Link to this heading")

Python

Before you can start the launch file, you have to modify the `setup.py` file to include the extra files you added. Open `my_package/setup.py` and replace its contents with:

```python
from setuptools import setup

package_name = 'my_package'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/robot_launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/my_world.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/my_robot.urdf']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user.name@mail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_robot_driver = my_package.my_robot_driver:main',
        ],
    },
)
```
This sets-up the package and adds in the `data_files` variable the newly added files: `my_world.wbt`, `my_robot.urdf` and `robot_launch.py`.

### [7 Test the code](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html#id12)[](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html#test-the-code "Link to this heading")

LinuxWindowsmacOS

From a terminal in your ROS 2 workspace run:
```bash
colcon build
source install/local_setup.bash
ros2 launch my_package robot_launch.py
```

This will launch the simulation. Webots will be automatically installed on the first run in case it was not already installed.

Note

If you want to install Webots manually, you can download it [here](https://github.com/cyberbotics/webots/releases/latest).

Then, open a second terminal and send a command with:

ros2 topic pub /cmd_vel geometry_msgs/Twist  "linear: { x: 0.1 }"

The robot is now moving forward.

![../../../../_images/Robot_moving_forward.png](https://docs.ros.org/en/humble/_images/Robot_moving_forward.png)

At this point, the robot is able to blindly follow your motor commands. But it will eventually bump into the wall as you order it to move forwards.

![../../../../_images/Robot_colliding_wall.png](https://docs.ros.org/en/humble/_images/Robot_colliding_wall.png)

Close the Webots window, this should also shutdown your ROS nodes started from the launcher. Close also the topic command with `Ctrl+C` in the second terminal.


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
