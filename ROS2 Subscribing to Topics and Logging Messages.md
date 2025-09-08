
## Part 1: Running a Simple Demo Project (Turtlesim)

First, let’s use the classic `turtlesim` package, which comes with ROS 2.

### Install turtlesim (if not already installed)

```bash
sudo apt install ros-humble-turtlesim
```

### Run the turtlesim simulator

```bash
ros2 run turtlesim turtlesim_node
```

You should see a window with a turtle.

### Control the turtle

Open another terminal and run:

```bash
ros2 run turtlesim turtle_teleop_key
```

Now you can move the turtle with your keyboard.

Check what topics are being published:

```bash
ros2 topic list
```

You’ll see topics like:

```
/turtle1/cmd_vel
/turtle1/pose
```

---

## Part 2: Creating a Custom Logger Package

Now let’s create our own ROS 2 package that subscribes to these topics and logs them to a file.

### Step 1: Create the package

Navigate to your ROS 2 workspace (`~/ros2_ws/src`):

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python turtle_logger
```

This creates a Python package.

### Step 2: Write the subscriber node

Open `turtle_logger/turtle_logger/turtle_logger_node.py` and add:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import datetime

class TurtleLogger(Node):
    def __init__(self):
        super().__init__('turtle_logger')

        # Open a log file
        self.log_file = open("turtle_log.txt", "a")

        # Subscribe to cmd_vel
        self.create_subscription(Twist, '/turtle1/cmd_vel', self.cmd_vel_callback, 10)
        
        # Subscribe to pose
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

    def cmd_vel_callback(self, msg: Twist):
        log_entry = f"[{datetime.datetime.now()}] CMD_VEL -> linear: {msg.linear.x:.2f}, angular: {msg.angular.z:.2f}\n"
        self.log_file.write(log_entry)
        self.log_file.flush()

    def pose_callback(self, msg: Pose):
        log_entry = f"[{datetime.datetime.now()}] POSE -> x: {msg.x:.2f}, y: {msg.y:.2f}, theta: {msg.theta:.2f}\n"
        self.log_file.write(log_entry)
        self.log_file.flush()

def main(args=None):
    rclpy.init(args=args)
    node = TurtleLogger()
    rclpy.spin(node)
    node.log_file.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 3: Update `setup.py`

In `turtle_logger/setup.py`, update `entry_points`:

```python
entry_points={
    'console_scripts': [
        'turtle_logger_node = turtle_logger.turtle_logger_node:main',
    ],
},
```

### Step 4: Build the package

From your workspace root:

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Step 5: Run the logger

Run turtlesim again:

```bash
ros2 run turtlesim turtlesim_node
```

In another terminal, run teleop:

```bash
ros2 run turtlesim turtle_teleop_key
```

Now run your logger:

```bash
ros2 run turtle_logger turtle_logger_node
```

Check the log file:

```bash
cat turtle_log.txt
```

You’ll see entries like:

```
[2025-09-09 00:12:34.567890] CMD_VEL -> linear: 2.00, angular: 0.50
[2025-09-09 00:12:35.123456] POSE -> x: 5.50, y: 3.20, theta: 0.75
```

---

✅ Now you’ve run a standard ROS package and created your own custom logging node that subscribes to topics and writes messages to a file.



## MAIN TASK:

### `turtlebot3` (Most practical, many topics)

TurtleBot3 is widely used and has **a rich set of topics** for sensors and movement.

Install:

```bash
sudo apt install ros-humble-turtlebot3*
```

Launch simulation:

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

Check topics:

```bash
ros2 topic list
```

You’ll see many:

```
/camera/depth/image_raw
/camera/rgb/image_raw
/cmd_vel
/odom
/scan
/joint_states
/tf
```

Perfect for logging exercises 🚀

#### go ahead and make the logger for any of the 3 topics