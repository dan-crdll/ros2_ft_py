# First Steps
In this tutorial I show how to start with ROS2 (I assume it is already installed, so no installation guide which is 
already available on the official website).

### Contents
- [Creating the first (useless) node](#creating-the-first-useless-node)
- [Creating a node with a callback](#creating-a-node-with-a-callback)
- [References](#references)

### Creating the first (useless) node
First thing, open terminal and go to the folder which will be your workspace (it can be this folder) and create a 
directory called `src`. After having created that run:
```bash
colcon build
```
other directories will be created. Go into the `src` directory and create a package:
```bash
ros2 pkg create --build-type ament_python --dependencies rclpy my_first_package
```
Now you will se a new directory inside `src` called `my_first_package`. Inside it you will find some configuration files and another 
directory called `my_first_package` which contains the code of your package, here you will create the scripts for your nodes.

Create a python file called `my_first_node.py` inside the `my_first_package` directory:
```python
import rclpy
from rclpy.node import Node

class MyFirstNode(Node):
    def __init__(self):
        super().__init__("my_first_node")

        self.get_logger().info("My First Node Created!")

def main(args=None):
    rclpy.init(args=args)
    node = MyFirstNode()
    rclpy.shutdown()
```
Then go to `setup.py` and add the entry point for your node:
```python
entry_points={
        'console_scripts': [
            'my_first_node = my_first_package.my_first_node:main'
        ],
    },
```   
go back to root of your workspace and run:
```bash
colcon build --symlink-install
```
now you built your package and created a symlink to your code, so you can edit it without rebuilding the package every time.

To run the node, first source setup in install directory:
```bash
source install/setup.bash
```
Then run the node:
```bash
ros2 run my_first_package my_first_node
```
and you will see something like:
```bash
[INFO] [my_first_node]: My First Node Created!
```

### Creating a node with a callback
To create a new node add a python file called `my_second_node.py` inside the `my_first_package` directory:
```python
import rclpy
from rclpy.node import Node

class MySecondNode(Node):
    def __init__(self):
        super().__init__("my_second_node")

        self.get_logger().info("My Second Node Created!")

        timer_period = 2.0  # seconds
        self.counter = 0
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info(f"Timer callback executed {self.counter} times!")
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = MySecondNode()
    rclpy.spin(node)
    rclpy.shutdown()
```
Then go to `setup.py` and add the entry point for your node:
```python
entry_points={
        'console_scripts': [
            'my_first_node = my_first_package.my_first_node:main',
            'my_second_node = my_first_package.my_second_node:main'
        ],
    },
```
go back to root of your workspace and run:
```bash
colcon build --symlink-install
```
now you built your package and created a symlink to your code, so you can edit it without rebuilding the package every time.
To run the node, first source setup in install directory:
```bash
source install/setup.bash
```
Then run the node:
```bash
ros2 run my_first_package my_second_node
```
and you will see something like:
```bash
[INFO] [my_second_node]: My Second Node Created!
[INFO] [my_second_node]: Timer callback executed 0 times!
[INFO] [my_second_node]: Timer callback executed 1 times!
[INFO] [my_second_node]: Timer callback executed 2 times!
[INFO] [my_second_node]: Timer callback executed 3 times!
...
```

### References
- [ROS2 Documentation](https://docs.ros.org/en/jazzy/index.html)
- [ROS2 Tutorials](https://docs.ros.org/en/jazzy/Tutorials.html)