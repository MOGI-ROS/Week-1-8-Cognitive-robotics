[//]: # (Image References)

[image1]: ./assets/starter-package.png "Starter package"
[image2]: ./assets/terminator.png "Terminator"
[image3]: ./assets/windows-terminal.png "Terminal"
[image4]: ./assets/rqt_graph_1.png "rqt_graph"
[image5]: ./assets/rqt_1.png "rqt"
[image6]: ./assets/turtlesim_1.png "turtlesim"
[image7]: ./assets/rqt_graph_2.png "rqt_graph"
[image8]: ./assets/rqt_2.png "rqt"
[image9]: ./assets/rqt_graph_3.png "rqt_graph"
[image10]: ./assets/rqt_graph_4.png "rqt_graph"
[image11]: ./assets/rqt_graph_5.png "rqt_graph"
[image12]: ./assets/gazebo.png "Shapes.sdf"
[image13]: ./assets/gazebo-1.png "Gazebo GUI"
[image14]: ./assets/gazebo-2.png "Gazebo models"
[image15]: ./assets/turtlebot.png "Turtlebot"
[image16]: ./assets/turtlebot-1.png "Turtlebot"
[image17]: ./assets/turtlebot-2.png "Turtlebot"
[image18]: ./assets/slam.png "SLAM"
[image19]: ./assets/navigation.png "Navigation"

# Week 1-8: Cognitive robotics

## This is how far we will get by the end of this lesson: 
  <a href="https://youtu.be/Exqm_VrOytY"><img width="600" src="./assets/youtube-line-following.png"></a>  

## Here you can see a short video about the final projects from the previous years:

<a href="https://youtu.be/opmUsI7y6Y8"><img width="600" src="./assets/projects.png"></a>

# Table of Contents
1. [ROS basics](#ros-basics)  
2. [Gazebo basics](#gazebo-basics)  
3. [Turtlebot3 simulation](#turtlebot3-simulation)  
4. [Test on the real Turtlebot3](#test-on-the-real-turtlebot3)  
5. [Turtlebot3 MOGI](#turtlebot3-mogi)  
6. [Line following](#line-following)  
7. [Neural network](#neural-network)  
8. [Test on the real robot](#test-on-the-real-robot)  


# ROS basics

## What is ROS(2)?
ROS, or Robot Operating System, is an open-source framework designed to facilitate the development of robotic applications. It provides a collection of tools, libraries, and conventions that simplify the process of designing complex robot behaviors across a wide variety of robotic platforms.

ROS was initially developed in 2007 by the Stanford Artificial Intelligence Laboratory and continued by Willow Garage, with the goal of providing a common platform for research and development in robotics. The primary motivation was to create a standard framework that could support a broad range of robotic applications, promote code reuse, and foster collaboration within the robotics community.

Key reasons for ROS development include:

- **Standardization**: Creating a common platform that simplifies the integration of different hardware and software components.
- **Modularity**: Enabling the development of modular and reusable software components (nodes) that can be easily shared and adapted for various robotic systems.
- **Community Collaboration**: Encouraging collaboration among researchers and developers, resulting in a vast collection of tools and libraries.

### Transition to ROS 2

ROS 2 was developed to address the limitations of ROS 1 and meet the growing demands for industrial and commercial robotics applications. The development began around 2014 and aimed to enhance the capabilities of ROS, particularly in areas such as security, real-time performance, and support for multi-robot systems. In practice, the biggest difference is in the underlying middleware, ROS1 uses a custom transport layer and message-passing system that was not designed for real-time or distributed applications (see ROS1's [`roscore`](http://wiki.ros.org/roscore)).

The latest ROS1 release is ROS Noetic which was intended to be used on Ubuntu 20.04. It goes to [EOL in May, 2025](http://wiki.ros.org/Distributions) together with Ubuntu 20.04.

## Required softwares

<details>
<summary>Ubuntu 24.04 LTS</summary>

<br>In the course we'll use ROS2 [Jazzy Jalisco](https://docs.ros.org/en/jazzy/index.html), which requires Ubuntu 24.04 for the smoothest operation.

You have a couple of options, but the most recommended is the native installation of the operating system - external SSD, dual boot, etc.

1) Native install, the most recommended way. You will learn how to set up the environment, there won't be difficulties with GPU acceleration and many more advantages.
2) Windows 11 WSL2 (Windows Subsystem Linux), see [instructions](https://documentation.ubuntu.com/wsl/en/latest/guides/install-ubuntu-wsl2/). It's a straightforward way if you want to use within Windows environment. You can still learn how to set up the environment but can be more challenging with GUI applications and 3D acceleration.
3) Virtual machine, VMware fusion is [now free for personal use](https://blogs.vmware.com/teamfusion/2024/05/fusion-pro-now-available-free-for-personal-use.html). Less flexible than WSL2 on Windows 11 but works well on macOS. 3D acceleration can be challenging.
4) Docker container. First it might look an easy way to use any ROS distribution on any host operating system, but it's getting more and more challenging if we need GUI applications, 3D acceleration and can be confusing for beginners how to work within the container. You might miss some important experience with setting up the environment if using a pre-configured container image. I only recommend this way for experienced Docker users.
5) Using an online environment e.g. [The Construct](https://www.theconstruct.ai). It looks promising that you don't have to install any special software, but you won't gain experience with setting up the environment. It can be difficult to cherry pick the software versions you need and accessing GUI applications through the web interface is a poor experience.

The options 1 and 2 are the most practical and preferred ways to use ROS. In an exotic case, if you want to run Ubuntu 24.04 and ROS2 Jazzy on macOS and Apple silicon [this](https://www.youtube.com/watch?v=kDosGTdwqO0) is a very good tutorial.

>Pro tip if you want to mount directories from your host system into your guest Ubuntu 24.04 running in VMware fusion, more details [on this link](https://www.liquidweb.com/blog/create-vmware-shared-folder/):
>```bash
>/usr/bin/vmhgfs-fuse .host:/BME/ROS2-lessons /home/david/ros2_ws/src/ROS2-lessons -o subtype=vmhgfs-fuse,allow_other
>```
</details>

<details>
<summary>Visual Studio Code</summary>

<br>The recommended code editor during the course is [Visual Studio Code](https://code.visualstudio.com/docs/setup/setup-overview), but it's up to your choice if you want to go with your different editor. Depending on your Ubuntu install method you might install it natively on Ubuntu, in your virtual environment or on your host operating system.

Recommended extensions to install:
- Markdown All in One
- C/C++
- Python
- CMake Tools
- Remote - SSH - if you work on physical robots, too
- Remote - WSL - if you do the course using WSL2
</details>

<details>
<summary>GitHub and a git client</summary>

<br>The course materials are available on GitHub, and the submissions of your final projects shall also use GitHub. *You'll need a very good excuse why to use alternative git solutions like GitLab.*

So I encourage everyone to [register your GitHub accounts](https://github.com/home), and if you are there don't forget to [sign up for the GitHub Student Developer Pack](https://education.github.com/pack) which gives you a bunch of powerful developer tools for free.

I recommend to use a graphical git client that can boost your experience with git, in my optinion the best one is [GitKraken](https://www.gitkraken.com), which is not a free software, but you get the pro version as part of the GitHub Student Developr Pack! If you prefer using git as a *cli* tool then no worries, it's absoluetely all right.

</details>

<details>
<summary>Markdown</summary>

<br>Markdown is not a standalone software but rather a lightweight, plain-text formatting language used to create formatted documents. It was created by John Gruber in 2004 with the goal of being easy to read and write, using simple syntax to style text, create lists, links, images, and more. It is widely used for writing documentation, readme files, and content for static websites.

Basic Markdown Syntax

- Headings: `#` Heading 1, `##` Heading 2, etc.
- Bold: `**bold text**` or `__bold text__`
- Italic: `*italic text*` or `_italic text_`
- Lists:
    - Unordered: `- Item` or `* Item`
    - Ordered: `1. Item`
    - Links: `[Link text](URL)`
    - Images: `![Alt text](Image URL)`
    - Code: Inline code or code blocks using triple backticks (```)

GitHub Flavored Markdown (GFM)

GitHub Flavored Markdown (GFM) is a variant of Markdown used by GitHub to provide additional features and syntax that are not available in standard Markdown. It includes:

- Tables:
    ```
    | Column 1 | Column 2 |
    |----------|----------|
    | Row 1    | Data     |
    | Row 2    | Data     |
    ```
- Task lists:
    ```
    - [x] Task 1
    - [ ] Task 2
    ```
- Strikethrough: `~~strikethrough text~~`
- Syntax highlighting in a specific language:
    ```
    ```python
    def hello_world():
    print("Hello, world!")
    ```
- Tables of Contents
- @mentions for users, references to issues, and pull requests using #number

Most of the tips and tricks that you might need for your own project documentation can be found [in the source of this readme](https://github.com/MOGI-ROS/Week-1-2-Introduction-to-ROS2/blob/main/README.md?plain=1) that you read right now, feel free to use any snippets from it!

</details>

<details>
<summary>A good terminal</summary>

<br>It's up to your choice which terminal tool would you like to use, but I strongly recommend one that support multiple split windows in a single unified window, because we will use a **lot of** terminals! On Linux, I can recommend  `terminator`:
![alt text][image2]

In case you use WSL2, the built-in Windows terminal also support multiple panes and works really well!
![alt text][image3]

</details>

<details>
<summary>And finally, install ROS2 Jazzy</summary>

<br>ROS always had very good and detailed installed guides, it's not anything different for ROS2's Jazzy release.
The installation steps can be found [here](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html), with Ubuntu 24.04 it can be installed simply through pre-built, binary `deb` packages.

After installing it we have to set up our ROS2 environment with the following command:
```bash
source /opt/ros/jazzy/setup.bash
```

By default, we have to run this command in every new shell session we start, but there is a powerful tool in Linux for such use cases. `.bashrc` file is always in the user's home directory and it is used for user-specific settings for our shell sessions. You can edit `.bashrc` directly in a terminal window with a basic text editor, like `nano`:

```bash
david@david-ubuntu24:~$ nano .bashrc
```

Here, you can add your custom user-specific settings in the end of the file, that will be executed every time you initiate a new shell session.
I created an [example gist](https://gist.github.com/dudasdavid/bb2366e2a68bf1401ed692e41fed04d8) that you can add to the end of your file and use it during the course.

ROS2 Jazzy has [an even more detailed tutorial](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html) about setting up your environment, you can check it out, too!

</details>

## Running some examples

Your ROS2 install comes with a couple of good examples as you can also find it [on the install page](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html#try-some-examples).

Let's try them!

The following command starts a simple `publisher` written in C++. A `publisher` is a node that is responsible for sending messages with a certain type over a specific `topic` (in this example the topic's name is `chatter` and the type is a string). A `topic` is a communication pipeline in the publish-subscribe communication model where a single message is sent to multiple subscribers, unlike message-queues that are point-to-point models, where a single message is sent to a single consumer. Publishers broadcast messages to topics, and subscribers listen to those topics to receive a copy of the message.

> Publish-subscribe models are asynchronous, one-to-many or many-to-many interactions where the publishers don't know how many subscribers there are (if any). Therefore publisher never expects any response or confirmation from the subscribers.

Now, let's run the demo publisher written in C++:

```bash
ros2 run demo_nodes_cpp talker
```

Your output should look like this:
```bash
david@david-ubuntu24:~$ ros2 run demo_nodes_cpp talker
[INFO] [1727116062.558281395] [talker]: Publishing: 'Hello World: 1'
[INFO] [1727116063.558177802] [talker]: Publishing: 'Hello World: 2'
[INFO] [1727116064.558010534] [talker]: Publishing: 'Hello World: 3'
[INFO] [1727116065.557939861] [talker]: Publishing: 'Hello World: 4'
[INFO] [1727116066.557849645] [talker]: Publishing: 'Hello World: 5'
```

Let's start a subscriber - written in Python - in another terminal window, which subscribes to the `chatter` topic and listens to the publisher node's messages:

```bash
david@david-ubuntu24:~$ ros2 run demo_nodes_py listener
[INFO] [1727116231.574662048] [listener]: I heard: [Hello World: 170]
[INFO] [1727116232.560517676] [listener]: I heard: [Hello World: 171]
[INFO] [1727116233.558907367] [listener]: I heard: [Hello World: 172]
[INFO] [1727116234.560768278] [listener]: I heard: [Hello World: 173]
[INFO] [1727116235.559821377] [listener]: I heard: [Hello World: 174]
[INFO] [1727116236.559993767] [listener]: I heard: [Hello World: 175]
```

### Useful cli and graphical tools
Now both nodes are running we can try a few useful tools. The first on let us know what kind of nodes are running in your ROS2 system:

```bash
ros2 node list
```

Which gives us the following output:
```bash
david@david-ubuntu24:~/ros2_ws$ ros2 node list
/listener
/talker
```

If we want to know more about one of our nodes, we can use the `ros2 node info /node` command:
```bash
david@david-ubuntu24:~/ros2_ws$ ros2 node info /listener 
/listener
  Subscribers:
    /chatter: std_msgs/msg/String
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /listener/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /listener/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /listener/get_parameters: rcl_interfaces/srv/GetParameters
    /listener/get_type_description: type_description_interfaces/srv/GetTypeDescription
    /listener/list_parameters: rcl_interfaces/srv/ListParameters
    /listener/set_parameters: rcl_interfaces/srv/SetParameters
    /listener/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:

  Action Clients:
```

At the moment, the most interesting detail we can gather about a node is if it's subscribing or publishing to any topic. In later lessons we'll learn more about parameters and services.

---

In a very similar way, we can also list all of our topics with `ros2 topic list` command:
```bash
david@david-ubuntu24:~/ros2_ws$ ros2 topic list 
/chatter
/parameter_events
/rosout
```

And we can get more details about a certain topic with the `ros2 topic info /topic` command:
```bash
david@david-ubuntu24:~/ros2_ws$ ros2 topic info /chatter 
Type: std_msgs/msg/String
Publisher count: 1
Subscription count: 1
```

Another powerful tool is `rqt_graph` that helps us visualizing the nodes and topics in a graph.
![alt text][image4]

`rqt_graph` can be used as a standalone tool, or part of `rqt` which can be used to build a complete dashboard to mintor and control your nodes. We'll spend a lot of time with it, at the moment let's just see the message monitoring function:
![alt text][image5]

---

### Let's run more examples: turtlesim
Let's see another built in example which is a simple 2D plotter game.
> In a case it's not automatically installed, you can install it with the following command:
> ```bash
> sudo apt install ros-jazzy-turtlesim
> ```

To run the main node just execute the follwoing command:
```bash
ros2 run turtlesim turtlesim_node
```

And in another terminal start its remote controller, you can simply drive the turtle with the arrows:
```bash
ros2 run turtlesim turtle_teleop_key
```
![alt text][image6]

We can use the same tools as before to see the running nodes and topics, here is how does it look like in `rqt_graph`.
![alt text][image7]

>We should notice two important things:
>
>1. turtlesim is more complex than the previous example with multiple services and parameters that we'll check in the end of this lesson.
>2. the turtle is controlled with a `cmd_vel` message which is a 6D vector in space. We'll use this exact same message type in the future to drive our simulated robots.

Now let's move on to create our own nodes!

## Create a colcon workspace

To create, build and run custom nodes we need packages, but first we need a workspace where we'll maintain our future packages.
There are 2 new terms we must learn about ROS2 workspaces:

1. `ament` provides the underlying build system and tools specifically for ROS2 packages. `ament_cmake` is a CMake-based build system for C/C++ nodes and `ament_python` provides the tools for packing and installing python nodes and libraries.
2. `colcon` (COmmand Line COLlectioN) is a general-purpose tool to build and manage entire workspaces with various build systems, including ament, cmake, make, and more. 

It means that our ROS2 workspace will be a `colcon workspace` which - in the backround - will use `ament` for building the individual packages.

>If you have experience with ROS1, `colcon` and `ament` replaces the old `catkin` tools.

Let's create our workspace inside our user's home directory:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

>A workspace must have a `src` folder where we maintain the source files of our packages, during building of the workspace colcon will create folders for the deployment of binaries and other output files.

Let's go into the `src` folder and create our first python package:
```bash
ros2 pkg create --build-type ament_python bme_ros2_tutorials_py
```
> During package creation we should define if it's a C/C++ (`ament_cmake`) or a python (`ament_python`) package. If we don't do it, the default is always `ament_cmake`!

We'll put our python scripts under `bme_ros2_tutorials_py` which is an automatically created folder with the same name as our package, it already has an empty file `__init__.py`, let's add our first node here: `hello_world.py`.

>We can create files in Linux in several different ways, just a few examples:
> - Right click in the folder using the desktop environment
> - Through the development environment, in our case Visual Studio Code
> - From command line in the current folder using the `touch` command: `touch hello_world.py`

At this point our workspace should look like this (other files and folders are not important at this point):
```bash
david@david-ubuntu24:~/ros2_ws$ tree -L 4
.
└── src
    └── bme_ros2_tutorials_py
        ├── bme_ros2_tutorials_py
        │   ├── __init__.py
        │   └── hello_world.py
        ├── package.xml
        └── setup.py
```

> It's always recommended to fill the `description`, `maintainer` with your name and email address and `license` fields in your `package.xml` and `setup.py` files. I personally prefer a highly permissive license in non-commercial packages of mine, like `BSD` or `Apache License 2.0`.

## Let's write the simplest possible `hello_world` in python:

```python
#!/usr/bin/env python3

# Main entry point, args is a parameter that is used to pass arguments to the main function
def main(args=None):
    print("Hello, world!")

# Check if the script is being run directly
if __name__ == '__main__':
    main()
```

Although this is a python script that doesn't require any compilation, we have to make sure that `ament` will pack, copy and install our node. It's important to understand that we are not running python scripts directly from the source folder!

Let's edit `setup.py` that was automatically generated when we defined that our package will use `ament_python`.

Add an entry point for our python node. An entry point describes the folder, the filename (without `.py`) and the main entry point within the script:

```python
...
        entry_points={
            'console_scripts': [
                'py_hello_world = bme_ros2_tutorials_py.hello_world:main'
            ],
        },
...
```

Our first node within our first package is ready for building it! Build must be initiated always in the root of our workspace!

```bash
cd ~/ros2_ws
```

And here we execute the `colcon build` command.

After a successful build we have to update our environnment to make sure ROS2 cli tools are aware about of any new packages. To do this we have to run the following command:
```bash
source install/setup.bash
```

As we did with the base ROS2 environment, we can add this to the `.bashrc` so it'll be automatically sourced every time when we open a terminal:
```bash
source ~/ros2_ws/install/setup.bash
```

And now we are ready to run our first node:
```bash
ros2 run bme_ros2_tutorials_py py_hello_world
```

Athough we could run our first node, it was just a plain python script, not using any ROS API. Let's upgrade hello world to a more *ROS-like* hello world. We import the `rclpy` which is the ROS2 python API and we start using the most basic functions of `rclpy` like `init()`, `create_node()` and `shutdown()`. If you already want to do a deep-dive in the API functions you can find everything [here](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Client-Libraries.html#the-rclpy-package).

```python
#!/usr/bin/env python3
import rclpy # Import ROS2 python interface

# Main entry point, args is a parameter that is used to pass arguments to the main function
def main(args=None):
    rclpy.init(args=args)                          # Initialize the ROS2 python interface
    node = rclpy.create_node('python_hello_world') # Node constructor, give it a name
    node.get_logger().info("Hello, ROS2!")         # Use the ROS2 node's built in logger
    node.destroy_node()                            # Node destructor
    rclpy.shutdown()                               # Shut the ROS2 python interface down

# Check if the script is being run directly
if __name__ == '__main__':
    main()
```

We don't have to do anything with `setup.py`, the entrypoint is already there, but we have to re-build the colcon workspace!

After the build we can run our node:
```bash
ros2 run bme_ros2_tutorials_py py_hello_world
```

## Create a python publisher

Let's make our first publisher in python, we create a new file in the `bme_ros2_tutorials_py` folder: `publisher.py`.

We start expanding step-by-step our knowledge about the ROS2 API with publishing related functions (`create_publisher()` and `publish()`).

```python
#!/usr/bin/env python3
import rclpy
from std_msgs.msg import String # Import 'String' from ROS2 standard messages
import time

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('python_publisher')
    # Register the node as publisher
    # It will publish 'String' type to the topic named 'topic' (with a queue size of 10)
    publisher = node.create_publisher(String, 'topic', 10)

    msg = String()                     # Initialize msg as a 'String' instance
    i = 0
    while rclpy.ok():                  # Breaks the loop on ctrl+c
        msg.data = f'Hello, world: {i}' # Write the actual string into msg's data field
        i += 1
        node.get_logger().info(f'Publishing: "{msg.data}"')
        publisher.publish(msg)         # Let the node publish the msg according to the publisher setup
        time.sleep(0.5)                # Python wait function in seconds

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

We have to edit `setup.py`, registering our new node as entry point:
```python
...
    entry_points={
        'console_scripts': [
            'py_hello_world = bme_ros2_tutorials_py.hello_world:main',
            'py_publisher = bme_ros2_tutorials_py.publisher:main'
        ],
    },
...
```

Don't forget to rebuild the workspace and we can run our new node:

```bash
david@david-ubuntu24:~$ ros2 run bme_ros2_tutorials_py py_publisher
[INFO] [1727526317.470055907] [python_publisher]: Publishing: "Hello, world: 0"
[INFO] [1727526317.971461827] [python_publisher]: Publishing: "Hello, world: 1"
[INFO] [1727526318.473896872] [python_publisher]: Publishing: "Hello, world: 2"
[INFO] [1727526318.977439178] [python_publisher]: Publishing: "Hello, world: 3"
```

We can observe the published topic through `rqt`'s topic monitor:
![alt text][image8]

Or we can use a simple but powerful tool, the `topic echo`:
```bash
david@david-ubuntu24:~$ ros2 topic echo /topic 
data: 'Hello, world: 23'
---
data: 'Hello, world: 24'
---
data: 'Hello, world: 25'
```

The publisher node above is very simple and looks exactly how we historically impelented nodes in ROS1. But ROS2 provides more powerful API functions and also places a greater emphasis on object-oriented programming. So let's create another publisher in a more OOP way and using the timer functions (`create_timer()`) of the ROS2 API. The other important API function is `rclpy.spin(node)` which keeps the node running until we don't quit it with `ctrl+c` in the terminal.

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node     # Import ROS2 Node as parent for our own node class
from std_msgs.msg import String

class MyPublisherNode(Node):
    def __init__(self):
        super().__init__("python_publisher_oop")
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)     # Timer callback, period in seconds, not frequency!
        self.i = 0
        self.msg = String()
        self.get_logger().info("Publisher OOP has been started.")

    def timer_callback(self):                                        # Timer callback function implementation
        self.msg.data = f"Hello, world: {self.i}"
        self.i += 1
        self.get_logger().info(f'Publishing: "{self.msg.data}"')
        self.publisher_.publish(self.msg)

def main(args=None):
    rclpy.init(args=args)
    node = MyPublisherNode() # node is now a custom class based on ROS2 Node
    rclpy.spin(node)         # Keeps the node running until it's closed with ctrl+c
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

As we did previously, add the new script's entrypoint in the `setup.py`, build the workspace and run our new node:
```bash
ros2 run bme_ros2_tutorials_py py_publisher_oop
```

## Create a python subscriber

Let's create a new file `subscriber.py` in our python package (`bme_ros2_tutorials_py`). First we make a very simple implementation and after that we'll implement a more OOP version of it again. We further extend our knowledge with more API functions related to subscriptions (`create_subscription()`).

```python
#!/usr/bin/env python3
import rclpy
from std_msgs.msg import String


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('python_subscriber')

    def subscriber_callback(msg):                      # Subscriber callback will be invoked every time when a message arrives to the topic it has subsctibed
        node.get_logger().info(f"I heard: {msg.data}")

    # Register the node as a subscriber on a certain topic: 'topic' (with a certain data type: String)
    # and assign the callback function that will be invoked when a message arrives to the topic
    # with a queue size of 10 which determines how many incoming messages can be held in the subscriber’s
    # queue while waiting to be processed by the callback function
    subscriber = node.create_subscription(String, 'topic', subscriber_callback, 10) 
    node.get_logger().info("Subsciber has been started.")

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Add the node to the `setup.py` file as a new entry point

```python
...
    entry_points={
        'console_scripts': [
            'py_hello_world = bme_ros2_tutorials_py.hello_world:main',
            'py_publisher = bme_ros2_tutorials_py.publisher:main',
            'py_publisher_oop = bme_ros2_tutorials_py.publisher_oop:main',
            'py_subscriber = bme_ros2_tutorials_py.subscriber:main'
        ],
    },
...
```

Then, build the workspace and we can run our new node!

```bash
david@david-ubuntu24:~$ ros2 run bme_ros2_tutorials_py py_subscriber
[INFO] [1727606328.416973729] [python_subscriber]: Subsciber has been started.
```

If we don't start a publisher, then our subscriber is just keep listening to the `/topic` but the callback function is not invoked. The node doesn't stop running because of the `rclpy.spin(node)`  function.

Let's start our C++ publisher in another terminal:
```bash
david@david-ubuntu24:~$ ros2 run bme_ros2_tutorials_cpp publisher_cpp 
[INFO] [1727606744.184678739] [cpp_publisher]: CPP publisher has been started.
[INFO] [1727606744.685934650] [cpp_publisher]: Publishing: 'Hello, world: 0'
[INFO] [1727606745.185073828] [cpp_publisher]: Publishing: 'Hello, world: 1'
[INFO] [1727606745.686288921] [cpp_publisher]: Publishing: 'Hello, world: 2'
[INFO] [1727606746.186169881] [cpp_publisher]: Publishing: 'Hello, world: 3'
```

And let's see what happens with the subscriber! It's subscription callback function is invoked every time when the publisher sends a message onto the `/topic`.
```bash
david@david-ubuntu24:~/ros2_ws$ ros2 run bme_ros2_tutorials_py py_subscriber
[INFO] [1727606614.099180007] [python_subscriber]: Subsciber has been started.
[INFO] [1727606744.695260304] [python_subscriber]: I heard: Hello, world: 0
[INFO] [1727606745.187956805] [python_subscriber]: I heard: Hello, world: 1
[INFO] [1727606745.689289484] [python_subscriber]: I heard: Hello, world: 2
[INFO] [1727606746.188467429] [python_subscriber]: I heard: Hello, world: 3
```

We can also check it with `rqt_graph`:
![alt text][image9]

> And we can also observe the language agnostic approach of ROS2, without any additional effort this middleware provides interfacing between nodes written in different programming languages.

As before, let's make our subscriber more OOP using our previous template from the publisher. Compared to the publisher we just need to replace the timer callback with a subscription callback and that's all!
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MySubscriberNode(Node):
    def __init__(self):
        super().__init__("python_subsciber_oop")
        self.subscriber_ = self.create_subscription(String, 'topic', self.subscriber_callback, 10)
        self.get_logger().info("Subsciber OOP has been started.")

    def subscriber_callback(self, msg):
        self.get_logger().info(f"I heard: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = MySubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```
Build the workspace and run the node.

## Launchfiles

As you noticed with the previous examples we have to use as many terminals as many nodes we start. With a simple publisher and subscriber this isn't really a big deal, but in more complex robotic projects, it's quite common to use ROS nodes in the range of tens or even hundreds. Therefore ROS provides an efficient interface to start multiple nodes together and even re-map their topics to different ones or change its parameters instead of changing the source code itself. 

Compared to ROS1 it's a bit more complicated to bundle these launchfiles with our nodes, so as a best practice, I recommend creating an individual pakage only for our launcfiles.

Let's create a new package with `ament_cmake` or simply without specifying the build type (by default it's `ament_cmake`).

```bash
ros2 pkg create bme_ros2_tutorials_bringup
```

Now let's create a `launch` folder within this new package.
We can freely delete include and src folders:

> If you want to delete a folder from command line that is not empty you can use the `rm -rf folder` command

```bash
rm -rf include/ src/
```

Add the following to the `CMakeLists.txt` to install the content of `launch` when we build the workspce:
```bash
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
```

Create a new launch file, and let's call it `publisher_subscriber.launch.py`. In ROS2 the launchfiles are special declarative python scripts (with some imperative flavours) instead of the `xml` files we used in ROS1! Actually ROS2 also has the possibility to use `xml` based launch files, but the general usage and the documentation of this feature is very poor. Initially the python based launch system was intended to be the backend of xml launchfiles but it wasn't ready for the initial launch of ROS2 and the community rather jumped on using the python launch system.
```bash
touch publisher_subscriber.launch.py
```

Let's create our template that we can re-use in the future with only one publisher first. When we add a node to the launch file we must define the the `package`, the `node` (executable) and a freely chosen `name`.
```python
#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    publisher_node = Node(
        package="bme_ros2_tutorials_py",
        executable="py_publisher",
        name="my_publisher"
    )

    ld.add_action(publisher_node)
    return ld
```

Build and don't forget to source the workspace because we added a new package!

After it we can execute our launchfile with the `ros2 launch` command:
```bash
david@david-ubuntu24:~$ ros2 launch bme_ros2_tutorials_bringup publisher_subscriber.launch.py
[INFO] [launch]: All log files can be found below /home/david/.ros/log/2024-09-29-14-17-29-864407-david-ubuntu24-41228
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [py_publisher-1]: process started with pid [41231]
[py_publisher-1] [INFO] [1727612250.056173684] [my_publisher]: Publishing: "Hello, world: 0"
[py_publisher-1] [INFO] [1727612250.559170990] [my_publisher]: Publishing: "Hello, world: 1"
[py_publisher-1] [INFO] [1727612251.061618736] [my_publisher]: Publishing: "Hello, world: 2"
```

> If we don't write the *`launch`* word explicitly in the filename of our launch file, the `ros2 launch` cli tool won't be able to autocomplete the filenames.

We can notice that our node is now called `my_publisher` instead of `python_publisher` as we coded in the node itself earlier. With the launch files we can easily rename our nodes for better handling and organizing as our application scales up.

We can use the `node list` tool to list our nodes and the output will look like this:
```bash
david@david-ubuntu24:~/ros2_ws$ ros2 node list 
/my_publisher
```

### Now let's add the subscriber too:

Every time when we add a node to the launch file we also have to register it with the `ld.add_action()` function:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    publisher_node = Node(
        package="bme_ros2_tutorials_py",
        executable="py_publisher",
        name="my_publisher",
    )

    subscriber_node = Node(
        package="bme_ros2_tutorials_py",
        executable="py_subscriber",
        name="my_subscriber",
    )

    ld.add_action(publisher_node)
    ld.add_action(subscriber_node)
    return ld
```

Don't forget to rebuild the workspace so the changed launchfile will be installed, after that we can run it!
```bash
david@david-ubuntu24:~$ ros2 launch bme_ros2_tutorials_bringup publisher_subscriber.launch.py
[INFO] [launch]: All log files can be found below /home/david/.ros/log/2024-09-29-14-20-15-603371-david-ubuntu24-41380
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [py_publisher-1]: process started with pid [41383]
[INFO] [py_subscriber-2]: process started with pid [41384]
[py_publisher-1] [INFO] [1727612415.811451529] [my_publisher]: Publishing: "Hello, world: 0"
[py_subscriber-2] [INFO] [1727612415.811459737] [my_subscriber]: Subsciber has been started.
[py_subscriber-2] [INFO] [1727612415.811878677] [my_subscriber]: I heard: Hello, world: 0
[py_publisher-1] [INFO] [1727612416.313222973] [my_publisher]: Publishing: "Hello, world: 1"
[py_subscriber-2] [INFO] [1727612416.315340170] [my_subscriber]: I heard: Hello, world: 1
```

We can see that both nodes started and their logging to the standard output is combined in this single terminal window.

We can verify this with `node list` or using `rqt_graph` visually:
```bash
david@david-ubuntu24:~$ ros2 node list 
/my_publisher
/my_subscriber
```

We can also verify the used topics with the `topic list` tool:
```bash
david@david-ubuntu24:~$ ros2 topic list 
/parameter_events
/rosout
/topic
```

# Gazebo basics
Gazebo is a powerful robotics simulation tool that provides a 3D environment for simulating robots, sensors, and objects. It is widely used in the ROS ecosystem for testing and developing robotics algorithms in a realistic virtual environment before deploying them to real hardware.

Gazebo integrates tightly with ROS, enabling simulation and control of robots using ROS topics, services, and actions. In ROS2 with the latest Gazebo releases the integration is facilitated by `ros_gz`.

Key Features of Gazebo:

- 3D Physics Engine:
    Simulates rigid body dynamics, collision detection, and other physics phenomena using engines like ODE, Bullet, and DART.
- Realistic Sensors:
    Simulates cameras, LiDAR, IMUs, GPS, and other sensors with configurable parameters.
- Plugins:
    Extensible via plugins to control robots, customize physics, or add functionality.
- Worlds and Models:
    Enables users to create complex environments with pre-built or custom objects and robots.

Besides Gazebo, there are many alternative simulation environments for ROS, but usually the setup of these simulators are more complicated and less documented. Certain simulators also have very high requirements for the GPU.

| **Simulator** | **Best For**                          | **Advantages**                      | **Disadvantages**                           |
|---------------|---------------------------------------|-------------------------------------|---------------------------------------------|
| **Gazebo**    | General robotics simulation in ROS    | Free, accurate physics, ROS support | Moderate visuals, resource-heavy            |
| **Unity**     | High-fidelity visuals and AI/ML tasks | Realistic graphics, AI tools        | Steep learning curve, not robotics-specific |
| **Webots**    | Beginner-friendly robotics simulation | Easy setup, cross-platform          | Limited graphics, less customizable         |
| **Isaac Sim** | High-end AI and robotics simulation   | High-fidelity physics, AI support   | GPU-intensive, complex setup                |


## Install Gazebo

Before we install Gazebo we have to understand the compatibility between Gazebo versions and ROS distributions.

| **ROS Distribution** | **Gazebo Citadel (LTS)** | **Gazebo Fortress (LTS)** | **Gazebo Garden** | **Gazebo Harmonic (LTS)** | **Gazebo Ionic** |
|-----------------------|--------------------------|---------------------------|-------------------|---------------------------|------------------|
| **ROS 2 Rolling**     | ❌                        | ❌                         | ⚡                 | ⚡                         | ✅                |
| **ROS 2 Jazzy (LTS)** | ❌                        | ❌                         | ⚡                 | ✅                         | ❌                |
| **ROS 2 Iron**        | ❌                        | ✅                         | ⚡                 | ⚡                         | ❌                |
| **ROS 2 Humble (LTS)**| ❌                        | ✅                         | ⚡                 | ⚡                         | ❌                |
| **ROS 2 Foxy (LTS)**  | ✅                        | ❌                         | ❌                 | ❌                         | ❌                |
| **ROS 1 Noetic (LTS)**| ✅                        | ⚡                         | ❌                 | ❌                         | ❌                |

Since we use the latest LTS ROS2 distribution, Jazzy, we need Gazebo Harmonic.

To install Gazebo Harmonic binaries on Ubuntu 24.04 simply follow the steps [on this link](https://gazebosim.org/docs/harmonic/install_ubuntu/).

Once it's installed we can try it with the following command:
```bash
gz sim shapes.sdf
```

If everything works well you should see the following screen:
![alt text][image12]

If you have a problem with opening this example `shapes.sdf` there might be various reasons that requires some debugging skills with Gazebo and Linux.

- If you see a `Segmentation fault (Address not mapped to object [(nil)])` due to problems with `Qt` you can try to set the following environmental variable to force Qt to use X11 instead of Wayland. [Link](https://robotics.stackexchange.com/questions/111547/gazebo-crashes-immediately-segmentation-fault-address-not-mapped-to-object-0)
    ```bash
    export QT_QPA_PLATFORM=xcb
    ```

- If you run Gazebo in WSL2 or virtual machine the most common problem is with the 3D acceleration with the OGRE2 rendering engine of Gazebo. You can either try disabling HW acceleration (not recommended) or you can switch the older OGRE rendering engine with the following arguments. [Link](https://github.com/gazebosim/gz-sim/issues/1492)
    ```bash
    gz sim shapes.sdf --render-engine ogre
    ```

- If you run Ubuntu natively on a machine with an integrated Intel GPU and a discrete GPU you can check [this troubleshooting guide](https://gazebosim.org/docs/harmonic/troubleshooting/#problems-with-dual-intel-and-nvidia-gpu-systems).

After Gazebo successfully starts we can install the Gazebo ROS integration with the following command:
```bash
sudo apt install ros-jazzy-ros-gz
```

You can find the official install guide [here](https://gazebosim.org/docs/harmonic/ros_installation/).

## Run Gazebo examples

Let's start again the `gz sim shapes.sdf` example again and let's see what is important on the Gazebo GUI:
![alt text][image13]

1. Blue - Start and pause the simulation. By default Gazebo starts the simulation paused but if you add the `-r` when you start Gazebo it automatically starts the simulation.
2. Cyan - The display shows the real time factor. It should be always close to 100%, if it drops seriously (below 60-70%) it's recommended to change the simulation step size. We'll see this later.
3. Red - You can add basic shapes or lights here and you can move and rotate them.
4. Pink - The model hierarchy, every item in the simulation is shown here, you can check the links (children) of the model, their collision, inertia, etc.
5. Green - Detailed information of the selected model in `4.` some parameters can be changed most of them are read only.
6. Plug-in browser, we'll open useful tools like `Resource Spawner`, `Visualize Lidar`, `Image Display`, etc.

Gazebo has an online model database available [here](https://app.gazebosim.org/), you can browse and download models from here. Normally this online model library is accessible within Gazebo although there might be issues in WSL2 or in virtual machines, so I prepared an offline model library with some basic models.

You can [download this offline model library](https://drive.google.com/file/d/1tcfoLFReEW1XNHPUAeLpIz2iZXqQBvo_/view?usp=share_link) from Google Drive.

After download unzip it and place it in the home folder of your user. To let Gazebo know about the offline model library we have to set the `GZ_SIM_RESOURCE_PATH` environmental variable, the best is to add it to the `.bashrc`:
```bash
export GZ_SIM_RESOURCE_PATH=~/gazebo_models
```

After setting up the offline model library let's open the `empty.sdf` in Gazebo and add a few models through the `Resource Spawner` within the `plug-in browser`:
![alt text][image14]

# Turtlebot3 simulation

In this lesson we'll use [the simulated and the real Turtlebot3 robot](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) in `burger` configuration. Turtlebot3 is not supported anymore with the latest ROS2 and Gazebo distributions, but we maintain our own packages to ensure compatibility.

Let's download the following GitHub repositories with the right branch (using the `-b branch` flag) to our colcon workspace:

```bash
git clone -b ros2 https://github.com/MOGI-ROS/turtlebot3_msgs
git clone -b mogi-ros2 https://github.com/MOGI-ROS/turtlebot3
git clone -b new_gazebo https://github.com/MOGI-ROS/turtlebot3_simulations
```

We'll need to install a couple of other dependencies with `apt` - don't forget to run `sudo apt update` and `sudo apt upgrade` if your system is not up to date:
```bash
sudo apt install ros-jazzy-dynamixel-sdk
sudo apt install ros-jazzy-hardware-interface
sudo apt install ros-jazzy-nav2-msgs
sudo apt install ros-jazzy-nav2-costmap-2d
sudo apt install ros-jazzy-nav2-map-server
sudo apt install ros-jazzy-nav2-bt-navigator
sudo apt install ros-jazzy-nav2-bringup
sudo apt install ros-jazzy-interactive-marker-twist-server
sudo apt install ros-jazzy-cartographer-ros
sudo apt install ros-jazzy-slam-toolbox
```

> If for some reasons you want to install the Dynamixel SDK from source you can download the following branch from GitHub:
> ```bash
> git clone -b humble-devel https://github.com/MOGI-ROS/DynamixelSDK/
> ```
> and if your Dynamixel SDK runs into a problem with module `em`, uninstall existing `em` and install this version as it's reported in [this GitHub issue](https://github.com/ros2/rosidl/issues/779). You might also need to install the module `lark`:
> ```bash
> pip install empy==3.3.4
> pip install lark
> ```

Before we can test the Turtlebot3 packages we have to set up `TURTLEBOT3_MODEL` environmental variable:
```bash
export TURTLEBOT3_MODEL=burger
```

It's only valid for that terminal session where you set it up, so it's recommended to add it into your `.bashrc` file so every time when you open a new terminal, it will be executed. You can use the [following gist](https://gist.github.com/dudasdavid/bb2366e2a68bf1401ed692e41fed04d8) as an example how to set up the .bashrc file.

After building the workspace and sourcing the `setup.bash` file we can test the simulation of the Turtlebot3 burger:
```bash
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

![alt text][image15]

If we start a keyboard teleop node we can already drive the robot in the simulation:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
```

Or there is another example world:
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
![alt text][image16]

Where we can try the `cartographer` package for mapping:
```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim:=true
```
![alt text][image18]

There is a third simulated environment:
```bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```
![alt text][image17]

Where we can try the `nav2` navigation stack:
```bash
ros2 launch turtlebot3_navigation2 navigation2_use_sim_time.launch.py map_yaml_file:=/home/david/ros2_ws/src/turtlebot3_simulations/turtlebot3_gazebo/maps/map.yaml
```
> Replace the `map_yaml` path to your path!

![alt text][image19]


# Test on the real Turtlebot3

Let's try the same functionality on the real Turtlebot3 Burger. The robots at the lab are updated to the latest SD card image but in case you need to write the official MOGI image to another SD card you find it [here](https://drive.google.com/file/d/1DOdRYKexACRd480pxIIv4g2WUbkFnlHY/view?usp=sharing).

> On Linux you can use the `dd` tool to create a backup or write your image back on a card. `if` is the input file and `of` is the output file.
> To create a backup you can run the following command, where `/dev/sda` must match the path to your SD card:
> ```
> sudo dd if=/dev/sda of=/home/david/backup.img status=progress
> ```
> And you can easily write an image file back to the card:
> ```
> sudo dd of=/dev/sda if=/home/david/backup.img status=progress
> ```

With the MOGI image the robots are already fully set up, [this is the `.bashrc`](https://gist.github.com/dudasdavid/a7412c46ff7c174e670a5b2b5ea1e340) that is running on the robots.

It's useful to take a look especially on this environmental variable:
```bash
# Set up a ROS2 domain ID
export ROS_DOMAIN_ID=30
```

This environment variable used in ROS2 that plays a key role in how nodes communicate over the DDS (Data Distribution Service) middleware. It partitions the DDS network into isolated segments. Nodes with the same domain ID can discover and communicate with each other, while nodes with different domain IDs remain isolated.

First, we have to make sure that the robots are on the same wireless network, if needed this must be set up using a screen and a keyboard. The wifi networks can be configured by editing the `/etc/netplan/50-cloud-init.yaml` file.

When the robot is on the same network as our PC we can connect to it using SSH, where the user name is `pi` and the IP address must match with our robot's IP address:
```bash
ssh pi@192.168.1.45
```
Then we are asked to enter the password, which is `123` for this image:
```
pi@192.168.1.45's password:
```

The ROS2 workspace is already set up on the robot, we can run the following launch file to start all the functions of the real robot:
```bash
ros2 launch turtlebot3_bringup hardware.launch.py
```

The on the PC we can start the teleop node:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
```

> I suggest to decrease the linear and angular speeds with the `z` key to some value like this:
> ```
> q/z : increase/decrease max speeds by 10%
> currently:	speed 0.12709329141645007	turn 0.25418658283290013 
> ```

Then on the PC we can try a SLAM algorithm like the `cartographer` as before or the best open-source SLAM package `slam_toolbox`:

```bash
ros2 launch turtlebot3_slam_toolbox slam_toolbox.launch.py 
```

or if you prefer `cartographer`:

```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py
```

# Turtlebot3 MOGI



ros2 launch turtlebot3_mogi simulation_bringup_slam.launch.py
ros2 launch turtlebot3_mogi simulation_bringup_navigation.launch.py
ros2 launch turtlebot3_mogi simulation_bringup_navigation_with_slam.launch.py

robot:
ros2 launch turtlebot3_bringup hardware.launch.py

pc:
ros2 launch turtlebot3_mogi robot_visualization.launch.py 
ros2 launch turtlebot3_mogi robot_mapping.launch.py
ros2 launch turtlebot3_mogi robot_navigation.launch.py


# Line following





tf virtualenv setup:

Python environment:
sudo apt install python3-pip
sudo apt install pipx
pipx ensurepath
pipx install virtualenv
pipx install virtualenvwrapper

.bashrc

replace (created by pipx ensurepath)
# Created by `pipx` on 2024-12-15 20:49:03
export PATH="$PATH:/home/david/.local/bin"

# Virtual environment for pipx and tensorflow
export PATH="$PATH:/home/$USER/.local/bin"
export WORKON_HOME=~/.virtualenvs
export VIRTUALENVWRAPPER_PYTHON=/home/$USER/.local/share/pipx/venvs/virtu>
source /home/$USER/.local/share/pipx/venvs/virtualenvwrapper/bin/virtuale>
workon tf

ERROR: Environment 'tf' does not exist. Create it with 'mkvirtualenv tf'.

mkvirtualenv tf

Install python packages:

(tf) david@david-ubuntu24:~$ 
pip install tensorflow
pip install imutils
pip install scikit-learn
pip install opencv-python
pip install matplotlib
pip install numpy==1.26.4 - to downgrade from 2.x.x



ros2 launch turtlebot3_mogi simulation_bringup_line_follow.launch.py
ros2 run turtlebot3_mogi_py line_follower


# Neural network


OpenCV version: 4.11.0
Tensorflow version: 2.18.0
Keras version: 3.7.0
CNN model: /home/david/ros2_ws/src/ROS2-lessons/Week-1-8-Cognitive-robotics/turtlebot3_mogi_py/network_model/model.best.keras
Model's Keras version: 3.7.0



```
┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━┓
┃ Layer (type)                         ┃ Output Shape                ┃         Param # ┃
┡━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━━━━┩
│ conv2d (Conv2D)                      │ (None, 24, 24, 20)          │           1,520 │
├──────────────────────────────────────┼─────────────────────────────┼─────────────────┤
│ activation (Activation)              │ (None, 24, 24, 20)          │               0 │
├──────────────────────────────────────┼─────────────────────────────┼─────────────────┤
│ max_pooling2d (MaxPooling2D)         │ (None, 12, 12, 20)          │               0 │
├──────────────────────────────────────┼─────────────────────────────┼─────────────────┤
│ conv2d_1 (Conv2D)                    │ (None, 12, 12, 50)          │          25,050 │
├──────────────────────────────────────┼─────────────────────────────┼─────────────────┤
│ activation_1 (Activation)            │ (None, 12, 12, 50)          │               0 │
├──────────────────────────────────────┼─────────────────────────────┼─────────────────┤
│ max_pooling2d_1 (MaxPooling2D)       │ (None, 6, 6, 50)            │               0 │
├──────────────────────────────────────┼─────────────────────────────┼─────────────────┤
│ flatten (Flatten)                    │ (None, 1800)                │               0 │
├──────────────────────────────────────┼─────────────────────────────┼─────────────────┤
│ dense (Dense)                        │ (None, 500)                 │         900,500 │
├──────────────────────────────────────┼─────────────────────────────┼─────────────────┤
│ activation_2 (Activation)            │ (None, 500)                 │               0 │
├──────────────────────────────────────┼─────────────────────────────┼─────────────────┤
│ dense_1 (Dense)                      │ (None, 4)                   │           2,004 │
├──────────────────────────────────────┼─────────────────────────────┼─────────────────┤
│ activation_3 (Activation)            │ (None, 4)                   │               0 │
└──────────────────────────────────────┴─────────────────────────────┴─────────────────┘
 Total params: 2,787,224 (10.63 MB)
 Trainable params: 929,074 (3.54 MB)
 Non-trainable params: 0 (0.00 B)
 Optimizer params: 1,858,150 (7.09 MB)
```

Trainable params: 929,074

GPT-4 1.76 trillion, GPT-3 175 billion, GPT-2 pedig 1.5 billion paremeters.
DeepSeek R1 671 billion = 404GB, 1.5 billion = 1.1GB


# Test on the real robot

```
┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━┓
┃ Layer (type)                         ┃ Output Shape                ┃         Param # ┃
┡━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━━━━┩
│ conv2d (Conv2D)                      │ (None, 28, 28, 2)           │              52 │
├──────────────────────────────────────┼─────────────────────────────┼─────────────────┤
│ activation (Activation)              │ (None, 28, 28, 2)           │               0 │
├──────────────────────────────────────┼─────────────────────────────┼─────────────────┤
│ max_pooling2d (MaxPooling2D)         │ (None, 7, 7, 2)             │               0 │
├──────────────────────────────────────┼─────────────────────────────┼─────────────────┤
│ conv2d_1 (Conv2D)                    │ (None, 7, 7, 4)             │             204 │
├──────────────────────────────────────┼─────────────────────────────┼─────────────────┤
│ activation_1 (Activation)            │ (None, 7, 7, 4)             │               0 │
├──────────────────────────────────────┼─────────────────────────────┼─────────────────┤
│ max_pooling2d_1 (MaxPooling2D)       │ (None, 2, 2, 4)             │               0 │
├──────────────────────────────────────┼─────────────────────────────┼─────────────────┤
│ flatten (Flatten)                    │ (None, 16)                  │               0 │
├──────────────────────────────────────┼─────────────────────────────┼─────────────────┤
│ dense (Dense)                        │ (None, 32)                  │             544 │
├──────────────────────────────────────┼─────────────────────────────┼─────────────────┤
│ activation_2 (Activation)            │ (None, 32)                  │               0 │
├──────────────────────────────────────┼─────────────────────────────┼─────────────────┤
│ dense_1 (Dense)                      │ (None, 4)                   │             132 │
├──────────────────────────────────────┼─────────────────────────────┼─────────────────┤
│ activation_3 (Activation)            │ (None, 4)                   │               0 │
└──────────────────────────────────────┴─────────────────────────────┴─────────────────┘
 Total params: 2,798 (10.93 KB)
 Trainable params: 932 (3.64 KB)
 Non-trainable params: 0 (0.00 B)
 Optimizer params: 1,866 (7.29 KB)
```

Trainable params: 932 (1000 times smaller than a LeNet-5)

ros2 launch turtlebot3_bringup hardware.launch.py  
ros2 run turtlebot3_mogi_py line_follower_cnn_robot