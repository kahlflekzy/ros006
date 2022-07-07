# Dynamic Navigation with ROS, ROS Navigation and BehaviorTree

- [Dynamic Navigation with ROS, ROS Navigation and BehaviorTree](#dynamic-navigation-with-ros-ros-navigation-and-behaviortree)
  - [Introduction](#introduction)
    - [Initial Procedures](#initial-procedures)
      - [Step 1](#step-1)
      - [Step 2](#step-2)
      - [Step 3](#step-3)
      - [Step 4](#step-4)
  - [ActionServer Nodes](#actionserver-nodes)
    - [LoadMap](#loadmap)
    - [Localization](#localization)
    - [TaskListener](#tasklistener)
    - [Task Publisher](#task-publisher)
    - [ExecuteTask](#executetask)
    - [MoveToBase](#movetobase)
      - [Setup.py](#setuppy)
  - [ActionClients (BehaviorTree) Nodes](#actionclients-behaviortree-nodes)
  - [Launch Files](#launch-files)
  - [Execution](#execution)
- [REFERENCES](#references)

## Introduction
In this work, a task is defined as a `PoseTimeArray`.  
A `PoseTimeArray` consists of `PoseTime` messages.  
A `PoseTime`message is made up of a `Pose` and a duration (in seconds).  

So, a task is gotten (published to the robot) and it moves to those poses then wait for the duration of time in the 
message, before it goes to another pose. After the task is completed, the robot returns to its initial position or base 
and awaits further instructions. 

This is the main sequence of the work, but overall, we need to implement functionalities for loading the map, initializing the 
robot pose (for `amcl`) and then going through the sequence above. 

### Initial Procedures
#### Step 1
As usual, we begin by creating a catkin workspace

```
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3/
```
#### Step 2
Install our packages below. Please refer to project [roo005](https://github.com/kahlflekzy/ros005) for details of these 
procedures.
1. BehaviorTree
2. Turtlebot3 Navigation Simulation

#### Step 3
Create our main package with dependencies.
```
catkin_create_pkg behavior_tree_navigation_v2 rospy roscpp behaviortree_cpp_v3 \
std_msgs geometry_msgs genmsg actionlib actionlib_msgs
```

#### Step 4
Create XML tree definition. Again I set the last Node to use an Inverter so the tree is restarted when execution is 
successful. See `src/tree/tree.xml` for details.

## ActionServer Nodes
### LoadMap
Create the first action node to load map from file.   
Create a `LoadMap.action` for it. See the action file with this name in the actions folder and `map_server_node.py` 
file in the scripts folder for details. Again refer to `roo005` for building process and other details. Node that we 
load a map file we had initially created and stored in a particular location. These map files can be gotten from 
[ros003](https://github.com/kahlflekzy/ros003). 

We launch the `map_server` which serves the map file via `subprocess` then listen to a topic via a subscriber to ensure 
the map server is up and running. In future work we will try to do this without launching the `map_server` node from 
`subprocess`.

### Localization
Was quite straight forward. I only needed to define the actions and the methods. This Node also sends the initial pose 
to the client which is eventually used to move the robot to base. The process of serialization required serializing one 
of the messages,then converting to `base64` then converting to a string.

Also, we load initial pose from a file that was initially created and populated with initial pose data. This file can 
be accessed from [ros004](https://github.com/kahlflekzy/ros004)

See the named file in the actions and scripts folder for more details.

### TaskListener
I had to create a custom message. `PoseTime` and `PoseTimeArray`. See the files in the `msg` folder for details.
Then the building process requires the steps below

Add the code below in `package.xml`
```
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

Add `message_generation` to `CMakeLists.txt` in the `find_package`block.

Export the `message_runtime` like shown below 
```
catkin_package(
CATKIN_DEPENDS message_runtime
)
```

Finally, uncomment the `generate_messages(...) `section, such that it looks as below. I just added more dependencies, 
action_lib had been there.
```
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
  geometry_msgs
)
```

Uncomment the `add_message_files(...)` block and include message files in the block.

Then run `catkin_make` to build project. All should run successfully.

Start `roscore` and run the code below to see your message definition.
```
rosmsg show behavior_tree_navigation_v2/PoseTimeArray [ or PoseTime] 
```

I added functionalities for listening to `actionlib` request and executing them, the task is to check if there is a 
task then reply the client with a serialized `PoseTimeArray` message. A subscriber listens for `PoseTimeArray` messages 
and stores them in a tasks lists. The serialization process again uses `base64`. See the named file in the action and 
script folders for more details.

### Task Publisher
I edited my previous task publisher to now publish PoseTimeArray see the file `task_publisher.py` for details.

### ExecuteTask
I created an action file again. This Node is based on previous work so wasn't difficult to get running. The tricky part 
was initially deserializing `PoseTimeArray` messages. But after I added the `base64` technique, I was able to obtain 
the message back, from which publishing to `/move_base_simple/goal` was straight forward. See the files for more 
details.

### MoveToBase
Here, I inherited from ExecuteTask and redefined two functions, execute and deserialize. This is because they are 
similar, but this node takes and initial pose `PoseStamped` while execute task takes in a `PoseTimeArray`.

#### Setup.py
Initially, I had to create a setup file in order to be able to install and import the execute_task module, because 
package import wasn't working.

First step was to create a `setup.py` file on the same level as `package.xml`.
Then I set
1. packages to my package name, and
2. package_dir to where my scripts where, i.e. the `scripts` folder

Uncomment `catkin_python_setup()`, then run `catkin_make`.

See the reference below for details of how to go about this process.

I had to create a package in the `scripts` folder with the name of the main package. 

Then finally, I realized I didn't need to go to all those stress. So I commented out `catkin_python_setup()` and just 
imported the module where I needed to. It works so far.

## ActionClients (BehaviorTree) Nodes
I created `BT` ActionsNodes with `actionlib` clients and Action definitions. They are all straightforward processes, 
simply create clients and communicate with the server and keep looping down the Nodes, receiving results from the 
servers and exchanging messages via blackboards. See the files `tree_nodes.h` and `behaviortree_node.cpp` in the src 
folder for details. 

## Launch Files
I copied the `turtlebot3_navigation.launch` file and edit it, commenting out the map_server section.
Added the gazebo launch file as an include statement.

I then created another launch file which has all my nodes.

## Execution
To run the codes, simply
1. Export the turtlebot model 
2. Source devel/setup.bash and deactivate conda 
3. `roslaunch` turtlebot3_navigation.launch, in one terminal
4. `roslaunch` behaviortree_navigation.launch, in another terminal
5. `rosrun` task_publisher.py, in yet another terminal

In future work we need to consider passing files via command lines.

# REFERENCES
1. [Subprocess](https://docs.python.org/3/library/subprocess.html#popen-objects)
2. [Serialize Python](https://gist.github.com/PeterMitrano/7daa8a993d7d05bd6f440daff7299f6a)
3. [Serialize/Deserialize C++](http://wiki.ros.org/roscpp/Overview/MessagesSerializationAndAdaptingTypes)
4. [Kill `roscore`](https://answers.ros.org/question/172879/kill-other-roscoremaster-processes-on-bourn-shell/)
5. [Import Python Modules](https://roboticsbackend.com/ros-import-python-module-from-another-package/)
