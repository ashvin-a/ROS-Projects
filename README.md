# ROS 2 Repository

Go through this readme to understand some basics of ROS. In order to use ROS,
you have to create a workspace. The packages that you will build later on will 
be in this workspace. 

The command used for building a workspace is as follows:

> mkdir -p ~/ros2_ws/src

Just a bash command. After this, we can build the workspace by running this command:

> colcon build

A thing to note: You should build your workspace by standing at the ros2_ws directory.

This command will create some folders which are essential for building the application. 
In the `install` folder created, you can see a file named `setup.bash`. Add the path to this file
to bashrc.

After creating the workspace, we need to create the packages int it.
It can be built using this command:

> ros2 pkg create my_robot_controller --build-type ament_python

We can either use ament_python or ament_cmake. Also, a good convention to follow while building the packages is to put the name of the robot in the first and the functionality of the package in the latter part. Here, you can say the name is `my_robot` and the functionality of the package is a `controller`

Ament is the build system. Colcon is the build tool. 

We can add the dependencies to the above command line like this:

> ros2 pkg create my_robot_controller --build-type ament_python --dependencies rclpy

## Running a package

We can run a package by using this command:

> ros2 run polo_controller test_node

The `test_node` is the node which is mentioned in the setup.py under `entry_points`. 

If you go and check `node.py`, you can see that `thala_node` is actually the node's name. `test_node` is the executable name.

Thus,
node -> File name
thala_node -> Node name
test_node -> Executable name

In order to reflect the changes into the new changes in code without building it every single time, we can use this command:

> colcon build --symlink-install


## Topics

They are used for transfering data from one node to another.
The node which sends data to the topic is called the `Publisher` and the one that receives is called `Subscriber`.

### Common Methods in topics

- ros2 topic list -> Lists out all the topics

- ros2 topic info /chatter -> Gives information about the Type, Publisher and Subscriber count.

We can look into what is being sent by using this:
> ros2 interface show std_msgs/msg/String

Here, std_msgs/msg/String is the type of /chatter(example)

We can print the data from the topic like this:
> ros2 topic echo /chatter

