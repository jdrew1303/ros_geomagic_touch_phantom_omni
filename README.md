# What is this?
This is a ROS metapackage for both Sensable Phantom Omni (IEEE1394 connection) 
and Geomagic Touch (Ethernet connection). Included in this metapackage is the
omni_driver package, which contains the omni_driver node. If you don't know or don't have ROS, you
should first go to http://www.ros.org/.

# Contents

## Topics
The omni_driver node publishes information to four topics:
- `[prefix]/joint_states` ([sensor_msgs/JointState](http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html)) -- position, velocity and name of each joint.
- `[prefix]/pose` ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html)) -- stylus tip pose with a timestamp.
- `[prefix]/twist` ([geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)) -- stylus tip linear and angular velocities.
- `[prefix]/button_state` (omni_driver/OmniButtonEvent) -- state (pressed/released) of the device buttons.

Besides, two topics are available to interact with the robot:
- `[prefix]/control` ([geometry_msgs/Vector3](http://docs.ros.org/api/geometry_msgs/html/msg/Vector3.html)) -- each component is read between [-1, 1] and controls a single motor. The (x, y, z) components command the joints 1, 2 and 3, respectively.
- `[prefix]/enable_control` ([std_msgs/Bool](http://docs.ros.org/api/std_msgs/html/msg/Bool.html)) -- true to enable the control, and false disable it.
 
If the node was launched as a slave for teleoperation (by setting the parameter `~teleop_master` to false) the node also subscribes to an additional topic:
- `/teleop` (omni_driver/TeleopControl) -- the device reads the velocity values and attempts to mimic them.

Otherwise, the it is assumed that the node is a teleoperation master (i.e. `~teleop_master` defaults to true) and the node published the additional topic
- `[prefix]/teleop` (omni_driver/TeleopControl) -- control to teleoperate another robot.

## Parameters
As you should have notices by now, most of the topics can be prefixed. This is done by altering the parameter `~omni_name` and/or running the node from a [group](http://wiki.ros.org/roslaunch/XML/group) with namespace inside a launch file. Below we list the available parameters:
| Parameter Name  |  Type  |      Options       | Default Value | Description |
| :-------------- | :----: | :----------------: | :-----------: | :---------- |
| `~omni_name`    | string | any                | omni          | Prefix that is put before many of the topics names |
| `~omni_type`    | string | firewire, ethernet | firewire      | The communication type |
| `~omni_serial`  | string | any                |               | The serial number printed below the device |
| `~teleop_master`|  bool  | true, false        | true          | The teleoperation mode (true for master, false for slave) |
| `~path_urdf`    | string | any                |               | Path to the URDF location |
| `~path_srdf`    | string | any                |               | Path to the SRDF location |

# Dependencies
The list of dependencies is quite large, so we decided to provide a Docker image with
everything ready to run. Therefore, using this Software through docker is recommended because very little
setting up is needed to get the program up and running. If Docker is your way to go, skip 
to **The Docker way** session. Otherwise, you may choose to install the dependencies and compile the program
from source. Once all dependencies are stored you may proceed to **Final steps**.

The following command will install the required dependencies to compile the program from source:
```sh
$ sudo apt-get update
$ sudo apt-get install build-essential libncurses5-dev freeglut3 dh-autoreconf \
    ros-indigo-ros-core ros-indigo-moveit ros-indigo-joy ros-indigo-joystick-drivers \
    libeigen3-dev libraw1394-dev libusb-dev libboost-all-dev 
```
This commands installs build-essential, libncurses 5, freeglut 3, dh-autoreconf, ROS-Indigo, MoveIt! and joy drivers for ROS-Indigo, Eigen, Boost, IEEE1394 (firewire) driver, and libusb-dev.

### Geomagic Touch Device Drivers and OpenHaptics:
As I can't distribute Sensable's software, those you'll have to download directly
from them. Please note that to access the download section, you may have to make
an account on their forum.

- https://3dsystems.teamplatform.com/pages/102863?t=fptvcy2zbkcc

From there, you'll need both the OpenHaptics and the GTDD files. Just extract it
and run the install script.

# The Docker way:
First, what you should do is install docker for your OS. A guide for Ubuntu 
can be found in the link below:

- https://www.docker.com/docker-ubuntu

From there, you just need to download the PhantomOmni docker image or build your
own using the Dockerfile provided.
### Build command:
```sh 
$ cd <Dockerfile Path Location>
$ sudo docker build -t <tag_prefix:tag_suffix> .
```
Ex: My Dockerfile is in ~/Downloads/docker/Dockerfile and it will be named 
gscar:PhantomOmni.
```sh 
$ cd ~/Downloads/docker
$ sudo docker build -t gscar:PhantomOmni .
```
Please note that the final dot is needed.
By default, both username and password on this image is omni.

### Run command:
Once the image is built, which will take quite a while, you can run it using the
following command:
```sh 
$ sudo docker run -ti --rm --privileged -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --net=host gscar:PhantomOmni
```

# Final steps:
At this point the firewire version should already be functional, but the 
ethernet still need two steps to work. First, you must change your IPV4 to
link-local only. 
Second, you need to generate your config files for GTDD. This 
works the same if you're doing the normal or docker way, but if you're using the 
docker way, make sure you do these steps running the image.
```sh 
$ cd /opt/geomagic_touch_device_driver
$ ./Geomagic_Touch_Setup 
```
A GUI should appear and you should select the Geomagic Touch as device model.
Then, click the "Add..." button and give your device the following name: Phantom
Omni. Just like that, starting with Phantom, separated with a blank space and 
finishing with Omni. If the name is different, the ROS Driver won't be able to 
find the device. Next, you have to pairyour device, selecting available devices
and clicking the " Pairing" button first on the GUI, then on the device itself. 
If your device is not listed here, check the device communication running 
/opt/geomagic_touch_device_driver/Geomagic_Touch_Diagnostic . Most of times, this happens because the IPV4 isn't
a link-local only connection. Now everything is ready to go, but if you're using
docker you'll have to generate config files everytime you exit and rerun the 
image, as this is the docker way. If you want to commit the changes done to a 
container, run the following command in a host terminal while the images is 
still running:
```sh 
$ sudo docker container ls 
```
From there, copy the Container ID and use it in the following command:
```sh 
$ sudo docker commit <ContainerID> <tag_prefix:tag_suffix>
```
This will save the image running on ContainerID to a tag_prefix:tag_suffix image
name. If you use the same name as the running image, it will be overwritten.

# Launch files:
If you want to use a firewire device, run:
```sh 
$ roslaunch omni_driver firewire.launch
```
If ethernet is the one you're using, then:
```sh 
$ roslaunch omni_driver ethernet.launch
```
If you want to teleoperate one of them, start the Master as you would normally
then run the slave i.e:
```sh 
$ roslaunch omni_driver firewire.launch
$ roslaunch omni_driver ethernet_slave.launch
```
If you want visualization of the device, then:
```sh 
$ roslaunch omnidriver firewire_rviz.launch
```
