# Table of Contents
1. [About](#about)
2. [Usage](#usage)
3. [Dependencies](#dependencies)
4. [The Docker Way](#the-docker-way)
5. [Final Steps](#final-steps)
6. [Troubleshooting](#troubleshooting)
7. [License](#license)

# About
This is a ROS metapackage for both Sensable Phantom Omni (IEEE1394 connection) 
and Geomagic Touch (Ethernet connection). Included in this metapackage is the
omni_driver package, which contains the omni_driver node. If you don't know or don't have ROS, you
should first go to http://www.ros.org/.

# Usage

## Topics
The omni_driver node publishes information to four topics:
- `[prefix]/joint_states` ([sensor_msgs/JointState](http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html)) -- position, velocity and name of each joint.
- `[prefix]/pose` ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html)) -- stylus tip pose with a timestamp.
- `[prefix]/twist` ([geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)) -- stylus tip linear and angular velocities.
- `[prefix]/button_state` (omni_driver/OmniButtonEvent) -- state (pressed/released) of the device buttons.

Besides, two topics are available to interact with the robot (the node subscribes to them):
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
| `~omni_type`    | string | FireWire, Ethernet | FireWire      | The communication type |
| `~omni_serial`  | string | any                |               | The serial number printed below the device |
| `~teleop_master`|  bool  | true, false        | true          | The teleoperation mode (true for master, false for slave) |
| `~path_urdf`    | string | any                |               | Path to the URDF location |
| `~path_srdf`    | string | any                |               | Path to the SRDF location |

## Launch Files
If you want to use a FireWire device, run:
```sh 
$ roslaunch omni_driver firewire.launch
```
Additionally, you need to update the serial number on the omni.launch file so that it matches yours. The Ethernet version doesn't need this step.
Go to omni_driver/launch/omni.launch and look for this line:
```sh
$ <param name="omni_type" type="string" value="$(arg type)" />
			<param name="omni_serial" type="string" value="11129400000" />
```
Then change the value to match your device's serial number.

If Ethernet is the one you're using, then:
```sh 
$ roslaunch omni_driver ethernet.launch
```
If you want to teleoperate one of them, start the Master as you would normally
then run the slave i.e:
```sh 
$ roslaunch omni_driver firewire.launch
$ roslaunch omni_driver ethernet_slave.launch
```

# Dependencies
The list of dependencies is quite large, so we decided to provide a Docker image with
everything ready to run. Therefore, using this Software through Docker is recommended because very little
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
This commands installs build-essential, libncurses 5, freeglut 3, dh-autoreconf, ROS-Indigo, MoveIt! and joy drivers for ROS-Indigo, Eigen, Boost, IEEE1394 (FireWire) driver, and libusb-dev.

## Geomagic Touch Device Drivers and OpenHaptics
As I can't distribute Sensable's software, those you'll have to download directly
from them. Please note that to access the download section, you may have to make
an account on their forum.

- https://3dsystems.teamplatform.com/pages/102863?t=fptvcy2zbkcc

From there, you'll need both the OpenHaptics and the GTDD files. Just extract it
and run the install script.

# The Docker Way
First, what you should do is install Docker for your OS. A guide for Ubuntu 
can be found in the link below:

- https://www.Docker.com/Docker-ubuntu

From there, you just need to download the PhantomOmni Docker image or build your
own using the Dockerfile provided.

## Downloading image
Downloading the image is very simple and is recommended in most cases. To download it, just run:
```sh
$ sudo docker pull brunogbv/phantomomni
```
As an optional step, you can rename it in order to use the examples below.
```sh
$ sudo docker tag brunogbv/phantomomni gscar:PhantomOmni
```

## Build command
```sh 
$ cd [Dockerfile Path Location]
$ sudo docker build -t [tag_prefix:tag_suffix] .
```
Ex: My Dockerfile is in ~/Downloads/Docker/Dockerfile and it will be named 
gscar:PhantomOmni.
```sh 
$ cd ~/Downloads/Docker
$ sudo docker build -t gscar:PhantomOmni .
```
Please note that the final dot is needed.
By default, both username and password on this image is omni.

## Run command
After downloading or building the image, you can run it using the
following command:
```sh 
$ sudo docker run -ti --rm --privileged -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --net=host --env QT_X11_NO_MITSHM=1 gscar:PhantomOmni
```


# Final steps
There are some additional steps required, but it's almost finished. These should be done whether you're using Docker or not.
## FireWire
In order to enable communications with the device, we need to set read and write permission's on the device to everyone. You
can check what is your device using **dmesg**. 

First, connect the device on the PC and wait a few moments. Then, run the following command:
```sh
$ dmesg | grep firewire
```
On my PC, I got the following output:
```sh
[    0.600300] FireWire_ohci 0000:05:00.0: enabling device (0000 -> 0003)
[    0.666343] FireWire_ohci 0000:05:00.0: added OHCI v1.10 device as card 0, 4 IR + 8 IT contexts, quirks 0x11
[    1.166421] FireWire_core 0000:05:00.0: created device fw0: GUID 0011066600000009, S400
[ 1243.251929] FireWire_core 0000:05:00.0: phy config: new root=ffc1, gap_count=5
[ 1249.803286] FireWire_core 0000:05:00.0: phy config: new root=ffc1, gap_count=5
[ 1252.790785] FireWire_core 0000:05:00.0: created device fw1: GUID 000b990080df6000, S400
```
The last device created was fw1, then that's my Phantom Omni. From now on, we just need to change its permissions:
```sh
$ sudo chmod a+rw /dev/fw1
```
If you're not sure which fw* is your Omni or doesn't bother setting all FireWire devices permission's, just run:
```sh 
$ sudo chmod a+rw /dev/fw*
```
This should be done everytime you reconnect the FireWire device.

## Ethernet
To use the Ethernet device, you must create a new Ethernet connection and set its IPV4 connection method to link-local only. 
Second, you need to generate your config files for GTDD. This 
works the same if you're doing the normal or Docker way. However, if running the program through Docker **make sure you run the following command from the virtual machine (aka container)**
```sh 
$ cd /opt/geomagic_touch_device_driver
$ ./Geomagic_Touch_Setup 
```
A GUI appears and you should select the Geomagic Touch as the device model.
Then, click the "Add..." button and give your device the following name: Phantom Omni. Just like that, starting with Phantom, separated with a blank space and 
finishing with Omni. If the name is different, the ROS Driver won't be able to 
find the device. Next, you have to pair your device, selecting available devices
and clicking the " Pairing" button first on the GUI, then on the device itself. 
If your device is not listed here, check the device communication running 
/opt/geomagic_touch_device_driver/Geomagic_Touch_Diagnostic . Most of the times, this happens because the IPV4 isn't
a link-local only connection. 

Now everything is ready to go, but if you're using
Docker you'll have to generate config files everytime you exit and rerun the 
image, as this is the Docker way. If you want to commit the changes done in the 
container, run the following command in a host terminal while the images is 
still running:
```sh 
$ sudo docker container ls 
```
From there, copy the Container ID and use it in the following command:
```sh 
$ sudo docker commit [ContainerID] [tag_prefix:tag_suffix]
```
This will save the image running on ContainerID to a tag_prefix:tag_suffix image
name. If you use the same name as the running image, it will be overwritten. I recommend preserving the original name
as to avoid confusion with created images.

# Troubleshooting
## FireWire
 - **[ERROR] [timestamp]: No firewire devices found.** -- Check the read and write permissions on /dev/fw*. Try reconnecting the device and running dmesg to see if connection was successfull.

## Ethernet
 - **[ERROR] [timestamp]: Failed to initialize haptic device** -- Check the config files on "/opt/geomagic_touch_device_driver/config". Please make sure the device was properly configured as explained on above. Please note that any name other than "Phantom Omni" will not work. If using the Docker Way, please make sure you committed the changes on the container after configuring the device before exiting, as changes are not automatically saved on Docker.

# License
This Software is distributed under the [MIT License](https://opensource.org/licenses/MIT).
