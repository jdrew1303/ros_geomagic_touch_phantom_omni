# What is this?
This is a ROS metapackage for both Sensable Phantom Omni (IEEE1394 connection) 
and Geomagic Touch (Ethernet connection). It publishes the device information 
such as joint angles, teleoperation data, buttons state, tip of stylus pose and 
twist to /omniFirewire or /omniEthernet by default. It also subscribes to some
topics, like .../control, .../enable_control and .../teleop, the latter only if 
the omni was launched in slave mode. If you don't know or don't have ROS, you
should first go to http://www.ros.org/.

# Dependencies:
The list of dependencies is quite large, but in order to facilitate the end user
life, we decided to make a Docker image. Running through docker is recommended
in cases where the user doesn't want to waste time setting up the environment or
wants complete isolation from the Omni space. If docker is your way to go, skip 
to docker session. If you don't mind docker, proceed to final steps once all
dependencies are installed.

Some of these packages will probably be already installed in your PC. If you're
not sure, run the command anyway.

- build-essential:
```sh 
$ sudo apt-get install build-essential 
```

- libncurses 5:
```sh
$ sudo apt-get install libncurses5-dev
```

- freeglut 3:
```sh
$ sudo apt-get install freeglut3
```

- dh-autoreconf:
```sh
$ sudo apt-get install dh-autoreconf
```

- MoveIt! for ROS-Indigo:
```sh
$ sudo apt-get install ros-indigo-moveit
```

- Joy drivers:
```sh 
$ sudo apt-get install ros-indigo-joy 
$ sudo apt-get install ros-indigo-joystick-drivers 
```

- Eigen:
```sh 
$ sudo apt-get update && apt-get install libeigen3-dev 
```

- IEEE1394 drivers:
```sh 
$ sudo apt-get update && apt-get install libraw1394-dev
```

- USB:
```sh 
$ apt-get update && apt-get install libusb-dev 
```

- Boost:
```sh 
$ apt-get update && apt-get install libboost-all-dev 
``` 

# Geomagic Touch Device Drivers and OpenHaptics:
As I'm not sure if I may post direct download links here, i'll post the link to 
download directly from Sensable.
https://3dsystems.teamplatform.com/pages/102863?t=fptvcy2zbkcc
From there, you'll need both the OpenHaptics and the GTDD files. Just extract it
and run the install script.

# The Docker way:
First, what you should do is install docker for your OS. A tutorial for Ubuntu 
can be found in the link below:
https://www.docker.com/docker-ubuntu
From there, you just need to download the PhantomOmni docker image or build your
own using the Dockerfile provided.

- Build command:
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

- Run command:
Once the image is built, which will take quite a while, you can run it using the
following command:
```sh 
$ sudo docker run --device=/dev/fw1 -ti --rm --privileged -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -ti --device=/dev/bus/usb/001/005 --net=host gscar:PhantomOmni
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