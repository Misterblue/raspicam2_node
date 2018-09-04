# raspicam_node

ROS2 node for the Raspberry Pi Camera Module.
Works with both the V1.x and V2.x versions of the module.
We recommend using the v2.x cameras as they have better auto gain, and the general image quality is better. 

[This repository](https://github.com/Misterblue/raspicam2_node) is @misterblue changes to make this work
on a Raspberry Pi 3 with ROS2 [Bouncy Bolson release](https://github.com/ros2/ros2/wiki/Release-Bouncy-Bolson) .
There were some compile errors from the [base repository](https://github.com/christianrauch/raspicam2_node) .

## Build Instructions

This is built as a stand-alone package.
So, presuming one followed [the instructions for building ROS2](https://github.com/ros2/ros2/wiki/Linux-Development-Setup)
with the usual defaults:

```bash
cd ~
source /opt/ros2/setup.bash
git clone https://github.com/Misterblue/raspicam2_node.git
cd raspicam2_node
ament build .
```

## Running the Node

Once built as above, the commands to run are:

```bash
cd raspicam2_node
source install/local_setup.bash
ros2 run raspicam2 raspicam2_node
```

TODO: there are parameters so document them.


## Troubleshooting
1. Make sure that your user is in the `video` group by running `groups|grep video`.

2. If you get an error saying: `Failed to create camera component`,
make sure that the camera cable is properly seated on both ends, and that the cable is not missing any pins. If this doesn't work update your firmware with `rpi-update`.

3. If the publish rate of the image over the network is lower than expected, consider using a lower resolution to reduce the amount of bandwidth required.

## Node Information
TODO
