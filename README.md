# LiDAR SLAM for ROS2
This is Group 1's from ARIA Lab's Summer Internship LiDAR SLAM repository. This is all the code that we used in order to run the following on ROS2: KissICP, ROS2 LiDAR Drivers, LIOSAM, and Msg Conversion.

## Set Up and Configure
Setting up should be easy as all packages have been cleaned up in a way that will build cleanly. Simply run `colcon build` to build everything!

For the ROS2 Drivers located inside of `ws_livox` it's important to edit the `{type}_config.json` file for the driver you want to use. Be sure that the host IP is set to the right one, and that the LiDAR IP is set to match the LiDAR in use.

_I recommend always defaulting to MID360 as it's newer, but always consult documentation and the internet to confirm._

_A more detailed tutorial on configuring can be found [here](https://docs.google.com/document/d/1Tqu_g_ehYhEzqJJr4pIFhBEOT2dJlDr-lP0_Bo0P9jo/edit?tab=t.0), else feel free to message Caitlyn on Discord for further assistance._

## General Documentation for Use
If you're looking to run a full SLAM system I suggest opening two terminals and running `colcon build` followed by `source install/setup.bash` to ensure that each terminal is set up properly. You can prepare each window in advanced with the following commands:

1) `ros2 launch livox_ros_driver2 msg_{driver type}_launch.py` __(make sure the config files are set and the LiDAR is on and running)__
2) `ros2 launch kiss_icp odometry.launch.py topic:={topic_name}` __(you can optionally pass a bag with bagfile:={path to your bag})__
3) `ros2 run livox_to_cloudpoint2 livox_converter` __(optional: runs the live msg converter for KissICP to process data properly)__

Alternatively if you don't want to use the live converter there is a Python script that will convert a recorded bag into PointCloud2 messages for KissICP. To run that use `python3 livox_bag_to_pointcloud2.py --ros-args -p input_bag_path:={your bag path}`

The converted bag will save in a sub folder within the original bag's folder.
