# terrain_property_prediction

## Getting started

Firstly, clone this package using the following command.

```
https://github.com/brolinA/terrain_property_prediction.git --recurse-submodules --branch mini-project

```

Check if you are in the `mini-project` branch.

Then follow the instruction to download and install all the dependencies for **wild visual navigation** from [here](https://github.com/leggedrobotics/wild_visual_navigation/tree/main). Make sure that you have the `wild_visual_navigation` package in the ROS workspace you are working on.

- You can skip either create the virtual environment mentioned in the WVN README or ignore it if you want to setup everything natively.
- You can also simply use

**Note:**

- While installing the dependencies mentioned in wild_visual_navigation and self_supervised_segmentation packages you might get some python dependency issues which is likely because of conflicting python package dependency. Unfortunately, you will have to deal with it yourself. I don't remember how I fixed it :wink:
- Make sure to do `catkin build` inside the virtual environment if you created it. If you managed to get everything installed in the native system then you don't have to worry about it.

## Aliengo simuation

We will be using the aliengo robot for this simulation.

To visualize the robot in gazebo and rviz, use the following command.

```
roslaunch aliengo_wild_visual_navigation aliengo_sim.launch
```

If you encounter repeated errors like the following, it is a [known issue](https://github.com/ms-iot/ROSOnWindows/issues/279) from 2020 in ROS Noetic.

```
[ WARN] [1719579313.021134038, 38.725000000]: TF_REPEATED_DATA ignoring data with redundant timestamp for frame camera_link (parent trunk) at time 39.225000 according to authority unknown_publisher
[ WARN] [1719579313.021149699, 38.725000000]: TF_REPEATED_DATA ignoring data with redundant timestamp for frame camera_rgb_frame (parent camera_link) at time 39.225000 according to authority unknown_publisher
[ WARN] [1719579313.021160935, 38.725000000]: TF_REPEATED_DATA ignoring data with redundant timestamp for frame camera_rgb_optical_frame (parent camera_rgb_frame) at time 39.225000 according to authority unknown_publisher
[ WARN] [1719579313.021171941, 38.725000000]: TF_REPEATED_DATA ignoring data with redundant timestamp for frame trunk (parent base) at time 39.225000 according to authority unknown_publisher
```

Use the following command to suppress the warning messages.

```
roslaunch aliengo_wild_visual_navigation aliengo_sim.launch 2> >(grep -v TF_REPEATED_DATA buffer_core)
```

You can run wild visual navigation using the following command.

```
 roslaunch aliengo_wild_visual_navigation aliengo_wild_visual_navigation.launch
```
