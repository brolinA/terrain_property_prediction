### Aliengo Dynamics Computation

This pacakge will be used to compute dynamics related data for the aliengo robot.

#### Doxygen Documentation.

All the function and variables are documented using [rosdoc_lite](https://wiki.ros.org/rosdoc_lite). 

You can view the documentation by accessing the [index.html](doc/index.html) file.

**Note**: 
If you are updating function/code remember to comment them appropriately and then use the following command to update the `doxygen` documentation. Run the command when you are inside the package on the terminal.

```
rosdoc_lite .
```

#### Building the package.

You can use normal `catkin build` to build the package. If you get error regarding **missing Boost version**, then use the following command

```
catkin build aliengo_dynamics_computer -DBOOST_DIR=<your/absolute/path/to/boost>
```

In my case the path the boost installation was in `/usr/include`. You need to find your installation location and use it here.

You can also add `-DCMAKE_WARN_DEPRECATED=FALSE` to the build command to supress warning regarding deprecated cmake version 