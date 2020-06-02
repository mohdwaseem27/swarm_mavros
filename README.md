# swarm formations

## To start the simulations go to the folder where you have cloned Firmware

`cd src/Firmware`

## Build the SITL code

`git submodule update --init --recursive`

`DONT_RUN=1 make px4_sitl_default gazebo` 

## Source your environment

`source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default`

`export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/sitl_gazebo`

## Run launch file

`roslaunch px4 multi_uav_mavros_sitl.launch`

This will open the three iris quadcopter models in Gazebo

## On another tab run 

`rosrun swarm_mavros swarm_formation`

or

`rosrun swarm_mavros swarm`

This automatically enables offboard mode and arms the vehicle. The three vehicles will move and form predefined positions

