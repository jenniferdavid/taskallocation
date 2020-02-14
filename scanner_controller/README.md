# Docking controller (using only scanner)
----------------
Author: Ali Murtatha Shuman
Email: ali.m.shuman@gmail.com
phone: +46 76 26 55 339

## Overview
-----------
The scanner_controller package provides three PID-controllers, which control the x- and y-translation as well as the "theta"-rotation. The coordinate system of the controller is illustrated in the figure below. The x-axis resembles the front of the platform.

                    ^ x
                    | 
                    | 
                    |
        y < ---------

### Controller different modes
--------
The controller has three different modes 
1. NO_TASK      : Waiting for a signal to perform a DOCKING_TASK or a BAKING_TASK. The "NO_TASK" mode has the two following states:
    1. state = WAIT_TO_DOCK     : when it is waiting for a DOCKING_TASK.
    2. state = DOCKING_ENDED    : when it is waiting for a BACKING_TASK .

2. DOCKING_TASK :  Switches to this task only if it receives a docking signal from the "start_dock" topic (only if it has true value) and while having a WAIT_TO_DOCK state, as well as receiving a new target position from the "docking_pose" topic. When the controller finishes the docking procedure it switch to a NO_TASK with the DOCKING_ENDED state.
While processing the docking procedure it switches to the DOCKING state.

3. BACKING_TASK : Switches to this task only if it receives a backing signal from the "start_dock" topic (only if it has false value) and while having a DOCKING_ENDED state. When the controller finishes the backing procedure it switches to a NO_TASK with the WAIT_TO_DOCK state.
While processing the backing procedure it switches to the BACKING state.

The controller publishes a "docking_state" topic aswell, witch indicates which state the controller is currently on. The topic
type is "std_msgs::UInt8" where the different states are coded as follows:
- WAIT_TO_DOCK:     0
- DOCKING:          1
- DOCKING_ENDED:    2
- BACKING:          3


## Building 
-----------
catkin_make

## Usage
--------
You can launch the module as follows:
launching with default parameters. 
> roslaunch scanner_controller scanner_navigation.launch

launching with new parameters file.
> roslaunch scanner_controller scanner_navigation.launch config_file:=new_config_file.yaml

## Launch file argument 
By default the launch file uses the scanner_controller_config.yaml file as an argument which is stored inside the config folder. This provides the required parameters for the package.

- **[config_file]**: A .yaml file containing the required parameters for the package, set by default to config/scanner_controller_config.yaml. 

NOTE: The package launch file including the launch file of the following package:
- positioning : positioning.launch

## Parameter file (controller_config.yaml)
---------------------------------------
The main parameters that defines the functionality of the package are stored in controller_config.yaml file inside the config folder. The required parameters are:

- **[BACKING_DISTANCE]**: The required backing distance [m] (in negative x-direction) when the controller performs the BACKING_TASK procedure.  Set by default to 1.0 m.
- **[TIME_DURATION]**: The time duration between each published topic [s], it is also used for estimating the current position. Set by default to 0.1 s.

##### Transformation parameters
- **[SCANNER_TO_CENTER_POS]**: The distance between the scanner and the center position of the platform, in the x-axis [m]. Set by default to 0.41 m (for the Ridgeback).
- **[PLATFORM_LENGTH_CENTER_TO_EDGE]**: The distance between the center position of the platform and the frontal edge of the platform, in the x-axis [m]. Set by default to 0.48 m (for the Ridgeback).

##### Accepted stop position error
- **[ACCEPTED_X_ERROR]**: The accepted stop position before reaching the target position (x-axis) [m] , set by default to 0.01 m.
- **[ACCEPTED_Y_ERROR]**: The accepted stop position before reaching the target position (y-axis) [m], set by default to 0.01 m.
- **[ACCEPTED_THETA_ERROR]**: The accepted stop angle before reaching the target angle [rad], set by default to 0.035 rad.

##### Platform speed limitation
- **[MAX_LINER_SPEED]**: The maximum translation speed for both x- and y-translations [m/s], set by default to 0.2 m/s.
- **[MIN_LINER_SPEED]**: The minimum translation speed for both x- and y-translations [m/s], set by default to 0.01 m/s.
- **[MAX_ANGULAR_SPEED]**: The maximum rotation speed [rad/s], set by default to 0.6 rad/s
- **[MIN_ANGULAR_SPEED]**: The minimum rotation speed [rad/s], set by default to 0.02 rad/s

##### Control parameters
- **[X_KP]**: The proportional gain for the x-translation PID-conroller, set by default to 0.2.
- **[X_KD]**: The derivative gain for the x-translation PID-conroller, set by default to 0.1.
- **[X_KI]**: The integral gain for the x-translation PID-conroller, set by default to 0.01.


- **[Y_KP]**: The proportional gain for the y-translation PID-conroller, set by default to 1.0.
- **[Y_KD]**: The derivative gain for the y-translation PID-conroller, set by default to 0.3.
- **[Y_KI]**: The integral gain for the y-translation PID-conroller, set by default to 0.01.


- **[THETA_KP]**: The proportional gain for the theta-rotation PID-conroller, set by default to 0.5.
- **[THETA_KD]**: The derivative gain for the theta-rotation PID-conroller, set by default to 0.1.
- **[THETA_KI]**: The integral gain for the theta-rotation PID-conroller, set by default to 0.01.
