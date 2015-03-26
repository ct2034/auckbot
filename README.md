The Auckbot Simulation
===========

This packages contain the Simulation of the AuckBot. In particular:

- __`auckbot_analysis`__: Tools to analyse the robots motion and energy consumption
<br/>
- __`auckbot_description`__: The description of the robot for simulation
<br/>
- __`auckbot_gazebo`__: Simulation settings and environment
<br/>
- __`auckbot_navigation`__: Navigation launchfiles and maps (mainly wrappers of available packages)
  - __`planner_setup_selector.sh`__: allows to choose configurations of global and local planner using a simple UI (needs to be sourced because it uses environment variables)
  - __`amcl_move_base_lab.launch`__: starts the navigation for an specific environment (here *lab*)
<br/>
- __`auckbot_teleop`__: Little tool to manually control the robot
  - __`keyboard_teleop.launch`__: enables you to control the robot using your keyboard

## Requirements

The following packages need to be installed to run and build the code:<br/>
- `sudo apt-get install mongodb`<br/>
- `sudo apt-get install libgcal-dev`<br/>
- `sudo apt-get install libncurses5-dev`<br/>
- (optional) `sudo apt-get install robomongo`<br/>

## Building

To use this software, follow these steps:<br/>
(It is recommended to have look at some [tutorials](http://wiki.ros.org/ROS/Tutorials) if you are new to ROS)

1. Create a ROS workspace
  - `mkdir -p ~/ros/auckbot_ws/src`
  - `cd ~/ros/auckbot_ws/src`
  - `catkin_init_workspace`
  
2. Download these sources
  - `git clone git@github.com:ct2034/auckbot.git`
  - `git clone git@github.com:ct2034/navigation.git`
  
3. Build the code
  - `cd ~/ros/auckbot_ws`
  - `catkin_make`
  - `source devel/setup.bash`

4. (OPTIONAL) If you want to use rtabmap:
  - Get the code and compile it according to the available [tutorial](https://code.google.com/p/rtabmap/wiki/Installation#ROS_version)
  - Notes:
    - In case you experience an error like <br/>
    *No rule to make target `/usr/lib/x86_64-linux-gnu/libGL.so'*<br/>
    create a link to the correct location of the library:<br/>
    `sudo ln -s /usr/lib/libGL.so /usr/lib/x86_64-linux-gnu/libGL.so`
    - The folders rtabmaplib and rtabmap should both be located in the src folder of your workspace
    
## Running

### Simulation

You will need probably *(at least)* 3 terminal windows:
- To start the simulation run `roslaunch auckbot_gazebo auckbot_mudcircle.launch`. *Note that the UI of gazebo is switched off by default. Edit the launchfile to change this.* <br/>
- The basic navigation can be started with `roslaunch auckbot_navigation amcl_move_base.launch`. <br/>
- A preconfigured version of rviz can be started with `roslaunch auckbot_navigation rviz.launch`. In rviz you can know set navigation goals which will initiate the movement of the robot towards these.<br/>

### Robot Tests

Start on the robot:
- the connection to TwinCat and necessary hardware drivers: `roslaunch auckbot_bringup auckbot.launch` <br/>

Start from control pc using `ssh -X student@192.168.0.7`:
- select navigation config using: `source src/auckbot/auckbot_navigation/launch/planner_setup_selector.sh`
- **in the same terminal** start the localization and navigation: `roslaunch auckbot_navigation amcl_move_base_lab.launch`
- for safety start as well `roslaunch auckbot_teleop keyboard_teleop.launch`

Start on the control computer after setting up the connection using `ROS_IP` and `ROS_MASTER_URI`
- start the visualization: `roslaunch auckbot_navigation rviz.launch`

After a succesful naviagtion you can check using `robomongo` if the trip was saved to the database.




