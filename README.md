# Blue Arm
## Setup
  Install some packages
  ```sh
  sudo apt-get install ros-$ROS_DISTRO-moveit
  sudo apt-get install ros-$ROS_DISTRO-joint-state-controller
  sudo apt-get install ros-$ROS_DISTRO-joint-state-publisher
  sudo apt-get install ros-$ROS_DISTRO-joint-trajectory-controller
  sudo apt-get install ros-$ROS_DISTRO-position-controllers
  sudo apt-get install ros-$ROS_DISTRO-velocity-controllers
  sudo apt-get install ros-$ROS_DISTRO-effort-controllers
  ```
  Fork and clone the repository, then install the maxon epos2 driver by running:
  ```sh
  cd blue_arm/maxon_epos2/EPOS_Linux_Library/
  ./install.sh
  cd $CATKIN_WS
  catkin_make
  ```
  ## Run this program
    Run with Gazebo
    ```sh
    roslaunch blue_arm_moveit demo_gazebo.launch
    ```
    Run with real robot
    ```sh
    roslaunch manipulator manipulator_maxon.launch
    roslaunch blue_arm_moveit demo.launch
    ```
