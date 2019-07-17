### Creating a workspace for catkin ###
source /opt/ros/melodic/setup.bash
mkdir -p ~/Deepdrone/catkin_ws/src
cd ~/Deepdrone/catkin_ws/
catkin build
source devel/setup.bash

# Warning catkin build, added to drone_racetrack/CMakeLists.txt >> https://answers.ros.org/question/222139/how-to-disable-policy-cmp0045-is-not-set-warnings/
cmake_policy(SET CMP0054 OLD)
cmake_policy(SET CMP0045 OLD)

# Run Gazebo with VPN >> http://answers.gazebosim.org/question/21103/exception-sending-a-message/
export IGN_IP=127.0.0.1

