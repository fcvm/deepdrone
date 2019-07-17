#!/usr/bin/env bash

cd ~/src/Firmware/
export IGN_IP=127.0.0.1

#gnome-terminal --tab --tab --tab

# http://answers.gazebosim.org/question/21103/exception-sending-a-message/
#gnome-terminal --tab -e export IGN_IP=127.0.0.1 --tab -e export IGN_IP=127.0.0.1 --tab -e export IGN_IP=127.0.0.1

# Run mavros on local host, start px4 sitl simulation, start trajectory following node
#gnome-terminal --tab -e "roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"" --tab -e "make px4_sitl_default gazebo" --tab -e "rosrun offb offb_node" # && exit
#gnome-terminal --tab -e "roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"" --tab -e "make posix_sitl_default gazebo" --tab -e "rosrun offb offb_node" # && exit

#gnome-terminal --tab -e "roslaunch px4 mavros_posix_sitl-iris_fpv_cam.launch" --tab -e "rosrun offb offb_node" # && exit
gnome-terminal --tab -e "roslaunch px4 mavros_posix_sitl.launch sdf:=$px4_dir/Tools/sitl_gazebo/models/iris_fpv_cam/iris_fpv_cam.sdf" --tab -e "rosrun offb offb_node" # && exit


#gnome-terminal --tab -e "roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"" --tab -e "make px4_sitl_default gazebo_typhoon_h480" --tab -e "rosrun offb offb_node" # && exit







