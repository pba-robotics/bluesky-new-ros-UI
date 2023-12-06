#!/usr/bin/env bash

#Sourcing for internal development
source /home/user/vcc_ws/install/share/gr_bringup/config/environment.bash
source /opt/ros/noetic/setup.bash
source /home/user/vcc_ws/install/setup.bash
#source ~/carto_ws/devel_isolated/setup.bash

export ROS_MASTER_URI=http://$ROBOT_IP:11311;
export ROS_HOSTNAME=$ROBOT_IP;

echo "Launching $ROBOT_BASE-$ROBOT_SERIAL_NUMBER [$ROBOT_ID][$ROBOT_IP] \
     server: [$FMS_IP_ADDRESS:$FMS_PORT_NUMBER] \
     simulation=$ROBOT_SIMULATION"

echo "Active map: $ROBOT_MAP_LAYOUT"
echo "Mode:": $1

source /home/user/web_ws/devel/setup.bash
roslaunch web_GUI web_GUI.launch

roslaunch gr_bringup gr-blk61.launch mapping_launch:=$1 --wait
#roslaunch bc_bringup bc.launch mapping_launch:=$1


