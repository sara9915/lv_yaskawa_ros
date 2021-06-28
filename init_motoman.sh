read -p "Starting roscore..." n
xterm -e roscore &

read -p "Load Robot URDF..." n
(sleep 3; xterm -e rosparam load ~/catkin_visservo/src/add/motoman_sia5d_support/urdf/sia5d.urdf robot_description) &

read -p "Wait for launching robot_interface_streaming launch..." n
(sleep 3; xterm -e roslaunch motoman_driver robot_interface_streaming_fs100.launch robot_ip:=192.168.2.101) &

read -p "Wait for launching the node..." n
(sleep 3; xterm -e rosrun motoman_command motoman_controll_node) &
