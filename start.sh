
sh ./net.sh $1
gnome-terminal -t "roscore" -x zsh -c "roscore"
gnome-terminal -t "bottol_control" -x zsh -c "source activate py27;LCM_DEFAULT_URL='udpm://224.0.0.1:7667?ttl=1' python control.py;exec zsh;"
gnome-terminal -t "bottol_info" -x zsh -c "cd ~/Code/LCMtest64/cmake-build-debug/;LCM_DEFAULT_URL="udpm://224.0.0.1:7667?ttl=1" ./LCMTest;exec zsh;"

gnome-terminal -t "vlp" -x zsh -c "source ~/catkin_ws/devel/setup.zsh;roslaunch velodyne_pointcloud VLP16_points.launch;exec zsh;"
gnome-terminal -t "stereo" -x zsh -c "source ~/Code/MYNT-EYE-D-SDK-master/wrappers/ros/devel/setup.zsh;roslaunch mynteye_wrapper_d mynteye.launch;exec zsh;"
sleep 1
gnome-terminal -t "record" -x zsh -c "rosbag record -o ~/data/vlp /velodyne_points /mynteye/imu/data_raw /mynteye/left/image_color;exec zsh;"
