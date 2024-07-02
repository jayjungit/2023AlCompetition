gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3; source /home/abot/UnmannedCarRaces/UnionTmp2/robot_ws/devel/setup.bash; roslaunch abot_bringup robot_with_imu.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; source /home/abot/UnmannedCarRaces/UnionTmp2/robot_ws/devel/setup.bash; roslaunch abot_bringup shoot.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; source /home/abot/UnmannedCarRaces/UnionTmp2/robot_ws/devel/setup.bash; roslaunch robot_slam navigation.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; source /home/abot/UnmannedCarRaces/UnionTmp2/robot_ws/devel/setup.bash; roslaunch track_tag usb_cam_with_calibration.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; source /home/abot/UnmannedCarRaces/UnionTmp2/robot_ws/devel/setup.bash; roslaunch track_tag ar_track_camera.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; source /home/abot/UnmannedCarRaces/UnionTmp2/robot_ws/devel/setup.bash; roslaunch robot_shoot nav_shoot.launch; exec bash"' \
