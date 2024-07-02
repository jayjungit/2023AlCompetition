gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3; source /home/abot/UnmannedCarRaces/UnionTmp2/robot_ws/devel/setup.bash; roslaunch abot_bringup robot_with_imu.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source /home/abot/UnmannedCarRaces/UnionTmp2/robot_ws/devel/setup.bash; roslaunch robot_slam gmapping.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source /home/abot/UnmannedCarRaces/UnionTmp2/robot_ws/devel/setup.bash; roslaunch robot_slam view_mapping.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; rosrun teleop_twist_keyboard teleop_twist_keyboard.py; exec bash"' \
 