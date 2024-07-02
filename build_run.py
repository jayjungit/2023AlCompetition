import sys
import os


if len(sys.argv)< 2:
    wn = input("workspace name is empty, please enter your workspace name: ")
else:
    wn = sys.argv[1]

curr_path = os.getcwd()

wn_path = os.path.join(curr_path, wn)
if not os.path.exists(wn_path):
    print(f"workapce name error: {wn_path} is not exists")
    sys.exit()

setup_path = os.path.join(wn_path, "devel/setup.bash")

print(f"current path:   {curr_path}")
print(f"workspace path: {wn_path}")
print(f"dep path:       {curr_path}/abot_vision\n")

pattern = "--tab -e 'bash -c \"sleep {}; source {}; roslaunch {} {}; exec bash\"' \\\n"


def write_gmapping():
    with open("gmapping.sh", "w", encoding="UTF-8") as fp:
        fp.write("gnome-terminal --window -e 'bash -c \"roscore; exec bash\"' \\\n")
        fp.write(pattern.format(3, setup_path, "abot_bringup", "robot_with_imu.launch"))
        fp.write(pattern.format(4, setup_path, "robot_slam",   "gmapping.launch"))
        fp.write(pattern.format(4, setup_path, "robot_slam",   "view_mapping.launch"))
        fp.write("--tab -e 'bash -c \"sleep 4; rosrun teleop_twist_keyboard teleop_twist_keyboard.py; exec bash\"' \\\n ")
    print("write file \"gmapping.sh\" done")


def write_shoot():
    with open("shoot.sh", "w", encoding="UTF-8") as fp:
        fp.write("gnome-terminal --window -e 'bash -c \"roscore; exec bash\"' \\\n")
        fp.write(pattern.format(3, setup_path, "abot_bringup", "robot_with_imu.launch"))
        fp.write(pattern.format(3, setup_path, "abot_bringup", "shoot.launch"))
        fp.write(pattern.format(3, setup_path, "robot_slam",   "navigation.launch"))
        fp.write(pattern.format(3, setup_path, "track_tag",    "usb_cam_with_calibration.launch"))
        fp.write(pattern.format(3, setup_path, "track_tag",    "ar_track_camera.launch"))
        fp.write(pattern.format(3, setup_path, "robot_shoot", "nav_shoot.launch"))
    print("write file \"shoot.sh\" done")


if __name__ == "__main__":
    write_gmapping()
    write_shoot()
    print("==============> script run done")

