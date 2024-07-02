#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>


void laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    std::vector<float> ranges = msg->ranges;

    float angle_increment = msg->angle_increment;
    
    float angle_min = msg->angle_min;
    float angle_max = msg->angle_max;
    
    int num_readings = (angle_max - angle_min) / angle_increment;

    float _front_dist_curr = ranges[num_readings / 2]; // update distance front
    float _rear_dist_curr  = ranges[num_readings - 1]; // update distance back

    int quarter_index = num_readings / 4;
    float _left_dist_curr   = ranges[num_readings - quarter_index]; // update distance left
    float _right_dist_curr  = ranges[quarter_index];               // update distance right

    ROS_INFO("[%.3F]  |  [%.3F]  |  [%.3F]  |  [%.3F]", _front_dist_curr, _rear_dist_curr, _left_dist_curr, _right_dist_curr);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_dist");

    ros::NodeHandle _nh("~");

    ros::Subscriber _scan_sub = _nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, &laser_scan_callback);  // 订阅激光雷达信息
    ros::Rate loop(1000);
    while (true) {
        loop.sleep();
        ros::spinOnce();
    }
    ros::spin();
    return 0;
}