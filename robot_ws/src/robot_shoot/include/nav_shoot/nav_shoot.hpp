#ifndef ROBOT_SHOOT_NAV_SHOOT_H
#define ROBOT_SHOOT_NAV_SHOOT_H

#include <chrono>
#include <memory>

#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <sensor_msgs/LaserScan.h>

#include <ros/ros.h>

#include "robot_task.hpp"


class NavShoot {
public:
    NavShoot(TaskQueue task_queue);
    ~NavShoot();

    void init();  // 初始化变量

    void shoot_debug();

    void loop_work();  // 开始运行

    void shoot_tag();            // 射击目标靶
    void position_correction();  // 位置纠正

    void pub_pose(double x, double y, double yaw_degrees);  // 发布自主导航位置点与一个角度

    void action_result_callback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg);  // 获取小车动作执行结果
    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);                     // 获取小车的姿态信息
    void laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);                   // 更新小车的右方与后方的距离信息
    void marker_tag_callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);        // 获取AR码识别结果

private:
    void _work(const ros::WallTimerEvent& event);  // 任务执行主函数, 循环调用主体

    ros::NodeHandle _nh;

    std::chrono::system_clock::time_point _start_time;  // 开始时间, 调用loop_work()时就重置这个时间

    TaskQueue _task_queue;  // 任务队列

    std::shared_ptr<BaseTask> _curr_task;
    std::shared_ptr<PoseTask> _pose_task;
    std::shared_ptr<CorrectTask> _correct_task;
    std::shared_ptr<ShootTask> _shoot_task;

    bool _marker_found;  // AR码目标检测标识
    double _vel_x;       // 瞄准时的旋转速度设置
    double _offset_x;    // 目标靶标的偏差

    double _front_dist_curr;
    double _rear_dist_curr;   // 获取当前的后方的距离
    double _left_dist_curr;
    double _right_dist_curr;  // 获取当前的右方的距离

    double _yaw_curr;  // 当前小车的偏航角

    // 当前小车的位置坐标
    double _curr_x;
    double _curr_y;

    ros::WallTimer _wall_timer;  // 定时器

    ros::Publisher _cmd_vel_pub;  // 小车控制发布
    ros::Publisher _pose_pub;     // 小车姿态发布
    ros::Publisher _shoot_pub;    // 射击信息发布

    ros::Subscriber _move_result_sub;
    ros::Subscriber _pose_sub;
    ros::Subscriber _scan_sub;
    ros::Subscriber _ar_pose_sub;
};

#endif  //ROBOT_SHOOT_NAV_SHOOT_H