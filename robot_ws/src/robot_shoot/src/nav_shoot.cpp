#include "nav_shoot/nav_shoot.hpp"

#include <chrono>
#include <cmath>
#include <vector>

#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

#include <Eigen/Dense>

#include "nav_shoot/nav_utils.hpp"


using namespace std;

using namespace geometry_msgs;
using namespace sensor_msgs;
using namespace move_base_msgs;
using namespace ar_track_alvar_msgs;

using namespace Eigen;


// 获取激光雷达的指定角度的距离信息, 不存在的角度则取最近的角度的距离信息
double get_dist_at_angle(const LaserScan::ConstPtr& scan_msg, double angle_degrees) {
    double angle   = angle_degrees * M_PI / 180.0;
    int num_ranges = scan_msg->ranges.size();

    int closest_index = static_cast<int>((angle - scan_msg->angle_min) / scan_msg->angle_increment + 0.5);

    if (closest_index < 0 || closest_index >= num_ranges) {
        return -1.0;
    }
    return scan_msg->ranges[closest_index];
}


NavShoot::NavShoot(TaskQueue task_queue) : _nh("~"), _task_queue(task_queue) {
    init();
}


NavShoot::~NavShoot() {}


// 初始化订阅与发布
void NavShoot::init() {
    _curr_task    = nullptr;
    _pose_task    = nullptr;
    _correct_task = nullptr;
    _shoot_task   = nullptr;

    _marker_found = false;

    // clang-format off
    _move_result_sub = _nh.subscribe<MoveBaseActionResult>("/move_base/result", 10, &NavShoot::action_result_callback, this);  // 订阅动作执行结果
    _pose_sub        = _nh.subscribe<PoseStamped>         ("/abot/pose",        1, &NavShoot::pose_callback,          this);  // 订阅姿态信no息
    _scan_sub        = _nh.subscribe<LaserScan>           ("/scan",              1, &NavShoot::laser_scan_callback,    this);  // 订阅激光雷达信息
    _ar_pose_sub     = _nh.subscribe<AlvarMarkers>        ("/ar_pose_marker",   10, &NavShoot::marker_tag_callback,    this);  // 订阅AR码识别信息

    _pose_pub    = _nh.advertise<PoseStamped>     ("/move_base_simple/goal", 10);  // 姿态信息发布
    _cmd_vel_pub = _nh.advertise<Twist>           ("/cmd_vel",               10);  // 命令信息发布, 用于控制小车
    _shoot_pub   = _nh.advertise<std_msgs::String>("/shoot",                 10);  // 炮台发射信息发布
    // clang-format on
}


// 启动小车运行
void NavShoot::loop_work() {
    ROS_INFO("开始时间: %s", get_current_time_str("%Y-%m-%d %H:%M:%S").c_str());
    _start_time = chrono::system_clock::now();
    if (_wall_timer.isValid()) {
        return;
    }
    _wall_timer = _nh.createWallTimer(ros::WallDuration(0.005), &NavShoot::_work, this);
}


// 小车的工作函数
void NavShoot::_work(const ros::WallTimerEvent& event) {
    if (_correct_task) {  // 处于纠正任务时, 进行位置纠正处理
        position_correction();
        return;
    }

    if (_shoot_task) {  // 处于射击任务时, 进行射击任务处理
        shoot_tag();
        return;
    }

    if (_pose_task) { return; }  // 处于自主巡航任务, 在未完成之前, 不进行任务切换

    // 当前任务为 nullptr 或者任务已经完成, 则从任务队列中提取一个任务
    if (_curr_task == nullptr || _curr_task->done) {
        _curr_task = _task_queue.front();
        _task_queue.pop();
    }

    // 如果 _current_task 为空, 说明任务队列已经为空, 所有任务已经完成
    if (_curr_task == nullptr) { 
        _pose_task    = nullptr;
        _correct_task = nullptr;
        _shoot_task   = nullptr;
        ROS_INFO("小车任务队列为空, 所有任务已经执行完成");
        ROS_INFO("结束时间: %s", get_current_time_str("%Y-%m-%d %H:%M:%S").c_str());
        ROS_INFO("小车执行任务用时: %s", get_time_diff_str(_start_time).c_str());
        _wall_timer.stop();
        ros::Duration(2).sleep();
        // ros::shutdown();
        return; 
    }

    // clang-format off
    switch (_curr_task->type) {
        case POSE_TASK: {
            // 自主导航任务, 在该任务中, 只需要进行一次pose发布即可 在 action_result_callback 中调函数中检测是否已经到达目标点
            _pose_task = dynamic_pointer_cast<PoseTask>(_curr_task);
            ROS_INFO_STREAM("\n" << *_pose_task);
            pub_pose(_pose_task->x, _pose_task->y, _pose_task->yaw_degrees);

            // 进行自主导航任务时, 需要先将纠正任务与射击任务设为空
            _correct_task = nullptr;
            _shoot_task   = nullptr;
            break;
        }
        case CORRECT_TASK: {
            // 纠正任务在该代码中需要循环调用 position_correction 来进行位置的纠正
            _correct_task = dynamic_pointer_cast<CorrectTask>(_curr_task);
            ROS_INFO_STREAM("\n" << *_correct_task);
            _shoot_task = nullptr;  // 设射击任务为空
            _pose_task  = nullptr;  // 设巡航任务为空
            break;
        }
        case SHOOT_TASK: {
            // 射击任务在该代码中需要循环调用 shoot_tag 来进行检测与射击处理
            _shoot_task = dynamic_pointer_cast<ShootTask>(_curr_task);
            ROS_INFO_STREAM("\n" << *_shoot_task);
            _correct_task = nullptr;  // 设纠正任务为空
            _pose_task    = nullptr;  // 设巡航任务为空
            break;
        }
        default:
            _curr_task = nullptr;
    }
    // clang-for.mat on
}


void NavShoot::shoot_debug() {
    ros::Rate loop(1000);
    int id         = 0;
    double yaw_th  = 0.10;
    double yaw_err = 0.0;

    auto temp_task = std::make_shared<ShootTask>(0, 0.1, 0.0);
    _curr_task     = std::make_shared<BaseTask>();

    while (true) {
        if (_shoot_task) {  // 处于纠正任务时, 进行位置纠正处理
            shoot_tag();
            loop.sleep();
            ros::spinOnce();
            continue;
        }

        cout << "输入 tag_id: ";
        cin >> id;
        
        cout << "输入 yaw_th: ";
        cin >> yaw_th;

        cout << "输入 yaw_err: ";
        cin >> yaw_err;

        temp_task->tag_id = id;
        temp_task->yaw_err = yaw_err;
        temp_task->yaw_th = yaw_th;
        ROS_INFO("%d %.3f %.3f", id, yaw_th, yaw_err);
        _shoot_task = temp_task;
        loop.sleep();
        ros::spinOnce();
    }
}

// 位置纠正, 以右方和后方的距离来定位目标位置
void NavShoot::position_correction() {
    Twist cmd_vel;

    // 检测右方距离是否符合要求
    if (_right_dist_curr > _correct_task->right_max)
        cmd_vel.linear.y = -_correct_task->vel;
    else if (_right_dist_curr < _correct_task->right_min)
        cmd_vel.linear.y = _correct_task->vel;

    // 检测后方距离是否符合要求
    if (_rear_dist_curr > _correct_task->rear_max)
        cmd_vel.linear.x = -_correct_task->vel;
    else if (_rear_dist_curr < _correct_task->rear_min)
        cmd_vel.linear.x = _correct_task->vel;

    // 如果纠正到指定范围, 则退出纠正位置任务, 任务目标完成
    if (!cmd_vel.linear.x && !cmd_vel.linear.y) {
        _curr_task->done = true;
        _correct_task    = nullptr;
        
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
    }

    // 发布纠正控制
    _cmd_vel_pub.publish(cmd_vel);
}


// 发布一个新位置点位坐标与一个角度
void NavShoot::pub_pose(double x, double y, double yaw_degrees) {
    // 将角度转换为弧度
    double yaw_radians = yaw_degrees * M_PI / 180.0;

    PoseStamped pose;

    pose.header.frame_id = "map";
    pose.header.stamp    = ros::Time::now();

    // 设置目标点位 x y
    pose.pose.position.x = x;
    pose.pose.position.y = y;

    // 使用 AngleAxisd 来指定偏航角度
    Eigen::AngleAxisd angle_axis(yaw_radians, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond quaternion(angle_axis);

    // 设置四元数来指定偏航角度
    pose.pose.orientation.x = quaternion.x();
    pose.pose.orientation.y = quaternion.y();
    pose.pose.position.z    = quaternion.z();
    pose.pose.orientation.z = quaternion.z();
    pose.pose.orientation.w = quaternion.w();

    // 发布姿态信息
    _pose_pub.publish(pose);
}


// 获取当前点位与偏航角  ---  [180, -180]度
void NavShoot::pose_callback(const PoseStamped::ConstPtr& msg) {
    // 获取当时点位 x y
    _curr_x = msg->pose.position.x;
    _curr_y = msg->pose.position.y;

    // 计算偏航角度数, 乘以 180.0 / M_PI 可以转换为角度
    double w  = msg->pose.orientation.w;
    double x  = msg->pose.orientation.x;
    double y  = msg->pose.orientation.y;
    double z  = msg->pose.orientation.z;
    _yaw_curr = atan2(2.0 * (z * w + x * y), 1.0 - 2.0 * (y * y + z * z));
}


// 更新小车的右方与后方的距离
void NavShoot::laser_scan_callback(const LaserScan::ConstPtr& msg) {
    std::vector<float> ranges = msg->ranges;

    float angle_increment = msg->angle_increment;
    
    float angle_min = msg->angle_min;
    float angle_max = msg->angle_max;
    
    int num_readings = (angle_max - angle_min) / angle_increment;

    _front_dist_curr = ranges[num_readings / 2]; // update distance front
    _rear_dist_curr  = ranges[num_readings - 1]; // update distance back

    int quarter_index = num_readings / 4;
    _left_dist_curr   = ranges[num_readings - quarter_index]; // update distance left
    _right_dist_curr  = ranges[quarter_index];               // update distance right
}


// 获取小车的动作执行结果
void NavShoot::action_result_callback(const MoveBaseActionResult::ConstPtr& msg) {
    // 未处于巡航任务时, 不对后续的信息进行处理
    if (!_pose_task) { return; }

    // 处于巡航任务时, 检测动作执行结果
    if (msg->status.status != 3) {
        return;
    }

    // 任务目标已经达成
    _pose_task       = nullptr;
    _curr_task->done = true;
}


// 获取AR码识别结果
void NavShoot::marker_tag_callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {
    // 不处于射击任务时, 不进行后续信息处理
    _marker_found = false;
    if (_shoot_task == nullptr) { return; }

    int count = msg->markers.size();

    for (int i = 0; i < count; i++) {
        if (msg->markers[i].id == _shoot_task->tag_id) {
            _marker_found = true;

            _offset_x = msg->markers[i].pose.pose.position.x + _shoot_task->yaw_err;
            _vel_x    = -_offset_x;
            return;  // 找到目标靶, 直接返回, 不再检测后续的靶号, 假设没有重复码的情况下
        }
    }
}



void NavShoot::shoot_tag() {
    if (_marker_found) {
        // 如果炮筒的偏航角指定范围内, 则进行射击 | 这个yaw_th容易受目标与小车的距离的大小影响
        if (-_shoot_task->yaw_th < _offset_x && _offset_x < _shoot_task->yaw_th) {
            // ros::Duration(0.5).sleep();
            ROS_INFO("shoot to ar_marker: %d", _shoot_task->tag_id);

            // 发布一次射击命令
            std_msgs::String cmd_shoot;
            cmd_shoot.data = "1";
            _shoot_pub.publish(cmd_shoot);
            ros::Duration(0.5).sleep();
            
            _shoot_pub.publish(cmd_shoot);
            ros::Duration(0.5).sleep();

            _shoot_pub.publish(cmd_shoot);
            ros::Duration(0.5).sleep();

            // TODO: 射击完之后, 能过检测_marker_found来查看目标靶号是否已经拿下
            // TODO: 目标靶号没有倒下, 可以通过设置yaw_err值来重新进行射击
            // 前提是瞄准的、唯一，没有重复
            // while (true) {
            //     ros::spinOnce();
            //     if (_marker_found) {
            //         _shoot_pub.publish(cmd_shoot);
            //         ros::Duration(0.5).sleep();
            //     } else {
            //         break;
            //     }
            // }
            // 任务目标已经达成
            _shoot_task      = nullptr;
            _curr_task->done = true;
            return;  // 任务完成, 不需要后续操作, 直接返回
        }

        // 如果小车的大体方向没有指向目标靶方向, 直接
        if (abs(_yaw_curr) > 0.5) {
            Twist cmd_vel;
            cmd_vel.angular.z = -_shoot_task->yaw_th * 0.5;
            _cmd_vel_pub.publish(cmd_vel);
        }
        // 将小车的炮台瞄准转到AR码中心上
        Twist cmd_vel;
        // if (_vel_x < 0.03) {
        //     _vel_x = 0.05;
        // } else if (_vel_x > 0.10) {
        //     _vel_x = 0.07;
        // }
        cmd_vel.angular.z = _vel_x;
        _cmd_vel_pub.publish(cmd_vel);
    }
}
