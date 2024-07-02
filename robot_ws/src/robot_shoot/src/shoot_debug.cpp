#include <iostream>
#include <string>

#include <ros/package.h>
#include <ros/ros.h>

#include "nav_shoot/nav_shoot.hpp"
#include "nav_shoot/robot_task.hpp"

#include "yaml-cpp/yaml.h"

int main(int argc, char** argv) {
    // 设置ros输出中文
    setlocale(LC_CTYPE, "zh_CN.utf8");

    std::string package_path   = ros::package::getPath("robot_shoot");
    YAML::Node cfg_node        = YAML::LoadFile(package_path + "/config/config.yaml");
    std::string task_file      = cfg_node["task"].as<std::string>();
    std::string task_config_fp = package_path + "/tasks/" + task_file;  // 加载指定任务配置文件的路径

    ros::init(argc, argv, "nav_shoot");

    ROS_INFO_ONCE("任务设置文件: %s", task_config_fp.c_str());
    // 加载任务配置
    TaskQueue task_queue(task_config_fp);
    if (task_queue.empty()) {
        ROS_INFO_ONCE("任务队列为空, 退出程序, 请配置任务队列");
        ros::shutdown();
        return 0;
    }

    // 实例结点
    NavShoot nav_shoot(task_queue);

    std::cout << "[Enter] 开始运行";
    std::cin.ignore();

    std::cout << std::endl;

    nav_shoot.shoot_debug();
    ros::spin();    

    return 0;
}
