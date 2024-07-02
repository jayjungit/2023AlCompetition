#ifndef ROBOT_SHOOT_NAV_TASK_H
#define ROBOT_SHOOT_NAV_TASK_H

#include <fstream>
#include <iostream>
#include <memory>
#include <queue>
#include <string>

#include "yaml-cpp/yaml.h"


// 任务类型 --------------------------------------------------------------------------------------------------------------------
enum TaskType {
    POSE_TASK    = 0,  // 自主导航任务
    CORRECT_TASK = 1,  // 纠正任务
    SHOOT_TASK   = 2,  // 目标射击任务
};


// 任务基础类 -------------------------------------------------------------------------------------------------------------------
class BaseTask {
public:
    BaseTask() : done(false), describe("") {};
    virtual ~BaseTask() {};

    TaskType type;         // 任务类型标识
    bool done;             // 任务完成标识
    std::string describe;  // 任务描述说明
};


// 自主导航任务 ------------------------------------------------------------------------------------------------------------------
class PoseTask : public BaseTask {
public:
    PoseTask(double x, double y, int yaw_degrees = 90) {
        this->x           = x;
        this->y           = y;
        this->yaw_degrees = yaw_degrees;
    };

    ~PoseTask() {};

    friend std::ostream& operator<<(std::ostream& os, const PoseTask& pt) {
        os << "| PoseTask |--------------------------------\n";
        os << "|           x: " << std::fixed << std::setprecision(3) << pt.x << "\n";
        os << "|           y: " << std::fixed << std::setprecision(3) << pt.y << "\n";
        os << "| yaw_degrees: " << pt.yaw_degrees << "\n";
        os << "|-------------------------------------------\n";
        return os;
    }

    double x;
    double y;
    int yaw_degrees;
};


// 纠正任务 --------------------------------------------------------------------------------------------------------------------
class CorrectTask : public BaseTask {
public:
    CorrectTask(double vel, double rear, double right, double tol_err_rear = 0.05, double tol_err_right = 0.05) {
        this->vel       = vel;  // 纠正速度
        this->rear      = rear;
        this->right     = right;
        this->rear_min  = rear - tol_err_rear;    // 后边最小距离
        this->rear_max  = rear + tol_err_right;   // 后边最大距离
        this->right_min = right - tol_err_right;  // 右边最小距离
        this->right_max = right + tol_err_right;  // 右边最大距离
    };

    ~CorrectTask() {};

    friend std::ostream& operator<<(std::ostream& os, const CorrectTask& ct) {
        os << "| CorrectTask |-----------------------------\n";
        os << "|          right: " << std::fixed << std::setprecision(3) << ct.vel << "\n";
        os << "|          right: " << std::fixed << std::setprecision(3) << ct.right << "\n";
        os << "|           rear: " << std::fixed << std::setprecision(3) << ct.rear << "\n";
        os << "|  tol_err_right: " << std::fixed << std::setprecision(3) << ct.right_max - ct.right << "\n";
        os << "|   tol_err_rear: " << std::fixed << std::setprecision(3) << ct.rear_max - ct.rear << "\n";
        os << "|-------------------------------------------\n";

        return os;
    }

    double rear, right;
    double vel;
    double rear_min, rear_max;
    double right_min, right_max;
};


// 目标射击任务 ------------------------------------------------------------------------------------------------------------------
class ShootTask : public BaseTask {
public:
    ShootTask(int tag_id, double yaw_th, double yaw_err) {
        this->tag_id  = tag_id;
        this->yaw_th  = yaw_th;
        this->yaw_err = yaw_err;
    };

    ~ShootTask() {};

    friend std::ostream& operator<<(std::ostream& os, const ShootTask& st) {
        os << "| ShootTask |-------------------------------\n";
        os << "|  tag_id: " << st.tag_id << "\n";
        os << "|  yaw_th: " << std::fixed << std::setprecision(3) << st.yaw_th << "\n";
        os << "| yaw_err: " << std::fixed << std::setprecision(3) << st.yaw_err << "\n";
        os << "|-------------------------------------------\n";
        return os;
    }

    int tag_id;
    double yaw_th;
    double yaw_err;
};


// 任务队列 --------------------------------------------------------------------------------------------------------------------
class TaskQueue {
public:
    TaskQueue(std::string task_config_path) {
        load_tasks(task_config_path);
    }

    TaskQueue() {}

    ~TaskQueue() {
        while (!_task_queue.empty()) {
            _task_queue.pop();
        }
    }

    void load_tasks(std::string task_config_path) {
        if (!std::fstream(task_config_path, std::ios::in).is_open()) {
            std::cerr << "任务配置文件不存在" << std::endl;
            return;
        }

        try {
            _init_task_queue(YAML::LoadFile(task_config_path));
        } catch (const YAML::Exception& e) {
            std::cerr << "YAML Exception: " << e.what() << std::endl;
        }
    }

    std::shared_ptr<BaseTask> front() {
        if (_task_queue.empty()) {
            return nullptr;
        }
        return _task_queue.front();
    }

    size_t size() { _task_queue.size(); }

    void pop() { _task_queue.pop(); }

    bool empty() { return _task_queue.empty(); }

private:
    void _init_task_queue(const YAML::Node& tasks) {
        std::string task_type;  // 任务类别
        std::string describe;   // 任务描述

        std::shared_ptr<BaseTask> temp = nullptr;

        for (const auto& task: tasks) {
            describe  = task["describe"].as<std::string>();
            task_type = task["type"].as<std::string>();

            if (task_type == "pose_task") {  // ADD POSE_TASK |=============================================================
                double x        = task["x"].as<double>();
                double y        = task["y"].as<double>();
                int yaw_degrees = task["yaw_degrees"].as<int>();

                temp       = std::make_shared<PoseTask>(x, y, yaw_degrees);
                temp->type = TaskType::POSE_TASK;

                std::cout << "添加任务: [自主巡航]" << std::endl;

            } else if (task_type == "correct_task") {  // ADD CORRECT_TASK |================================================
                double vel                   = task["vel"].as<double>();
                double right                 = task["right"].as<double>();
                double rear                  = task["rear"].as<double>();
                double tolerance_error_right = task["tolerance_error_right"].as<double>();
                double tolerance_error_rear  = task["tolerance_error_rear"].as<double>();

                temp       = std::make_shared<CorrectTask>(vel, rear, right, tolerance_error_rear, tolerance_error_right);
                temp->type = TaskType::CORRECT_TASK;

                std::cout << "添加任务: [位置纠正]" << std::endl;

            } else if (task_type == "shoot_task") {  // ADD SHOTT_TASK |====================================================
                int tag_id     = task["tag_id"].as<int>();
                double yaw_th  = task["yaw_th"].as<double>();
                double yaw_err = task["yaw_err"].as<double>();

                temp       = std::make_shared<ShootTask>(tag_id, yaw_th, yaw_err);
                temp->type = TaskType::SHOOT_TASK;

                std::cout << "添加任务: [目标射击]" << std::endl;

            } else {
                continue;
            }

            temp->describe = describe;  // 任务描述
            _task_queue.push(temp);     // 添加到任务队列中
        }
    }


    std::queue<std::shared_ptr<BaseTask>> _task_queue;
};

#endif  //ROBOT_SHOOT_NAV_TASK_H