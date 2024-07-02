#ifndef ROBOT_SHOOT_NAV_UTILS_H
#define ROBOT_SHOOT_NAV_UTILS_H

#include <chrono>
#include <iomanip>
#include <iostream>
#include <string>

// 获取当前时间, 以指定格式返回字符串
std::string get_current_time_str(const std::string& format) {
    auto current_time          = std::chrono::system_clock::now();
    std::time_t current_time_t = std::chrono::system_clock::to_time_t(current_time);
    std::tm local_time         = *std::localtime(&current_time_t);

    std::stringstream ss;
    ss << std::put_time(&local_time, format.c_str());

    return ss.str();
}


// 获取当前时间与指定时间的时间差字符串 精确到 毫秒
std::string get_time_diff_str(const std::chrono::system_clock::time_point& last_time) {
    using namespace std;

    auto current_time = chrono::system_clock::now();
    auto time_d       = current_time - last_time;

    int hours   = chrono::duration_cast<chrono::hours>(time_d).count();
    int minutes = chrono::duration_cast<chrono::minutes>(time_d).count();
    int seconds = chrono::duration_cast<chrono::seconds>(time_d).count();
    int milli   = chrono::duration_cast<chrono::milliseconds>(time_d).count();

    std::stringstream ss;
    if (hours > 0)
        ss << " " << hours << " 小时";
    if (minutes > 0)
        ss << " " << minutes << " 分";
    if (seconds > 0)
        ss << " " << seconds << " 秒";

    ss << " " << milli << " 毫秒";

    return ss.str();  // 只精确到毫秒
}

#endif  //ROBOT_SHOOT_NAV_UTILS_H