#pragma once
#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <memory>
#include <string>
namespace wheel_leg_demo {
class WheelLocal;
class FootLocal;
class TaskRunner {
public:
  TaskRunner(rclcpp::Node*,WheelLocal*,FootLocal*,std::function<bool()> planGlobal,std::function<std::string()> globalReason);
  ~TaskRunner();
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
