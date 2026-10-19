#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <memory>
#include <string>
namespace wheel_leg_demo {
class WheelLocal {
public:
  explicit WheelLocal(rclcpp::Node* node);
  ~WheelLocal();
  bool command(const std::string& name, std::string& message);
  bool running() const;
  std::string state() const;
  void reset();
  bool obstaclesCleared() const;
  void setReference(const nav_msgs::msg::Path& path,size_t lowerCount,size_t stairCount);
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
