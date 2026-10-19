#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <memory>
#include <string>
namespace wheel_leg_demo {
class FootLocal {
public:
  explicit FootLocal(rclcpp::Node* node);
  ~FootLocal();
  void connectGazebo();
  bool command(const std::string& name, std::string& message);
  bool running() const;
  std::string state() const;
  void reset();
  void setReference(const nav_msgs::msg::Path&,size_t lowerCount,size_t stairCount);
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
