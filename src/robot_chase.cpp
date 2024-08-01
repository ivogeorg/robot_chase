#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class RobotChase : public rclcpp::Node {
private:
  std::string mover_robot_name_;
  std::string chaser_robot_name_;
  std::string base_frame_name_;
  double base_radius_;

  rclcpp::TimerBase::SharedPtr timer_; //{nullptr};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
      publisher_; //{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_; //{nullptr};

public:
  RobotChase(std::string mover_robot_name, std::string chaser_robot_name,
             std::string base_frame_name, double base_radius)
      : Node("robot_chase_node"), mover_robot_name_{mover_robot_name},
        chaser_robot_name_{chaser_robot_name},
        base_frame_name_{base_frame_name},
        base_radius_(base_radius), timer_{this->create_wall_timer(
            100ms, std::bind(&RobotChase::tf_timer, this))},
        publisher_{this->create_publisher<geometry_msgs::msg::Twist>(
            chaser_robot_name_ + "/cmd_vel", 1)},
        tf_buffer_{std::make_unique<tf2_ros::Buffer>(this->get_clock())},
        tf_listener_{
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)} {}

  ~RobotChase() = default;

private:
  void tf_timer() {
    std::string from_frame = mover_robot_name_ + "/" + base_frame_name_;
    std::string to_frame = chaser_robot_name_ + "/" + base_frame_name_;

    geometry_msgs::msg::TransformStamped t;

    // Look up for the transform between the mover robot frame
    // and the chaser robot frame and send velocity commands
    // for the chaser to reach the mover
    try {
      t = tf_buffer_->lookupTransform(to_frame, from_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
                  to_frame.c_str(), from_frame.c_str(), ex.what());

      return;
    }

    double error_distance = sqrt(pow(t.transform.translation.x, 2) +
                                 pow(t.transform.translation.y, 2));

    double error_yaw =
        atan2(t.transform.translation.y, t.transform.translation.x);

    geometry_msgs::msg::Twist msg;

    static const double kp_yaw = 0.7;
    msg.angular.z = kp_yaw * error_yaw;

    static const double kp_distance = 0.3;
    msg.linear.x = kp_distance * error_distance;

    if (error_distance < 2 * base_radius_ + 0.05)
      msg.angular.z = msg.linear.x = 0.0;

    publisher_->publish(msg);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // TODO: could have a service
  // `barista_robot_names` to get
  // names, roles, base frame
  // and base radius
  rclcpp::spin(
      std::make_shared<RobotChase>("morty", "rick", "base_footprint", 0.178));

  rclcpp::shutdown();

  return 0;
}