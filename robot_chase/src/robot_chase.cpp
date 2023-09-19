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
public:
  RobotChase() : Node("robot_chase") {
    // Declare and acquire `target_frame` parameter
    target_frame_ =
        this->declare_parameter<std::string>("target_frame", "morty/base_link");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("rick/cmd_vel", 1);

    // Call on_timer function every second
    timer_ =
        this->create_wall_timer(1s, std::bind(&RobotChase::on_timer, this));
  }

private:
  void on_timer() {
    // Store frame names in variables that will be used to
    // compute transformations
    std::string fromFrameRel = "morty/base_link";
    std::string toFrameRel = "rick/base_link";

    geometry_msgs::msg::TransformStamped t;

    // Look up for the transformation between target_frame and turtle2
    try {
      t = tf_buffer_->lookupTransform(fromFrameRel, toFrameRel,
                                      tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
                  fromFrameRel.c_str(), toFrameRel.c_str(), ex.what());
      return;
    }

    // Calculate distance error
    double distance_error = sqrt(pow(t.transform.translation.x, 2) +
                                 pow(t.transform.translation.y, 2));

    // Calculate angular error (relative heading)
    double angular_error =
        atan2(t.transform.translation.y, t.transform.translation.x);

    // Create a Twist message to send linear and angular velocities
    geometry_msgs::msg::Twist msg;

    // Define proportional gains for control
    double linear_gain = 0.5;
    double angular_gain = 1.0;

    // Calculate linear and angular velocities based on errors and gains
    msg.linear.x = linear_gain * distance_error;
    msg.angular.z = angular_gain * angular_error;

    // Publish the Twist message as a velocity command
    publisher_->publish(msg);
  }

  // Boolean values to store the information
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string target_frame_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotChase>());
  rclcpp::shutdown();
  return 0;
}
