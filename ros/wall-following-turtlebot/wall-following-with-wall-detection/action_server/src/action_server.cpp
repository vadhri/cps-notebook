#include <chrono>
#include <functional>
#include <memory>
#include <thread>

#include "geometry_msgs/msg/detail/point32__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "action_msg/action/odom_record.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"
#include <geometry_msgs/msg/point32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <iostream>
using namespace std;

class ActionServer : public rclcpp::Node {
public:
  using Distance = action_msg::action::OdomRecord;
  using GoalHandleDistance = rclcpp_action::ServerGoalHandle<Distance>;

  explicit ActionServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("action_server_node", options) {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Distance>(
        this, "record_odom",
        std::bind(&ActionServer::handle_goal, this, _1, _2),
        std::bind(&ActionServer::handle_cancel, this, _1),
        std::bind(&ActionServer::handle_accepted, this, _1));

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    total_distance_pub_ =
        this->create_publisher<std_msgs::msg::Float64>("/total_distance", 10);

    // Subscribe to Odometry Topic
    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&ActionServer::odometry_callback, this,
                  std::placeholders::_1));
  }

private:
  rclcpp_action::Server<Distance>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  double last_x_ = 0.0, last_y_ = 0.0, total_distance_ = 0.0;
  double start_x_ = 0.0, start_y_ = 0.0, lap_ = 0;
  std::vector<geometry_msgs::msg::Point32> list_of_odoms_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr total_distance_pub_;

  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double current_x = msg->pose.pose.position.x;
    double current_y = msg->pose.pose.position.y;

    if (last_x_ == 0.0 && last_y_ == 0.0) {
      last_x_ = current_x;
      last_y_ = current_y;
      return;
    }

    double delta_x = current_x - last_x_;
    double delta_y = current_y - last_y_;

    double origin_delta_x = current_x - start_x_;
    double origin_delta_y = current_y - start_y_;

    double delta_distance = std::sqrt(delta_x * delta_x + delta_y * delta_y);
    double origin_delta_distance = std::sqrt(origin_delta_x * origin_delta_x +
                                             origin_delta_y * origin_delta_y);

    total_distance_ += delta_distance;

    cout << "origin_delta_distance = " << origin_delta_distance
         << " total_distance_ = " << total_distance_ << endl;

    if (origin_delta_distance < 0.1 && total_distance_ > 3) {
      cout << "Lap ... -----" << current_x << " --- " << current_y << endl;
      lap_ += 1;
    }

    last_x_ = current_x;
    last_y_ = current_y;
    // Get the quaternion from the message
    geometry_msgs::msg::Quaternion quaternion = msg->pose.pose.orientation;

    // Convert quaternion to Euler angles using tf2
    tf2::Quaternion q(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, theta;    // theta corresponds to yaw
    m.getRPY(roll, pitch, theta); // Extract roll, pitch, and yaw (theta)

    // Create a Point32 message
    geometry_msgs::msg::Point32 point;
    point.x = current_x;
    point.y = current_y;
    point.z = theta;
    list_of_odoms_.push_back(point);
  }

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const Distance::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal request with seconds");
    (void)uuid;
    start_x_ = last_x_;
    start_y_ = last_y_;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleDistance> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleDistance> goal_handle) {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a
    // new thread
    std::thread{std::bind(&ActionServer::execute, this, _1), goal_handle}
        .detach();
  }

  void execute(const std::shared_ptr<GoalHandleDistance> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Distance::Feedback>();
    auto &message = feedback->current_total;
    message = 0;
    auto result = std::make_shared<Distance::Result>();
    auto move = geometry_msgs::msg::Twist();
    rclcpp::Rate loop_rate(1);

    while (lap_ == 0) {
      // Check if there is a cancel request
      rclcpp::sleep_for(std::chrono::seconds(1));
      if (goal_handle->is_canceling()) {
        result->list_of_odoms = list_of_odoms_;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // Distance robot forward and send feedback
      message = total_distance_;
      //   move.linear.x = 0.3;
      //   publisher_->publish(move);
      std_msgs::msg::Float64 msg;
      msg.data = total_distance_;
      total_distance_pub_->publish(msg);

      goal_handle->publish_feedback(feedback);
      //   RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->list_of_odoms = list_of_odoms_;
      //   move.linear.x = 0.0;
      //   publisher_->publish(move);
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
      rclcpp::shutdown();
    }
  }
}; // class ActionServer

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<ActionServer>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
