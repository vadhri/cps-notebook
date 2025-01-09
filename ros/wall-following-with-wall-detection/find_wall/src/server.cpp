#include "find_wall_msg/srv/find_wall.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/empty.hpp"

#include <chrono>
#include <memory>

#include <cstdlib> // for abs()
#include <iostream>

using namespace std;

using Empty = std_srvs::srv::Empty;
using FindWall = find_wall_msg::srv::FindWall;
using std::placeholders::_1;
using std::placeholders::_2;

class ServerNode : public rclcpp::Node {
public:
  ServerNode() : Node("find_wall") {

    // Create mutually exclusive callback groups
    move_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    scan_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    srv_ = create_service<FindWall>(
        "find_wall", std::bind(&ServerNode::moving_callback, this, _1, _2),
        rclcpp::ServicesQoS().get_rmw_qos_profile(), move_callback_group_);

    // Create subscription with correct callback group
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = scan_callback_group_;

    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&ServerNode::scan_callback, this, std::placeholders::_1),
        sub_options);

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  rclcpp::Service<FindWall>::SharedPtr srv_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;

  rclcpp::CallbackGroup::SharedPtr move_callback_group_;
  rclcpp::CallbackGroup::SharedPtr scan_callback_group_;

  std::vector<float> laser_scan_data_; // Holds the latest laser scan data

  void moving_callback(
      const std::shared_ptr<find_wall_msg::srv::FindWall_Request> request,
      const std::shared_ptr<find_wall_msg::srv::FindWall_Response> response) {
    cout << "Request to move has come .. " << endl;
    // Assuming laser_scan_data is available after the scan callback
    while (laser_scan_data_.empty()) {
      response->wallfound = false;
      cout << "Laser scanner data is empty .. " << endl;
      rclcpp::sleep_for(std::chrono::seconds(1));
    }

    // Step 1: Find the shortest laser ray
    size_t min_index = 0;
    float min_distance = laser_scan_data_[0];
    for (size_t i = 1; i < laser_scan_data_.size(); ++i) {
      if (laser_scan_data_[i] < min_distance) {
        min_distance = laser_scan_data_[i];
        min_index = i;
      }
    }
    cout << "++++ rotate to 0 min_dist = " << min_distance << " -- "
         << min_index << endl;
    // Step 2: Rotate robot until ray 0 is the shortest
    rotate_to_min_ray(0);
    cout << "++++ Move towards wall" << endl;

    // Step 3: Move forward until ray 0 is shorter than 0.3 meters
    move_towards_wall();
    cout << "++++ Rotate to 270" << endl;

    // Step 4: Rotate again until ray 270 is pointing to the wall
    rotate_to_min_ray((270 - 180) * 2);

    // Step 5: Return success message
    response->wallfound = true;
  }

  void scan_callback(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg) {
    // Save the laser scan data for later processing
    laser_scan_data_ = msg->ranges;
  }
  void rotate_to_min_ray(int index) {
    RCLCPP_INFO(this->get_logger(),
                "[rotate_to_min_ray] Start rotation to minimum ray");
    int min_index = 0;
    int middle_index = laser_scan_data_.size() / 2;
    int center = (index + middle_index) % 360;
    float min_distance = laser_scan_data_[0];
    for (int i = 1; i < laser_scan_data_.size(); ++i) {
      if (laser_scan_data_[i] < min_distance) {
        min_distance = laser_scan_data_[i];
        min_index = i;
      }
    }

    while ((min_index < (index - 20)) || (min_index > (index + 20))) {
      cout << "rotate_to_min_ray, min_index = " << min_index << endl;
      float min_distance = laser_scan_data_[index];
      for (size_t i = 1; i < laser_scan_data_.size(); ++i) {
        if (laser_scan_data_[i] < min_distance) {
          min_distance = laser_scan_data_[i];
          min_index = i;
        }
      }

      geometry_msgs::msg::Twist twist_msg;

      twist_msg.angular.z = 0.25;

      publisher_->publish(twist_msg);
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    // Stop rotation after reaching the target
    geometry_msgs::msg::Twist twist_msg;
    twist_msg.angular.z = 0.0;
    publisher_->publish(twist_msg);

    RCLCPP_INFO(this->get_logger(), "Rotation to minimum ray completed.");
  }

  void move_towards_wall() {
    // Keep moving until ray 0 is shorter than 0.3 meters
    while (*std::min_element(laser_scan_data_.begin(),
                             laser_scan_data_.begin() + 10) > 0.3) {

      geometry_msgs::msg::Twist twist_msg;
      twist_msg.linear.x = -0.1; // Move forward at 0.5 m/s
      twist_msg.angular.z = 0;

      cout << "++++ move_towards_wall rotate to 0 = " << laser_scan_data_[0]
           << endl;

      publisher_->publish(twist_msg);

      rclcpp::sleep_for(std::chrono::milliseconds(200)); // Check
      //   periodically
    }
    geometry_msgs::msg::Twist twist_msg;
    // Stop moving once the condition is met
    twist_msg.linear.x = 0.0;
    publisher_->publish(twist_msg);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ServerNode>();

  // Create a MultiThreadedExecutor to execute callbacks in parallel
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  // Spin the executor to handle all callbacks
  executor.spin();

  rclcpp::shutdown();
  return 0;
}