#include "find_wall_msg/srv/detail/find_wall__struct.hpp"
#include "find_wall_msg/srv/find_wall.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

#include <iostream>

using namespace std;

class ScanSubscriber : public rclcpp::Node {
public:
  ScanSubscriber() : Node("scan_subscriber") {
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&ScanSubscriber::scan_callback, this, _1));
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    float max_laser_middle = 32;
    float max_laser_left = 32;
    float max_laser_right = 32;

    cout << "scan_callback " << endl;

    RCLCPP_DEBUG(this->get_logger(), "Scan callback");

    if (msg->ranges.size() > 0) {
      int middle = (int)msg->ranges.size() / 2;

      for (int i = middle - 20; i < middle + 20; i++) {
        if ((float)msg->ranges[i] <= msg->range_max &&
            (float)msg->ranges[i] >= msg->range_min) {
          max_laser_middle = std::min(max_laser_middle, (float)msg->ranges[i]);
        }
      }

      for (int i = 0; i < 20; i++) {
        if ((float)msg->ranges[i] <= msg->range_max &&
            (float)msg->ranges[i] >= msg->range_min) {

          max_laser_left = std::min(max_laser_left, (float)msg->ranges[i]);
        }
      }

      for (int i = msg->ranges.size() - 21; i < msg->ranges.size() - 1; i++) {
        if ((float)msg->ranges[i] <= msg->range_max &&
            (float)msg->ranges[i] >= msg->range_min) {

          max_laser_right = std::min(max_laser_right, (float)msg->ranges[i]);
        }
      }
      auto cmd_msg = geometry_msgs::msg::Twist();
      // Laser readings
      const float front_distance = max_laser_middle;
      const float left_distance = max_laser_left;
      const float right_distance = max_laser_right;

      RCLCPP_INFO(this->get_logger(), "front = %f, left = %f, right = %f ",
                  front_distance, left_distance, right_distance);

      geometry_msgs::msg::Twist cmd;
      float turn_angle = -0.1;
      float velocity = -0.1;

      if (front_distance < 0.5) {
        // Front wall detected, turn left slowly
        cmd.linear.x = velocity; // Slow forward movement
        cmd.angular.z = 0.75;    // Slow turn left
        RCLCPP_INFO(this->get_logger(),
                    "Front obstacle detected: %.2f meters. Turning left.",
                    front_distance);
      } else if (right_distance > 0.3) {
        // Too far from the wall, turn right gently
        cmd.linear.x = velocity;    // Slow forward movement
        cmd.angular.z = turn_angle; // Gentle turn right
        RCLCPP_INFO(this->get_logger(),
                    "Too far from the wall: %.2f meters. Moving forward + "
                    "turning right. .",
                    right_distance);
      } else if (right_distance < 0.2) {
        // Too close to the wall, turn left gently
        cmd.linear.x = velocity;     // Slow forward movement
        cmd.angular.z = -turn_angle; // Gentle turn left
        RCLCPP_INFO(this->get_logger(),
                    "Too close to the wall: %.2f meters. Moving forward + "
                    "turning left.",
                    right_distance);
      } else {
        // Maintain distance, move forward slowly
        cmd.linear.x = velocity; // Slow forward movement
        cmd.angular.z = 0;       // No rotation
        RCLCPP_INFO(this->get_logger(),
                    "Maintaining distance: %.2f meters. Moving forward.",
                    right_distance);
      }

      // Publish the command
      publisher_->publish(cmd);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  bool notReady;
};
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node_service_client =
      rclcpp::Node::make_shared("service_client");

  rclcpp::Client<find_wall_msg::srv::FindWall>::SharedPtr client =
      node_service_client->create_client<find_wall_msg::srv::FindWall>(
          "find_wall");

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "service not available, waiting again...");
  }

  auto request = std::make_shared<find_wall_msg::srv::FindWall::Request>();

  // Send the service request and wait for the response synchronously
  auto future = client->async_send_request(request);
  cout << "Send Async request ... " << endl;

  rclcpp::spin_until_future_complete(node_service_client, future);

  cout << "Future complete ... " << endl;
  std::shared_ptr<rclcpp::Node> node = std::make_shared<ScanSubscriber>();

  // Once the request completes, proceed to the subscriber spin
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
