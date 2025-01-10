#include "action_msg/action/detail/odom_record__struct.hpp"
#include "action_msg/action/odom_record.hpp"
#include "find_wall_msg/srv/detail/find_wall__struct.hpp"
#include "find_wall_msg/srv/find_wall.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
using std::placeholders::_1;

using namespace std::placeholders;
using namespace std;

#include <iostream>

#include "action_msg/action/odom_record.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class MyActionClient : public rclcpp::Node {
public:
  using Move = action_msg::action::OdomRecord;
  using GoalHandleMove = rclcpp_action::ClientGoalHandle<Move>;

  explicit MyActionClient(
      const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
      : Node("my_action_client", node_options), goal_done_(false) {

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    callback_group_1 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_2 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    this->client_ptr_ = rclcpp_action::create_client<Move>(
        this->get_node_base_interface(), this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(), "record_odom", callback_group_1);

    RCLCPP_INFO(this->get_logger(), "MyActionClient constructor .. ");

    this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&MyActionClient::send_goal, this), callback_group_2);
  }

  bool is_goal_done() const { return this->goal_done_; }

  void send_goal() {
    using namespace std::placeholders;

    this->timer_->cancel();

    this->goal_done_ = false;

    if (!this->client_ptr_) {
      RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    }

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      this->goal_done_ = true;
      return;
    }

    auto goal_msg = Move::Goal();

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Move>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        std::bind(&MyActionClient::goal_response_callback, this, _1);

    send_goal_options.feedback_callback =
        std::bind(&MyActionClient::feedback_callback, this, _1, _2);

    send_goal_options.result_callback =
        std::bind(&MyActionClient::result_callback, this, _1);

    auto goal_handle_future =
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Move>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr callback_group_1;
  rclcpp::CallbackGroup::SharedPtr callback_group_2;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  bool goal_done_;

  void
  goal_response_callback(std::shared_future<GoalHandleMove::SharedPtr> future) {
    auto goal_handle = future.get();

    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(GoalHandleMove::SharedPtr,
                         const std::shared_ptr<const Move::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "Dist: %f", feedback->current_total);
  }

  void result_callback(const GoalHandleMove::WrappedResult &result) {
    this->goal_done_ = true;
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
    }

    RCLCPP_INFO(this->get_logger(),
                "Result received: %d location co-ordinates ",
                result.result->list_of_odoms.size());

    for (auto l : result.result->list_of_odoms) {
      cout << " x = " << l.x << " y = " << l.y << " theta = " << l.z << endl;
    }

    auto cmd_msg = geometry_msgs::msg::Twist();
    cmd_msg.linear.x = 0; // Slow forward movement
    cmd_msg.angular.z = 0;
    publisher_->publish(cmd_msg);
    rclcpp::shutdown();
  }
}; // class MyActionClient

class ScanSubscriber : public rclcpp::Node {
public:
  ScanSubscriber() : Node("scan_subscriber") {
    // Create the callback group
    callback_group_1 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions options1;
    options1.callback_group = callback_group_1;

    // Create the subscription with the callback group
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&ScanSubscriber::scan_callback, this, std::placeholders::_1),
        options1);

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

private:
  rclcpp::CallbackGroup::SharedPtr callback_group_1;

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    float max_laser_middle = 32;
    float max_laser_left = 32;
    float max_laser_right = 32;

    // cout << "scan_callback " << endl;

    // RCLCPP_DEBUG(this->get_logger(), "Scan callback");

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

      //   RCLCPP_INFO(this->get_logger(), "front = %f, left = %f, right = %f ",
      //               front_distance, left_distance, right_distance);

      geometry_msgs::msg::Twist cmd;
      float turn_angle = -0.1;
      float velocity = 0.1; // change sign between simulation and real robot.

      if (front_distance < 0.6) {
        // Front wall detected, turn left slowly
        cmd.linear.x = velocity; // Slow forward movement
        cmd.angular.z = 0.49;    // Slow turn left
        // RCLCPP_INFO(this->get_logger(),
        //             "Front obstacle detected: %.2f meters. Turning left.",
        //             front_distance);
      } else if (right_distance > 0.3) {
        // Too far from the wall, turn right gently
        cmd.linear.x = velocity;    // Slow forward movement
        cmd.angular.z = turn_angle; // Gentle turn right
        // RCLCPP_INFO(this->get_logger(),
        //             "Too far from the wall: %.2f meters. Moving forward + "
        //             "turning right. .",
        //             right_distance);
      } else if (right_distance < 0.2) {
        // Too close to the wall, turn left gently
        cmd.linear.x = velocity;     // Slow forward movement
        cmd.angular.z = -turn_angle; // Gentle turn left
        // RCLCPP_INFO(this->get_logger(),
        //             "Too close to the wall: %.2f meters. Moving forward + "
        //             "turning left.",
        //             right_distance);
      } else {
        // Maintain distance, move forward slowly
        cmd.linear.x = velocity; // Slow forward movement
        cmd.angular.z = 0;       // No rotation
        // RCLCPP_INFO(this->get_logger(),
        //             "Maintaining distance: %.2f meters. Moving forward.",
        //             right_distance);
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
  std::shared_ptr<rclcpp::Node> node_client =
      std::make_shared<MyActionClient>();

  rclcpp::executors::MultiThreadedExecutor executor;

  // Add the action client, service client, and subscriber nodes to the executor
  executor.add_node(node_client);
  executor.add_node(node);

  executor.spin();

  // Once the request completes, proceed to the subscriber spin
  //   rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
