#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "MoCapParser/NatNetClient.hpp"

using namespace std::chrono_literals;

namespace {

geometry_msgs::msg::PoseStamped toPoseStamped(
    const mocap_subscriber::Pose& pose, const std::string& frame_id, rclcpp::Clock& clock) {
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = clock.now();
  msg.header.frame_id = frame_id;
  msg.pose.position.x = pose.position.x();
  msg.pose.position.y = pose.position.y();
  msg.pose.position.z = pose.position.z();
  msg.pose.orientation.w = pose.orientation.w();
  msg.pose.orientation.x = pose.orientation.x();
  msg.pose.orientation.y = pose.orientation.y();
  msg.pose.orientation.z = pose.orientation.z();
  return msg;
}

}  // namespace

class MocapBridgeNode : public rclcpp::Node {
 public:
  MocapBridgeNode() : Node("mocap_bridge_node") {
    declare_parameter<std::string>("frame_id", "mocap");
    declare_parameter<int>("publish_rate_hz", 120);
    declare_parameter<int>("pose_max_age_ms", 250);
    declare_parameter<std::string>("server_address", "192.168.0.189");
    declare_parameter<std::string>("local_address", "0.0.0.0");
    declare_parameter<bool>("use_multicast", false);
    declare_parameter<int>("command_port", 1510);
    declare_parameter<int>("data_port", 1511);
    declare_parameter<std::vector<int64_t>>("rigid_body_ids", {6});
    declare_parameter<std::vector<std::string>>("rigid_body_names", {"umi_gripper"});

    frame_id_ = get_parameter("frame_id").as_string();
    max_age_ = std::chrono::milliseconds(get_parameter("pose_max_age_ms").as_int());

    mocap_subscriber::NatNetConfig config;
    config.server_address = get_parameter("server_address").as_string();
    config.local_address = get_parameter("local_address").as_string();
    config.use_multicast = get_parameter("use_multicast").as_bool();
    config.command_port = static_cast<uint16_t>(get_parameter("command_port").as_int());
    config.data_port = static_cast<uint16_t>(get_parameter("data_port").as_int());
    for (const auto id : get_parameter("rigid_body_ids").as_integer_array()) {
      config.rigid_body_ids.push_back(static_cast<int32_t>(id));
    }
    config.rigid_body_names = get_parameter("rigid_body_names").as_string_array();

    client_ = std::make_unique<mocap_subscriber::NatNetClient>(config);
    if (!client_->start()) {
      RCLCPP_FATAL(get_logger(), "Failed to start NatNet client");
      throw std::runtime_error("NatNet client start failed");
    }

    for (size_t i = 0; i < config.rigid_body_ids.size(); ++i) {
      const int32_t id = config.rigid_body_ids[i];
      const std::string name =
          i < config.rigid_body_names.size() ? config.rigid_body_names[i] : std::to_string(id);
      publishers_[id] = create_publisher<geometry_msgs::msg::PoseStamped>(
          "mocap/" + name + "/pose", rclcpp::SensorDataQoS());
    }

    const auto rate_hz = get_parameter("publish_rate_hz").as_int();
    timer_ = create_wall_timer(
        std::chrono::milliseconds(1000 / std::max<int64_t>(1, rate_hz)),
        std::bind(&MocapBridgeNode::publishPoses, this));

    RCLCPP_INFO(get_logger(), "mocap_bridge_node started, tracking %zu rigid bodies",
                config.rigid_body_ids.size());
  }

 private:
  void publishPoses() {
    for (auto& [id, publisher] : publishers_) {
      if (!client_->isPoseFresh(id, max_age_)) {
        continue;
      }
      const auto pose = client_->latestPoseById(id);
      if (!pose) {
        continue;
      }
      publisher->publish(toPoseStamped(*pose, frame_id_, *get_clock()));
    }
  }

  std::unique_ptr<mocap_subscriber::NatNetClient> client_;
  std::unordered_map<int32_t, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr>
      publishers_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string frame_id_;
  std::chrono::milliseconds max_age_{250};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MocapBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
