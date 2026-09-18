#pragma once

#include <Eigen/Dense>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace mocap_subscriber {

struct Pose {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
    int32_t rigid_body_id{-1};
    uint32_t frame_number{0};
    std::chrono::steady_clock::time_point received_at{};
};

struct NatNetConfig {
    std::string server_address{"192.168.0.26"};
    std::string local_address{"0.0.0.0"};
    std::string multicast_address{"239.255.42.99"};
    uint16_t command_port{1510};
    uint16_t data_port{1511};
    std::vector<int32_t> rigid_body_ids{4};
    std::vector<std::string> rigid_body_names{};
    bool use_multicast{false};
    int protocol_major{3};
};

struct NatNetStats {
    uint64_t packets_received{0};
    uint64_t frames_received{0};
    uint64_t poses_matched{0};
    uint32_t last_frame_number{0};
    int32_t last_rigid_body_count{0};
    int32_t last_skeleton_rigid_body_count{0};
    std::array<int32_t, 16> last_seen_rigid_body_ids{};
    size_t last_seen_rigid_body_id_count{0};
};

NatNetConfig loadNatNetConfig(const std::string& path);

class NatNetClient {
public:
    using PoseCallback = std::function<void(const Pose&)>;

    explicit NatNetClient(NatNetConfig config = {});
    ~NatNetClient();

    NatNetClient(const NatNetClient&) = delete;
    NatNetClient& operator=(const NatNetClient&) = delete;

    bool start();
    void stop();

    [[nodiscard]] bool isRunning() const { return running_.load(); }
    [[nodiscard]] bool hasPose() const { return has_pose_.load(); }
    [[nodiscard]] std::optional<Pose> latestPose() const;
    [[nodiscard]] std::optional<Pose> latestPoseById(int32_t rigid_body_id) const;
    [[nodiscard]] std::vector<Pose> latestPoses() const;
    [[nodiscard]] bool isPoseFresh(int32_t rigid_body_id, std::chrono::milliseconds max_age) const;
    [[nodiscard]] NatNetStats stats() const;
    [[nodiscard]] const NatNetConfig& config() const { return config_; }

    void setPoseCallback(PoseCallback callback);

private:
    void receiveLoop();
    bool handshakeUdp();
    bool parseFrameOfData(const uint8_t* payload, size_t len);
    bool wantsRigidBody(int32_t rigid_body_id) const;
    void publishPoses(const std::vector<Pose>& poses);

    NatNetConfig config_;
    int data_socket_{-1};
    int command_socket_{-1};
    std::atomic<bool> running_{false};
    std::thread receive_thread_;

    mutable std::mutex pose_mutex_;
    std::optional<Pose> latest_pose_;
    std::unordered_map<int32_t, Pose> latest_poses_;
    PoseCallback pose_callback_;
    std::atomic<bool> has_pose_{false};

    mutable std::mutex stats_mutex_;
    NatNetStats stats_;

    uint8_t natnet_major_{3};
    uint8_t natnet_minor_{0};
};

}  // namespace mocap_subscriber
