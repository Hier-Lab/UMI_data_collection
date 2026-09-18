#include "MoCapParser/NatNetClient.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>

namespace mocap_subscriber {
namespace {

constexpr uint16_t NAT_CONNECT = 0;
constexpr uint16_t NAT_SERVERINFO = 1;
constexpr uint16_t NAT_REQUEST_FRAMEOFDATA = 6;
constexpr uint16_t NAT_FRAMEOFDATA = 7;
constexpr uint16_t NAT_KEEPALIVE = 10;

void writeLe16(uint8_t* p, uint16_t v)
{
    p[0] = static_cast<uint8_t>(v & 0xff);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xff);
}

uint16_t readLe16(const uint8_t* p)
{
    return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

bool toBool(const std::string& value)
{
    return value == "true" || value == "True" || value == "TRUE" || value == "1" || value == "yes" || value == "on";
}

void trim(std::string& s)
{
    const auto start = s.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) {
        s.clear();
        return;
    }
    const auto end = s.find_last_not_of(" \t\r\n");
    s = s.substr(start, end - start + 1);
}

std::vector<int32_t> parseRigidBodyIdList(std::string value)
{
    std::replace(value.begin(), value.end(), ',', ' ');
    std::vector<int32_t> ids;
    std::stringstream ss(value);
    int32_t id = 0;
    while (ss >> id) {
        ids.push_back(id);
    }
    return ids;
}

std::vector<std::string> parseStringList(const std::string& value)
{
    std::vector<std::string> items;
    std::string item;
    for (const char c : value) {
        if (c == '"' || c == '\'') {
            continue;
        }
        if (c == ',') {
            trim(item);
            if (!item.empty()) {
                items.push_back(item);
            }
            item.clear();
            continue;
        }
        item.push_back(c);
    }
    trim(item);
    if (!item.empty()) {
        items.push_back(item);
    }
    return items;
}

std::string joinIds(const std::vector<int32_t>& ids)
{
    std::ostringstream out;
    out << "[";
    for (size_t i = 0; i < ids.size(); ++i) {
        if (i > 0) {
            out << ",";
        }
        out << ids[i];
    }
    out << "]";
    return out.str();
}

void appendUniquePose(std::vector<Pose>& poses, const Pose& pose)
{
    for (auto& existing : poses) {
        if (existing.rigid_body_id == pose.rigid_body_id) {
            existing = pose;
            return;
        }
    }
    poses.push_back(pose);
}

bool setAddress(sockaddr_in& address, const std::string& ip, uint16_t port)
{
    address = {};
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    return inet_pton(AF_INET, ip.c_str(), &address.sin_addr) == 1;
}

bool sendNatUdpRequest(int socket_fd, const sockaddr_in& server, uint16_t command,
                       const void* payload, uint16_t payload_len)
{
    uint8_t packet[4 + 512];
    if (4u + static_cast<size_t>(payload_len) > sizeof(packet)) {
        return false;
    }

    writeLe16(packet, command);
    writeLe16(packet + 2, payload_len);
    if (payload_len > 0 && payload != nullptr) {
        std::memcpy(packet + 4, payload, payload_len);
    }

    const ssize_t sent = sendto(socket_fd, packet, 4 + payload_len, 0, reinterpret_cast<const sockaddr*>(&server), sizeof(server));
    return sent == static_cast<ssize_t>(4 + payload_len);
}

bool sendNatCommand(int socket_fd, const sockaddr_in& server, uint16_t command)
{
    return sendNatUdpRequest(socket_fd, server, command, nullptr, 0);
}

bool sendNatStringCommand(int socket_fd, const sockaddr_in& server, uint16_t command, const char* text)
{
    const size_t text_len = std::strlen(text);
    return sendNatUdpRequest(socket_fd, server, command, text, static_cast<uint16_t>(text_len + 1));
}

bool sendNatKeepalive(int socket_fd, const sockaddr_in& server)
{
    return sendNatCommand(socket_fd, server, NAT_KEEPALIVE);
}

class PacketReader {
public:
    PacketReader(const uint8_t* data, size_t len) : begin_(data), p_(data), end_(data + len) {}

    [[nodiscard]] size_t offset() const { return static_cast<size_t>(p_ - begin_); }

    bool skip(size_t n)
    {
        if (remaining() < n) {
            return false;
        }
        p_ += n;
        return true;
    }

    bool readI32(int32_t& value)
    {
        if (remaining() < sizeof(value)) {
            return false;
        }
        std::memcpy(&value, p_, sizeof(value));
        p_ += sizeof(value);
        return true;
    }

    bool readU32(uint32_t& value)
    {
        if (remaining() < sizeof(value)) {
            return false;
        }
        std::memcpy(&value, p_, sizeof(value));
        p_ += sizeof(value);
        return true;
    }

    bool readF32(float& value)
    {
        if (remaining() < sizeof(value)) {
            return false;
        }
        std::memcpy(&value, p_, sizeof(value));
        p_ += sizeof(value);
        return true;
    }

    bool skipCString()
    {
        while (remaining() > 0) {
            if (*p_++ == 0) {
                return true;
            }
        }
        return false;
    }

private:
    [[nodiscard]] size_t remaining() const { return static_cast<size_t>(end_ - p_); }

    const uint8_t* begin_;
    const uint8_t* p_;
    const uint8_t* end_;
};

bool skipDatasetSizeIfNeeded(PacketReader& reader, bool needs_dataset_size)
{
    return !needs_dataset_size || reader.skip(4);
}

bool skipMarkerSet(PacketReader& reader)
{
    int32_t marker_count = 0;
    if (!reader.skipCString() || !reader.readI32(marker_count)) {
        return false;
    }
    if (marker_count < 0 || marker_count > 1000000) {
        return false;
    }
    return reader.skip(static_cast<size_t>(marker_count) * 12u);
}

void rememberRigidBodyId(NatNetStats& stats, int32_t rigid_body_id)
{
    for (size_t i = 0; i < stats.last_seen_rigid_body_id_count; ++i) {
        if (stats.last_seen_rigid_body_ids[i] == rigid_body_id) {
            return;
        }
    }
    if (stats.last_seen_rigid_body_id_count < stats.last_seen_rigid_body_ids.size()) {
        stats.last_seen_rigid_body_ids[stats.last_seen_rigid_body_id_count++] = rigid_body_id;
    }
}

bool parseRigidBodyPose(PacketReader& reader, int protocol_major, Pose& pose)
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float qx = 0.0f;
    float qy = 0.0f;
    float qz = 0.0f;
    float qw = 1.0f;

    if (!reader.readI32(pose.rigid_body_id) || !reader.readF32(x) || !reader.readF32(y) ||
        !reader.readF32(z) || !reader.readF32(qx) || !reader.readF32(qy) ||
        !reader.readF32(qz) || !reader.readF32(qw)) {
        return false;
    }

    pose.position = Eigen::Vector3d{x, y, z};
    pose.orientation = Eigen::Quaterniond{qw, qx, qy, qz};
    if (pose.orientation.norm() > 0.0) {
        pose.orientation.normalize();
    }

    if (protocol_major >= 3) {
        return reader.skip(4 + 2);
    }

    int32_t marker_count = 0;
    if (!reader.readI32(marker_count) || marker_count < 0 || marker_count > 1000000) {
        return false;
    }
    return reader.skip(static_cast<size_t>(marker_count) * 12u) && reader.skip(static_cast<size_t>(marker_count) * 4u) &&
           reader.skip(static_cast<size_t>(marker_count) * 4u) && reader.skip(4 + 2);
}

}  // namespace

NatNetConfig loadNatNetConfig(const std::string& path)
{
    NatNetConfig config;
    std::ifstream file(path);
    if (!file) {
        return config;
    }

    std::string line;
    while (std::getline(file, line)) {
        const auto comment = line.find('#');
        if (comment != std::string::npos) {
            line = line.substr(0, comment);
        }
        const auto eq = line.find('=');
        if (eq == std::string::npos) {
            continue;
        }

        std::string key = line.substr(0, eq);
        std::string value = line.substr(eq + 1);
        trim(key);
        trim(value);
        if (value.size() >= 2 && value.front() == '"' && value.back() == '"') {
            value = value.substr(1, value.size() - 2);
        }

        if (key == "server_address") {
            config.server_address = value;
        } else if (key == "local_address") {
            config.local_address = value;
        } else if (key == "multicast_address") {
            config.multicast_address = value;
        } else if (key == "command_port") {
            config.command_port = static_cast<uint16_t>(std::stoi(value));
        } else if (key == "data_port") {
            config.data_port = static_cast<uint16_t>(std::stoi(value));
        } else if (key == "rigid_body_id" || key == "rigid_body_ids") {
            config.rigid_body_ids = parseRigidBodyIdList(value);
        } else if (key == "rigid_body_name" || key == "rigid_body_names") {
            config.rigid_body_names = parseStringList(value);
        } else if (key == "use_multicast") {
            config.use_multicast = toBool(value);
        } else if (key == "protocol_major") {
            config.protocol_major = std::stoi(value);
        }
    }

    return config;
}

NatNetClient::NatNetClient(NatNetConfig config) : config_(std::move(config))
{
    if (config_.rigid_body_ids.empty()) {
        config_.rigid_body_ids.push_back(4);
    }
    latest_poses_.reserve(config_.rigid_body_ids.size());
    natnet_major_ = static_cast<uint8_t>(std::clamp(config_.protocol_major, 2, 5));
}

NatNetClient::~NatNetClient()
{
    stop();
}

bool NatNetClient::start()
{
    if (running_.load()) {
        return true;
    }

    auto cleanup = [this]() {
        if (data_socket_ >= 0) {
            close(data_socket_);
            data_socket_ = -1;
        }
        if (command_socket_ >= 0) {
            close(command_socket_);
            command_socket_ = -1;
        }
    };

    data_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (data_socket_ < 0) {
        std::cerr << "[NatNet] data socket failed: " << std::strerror(errno) << '\n';
        return false;
    }

    int reuse = 1;
    if (setsockopt(data_socket_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
        std::cerr << "[NatNet] SO_REUSEADDR(data) failed: " << std::strerror(errno) << '\n';
        cleanup();
        return false;
    }

    sockaddr_in bind_data{};
    bind_data.sin_family = AF_INET;
    bind_data.sin_port = htons(config_.data_port);
    bind_data.sin_addr.s_addr = htonl(INADDR_ANY);

    if (config_.use_multicast && config_.local_address != "0.0.0.0") {
        inet_pton(AF_INET, config_.local_address.c_str(), &bind_data.sin_addr);
    }

    if (bind(data_socket_, reinterpret_cast<sockaddr*>(&bind_data), sizeof(bind_data)) < 0) {
        if (!config_.use_multicast && errno == EADDRINUSE) {
            sockaddr_in bind_any{};
            bind_any.sin_family = AF_INET;
            bind_any.sin_port = 0;
            bind_any.sin_addr.s_addr = htonl(INADDR_ANY);
            if (bind(data_socket_, reinterpret_cast<sockaddr*>(&bind_any), sizeof(bind_any)) < 0) {
                std::cerr << "[NatNet] data bind fallback failed: " << std::strerror(errno) << '\n';
                cleanup();
                return false;
            }
        } else {
            std::cerr << "[NatNet] data bind failed: " << std::strerror(errno) << '\n';
            cleanup();
            return false;
        }
    }

    if (config_.use_multicast) {
        ip_mreq mreq{};
        if (inet_pton(AF_INET, config_.multicast_address.c_str(), &mreq.imr_multiaddr) != 1) {
            std::cerr << "[NatNet] bad multicast_address: " << config_.multicast_address << '\n';
            cleanup();
            return false;
        }
        mreq.imr_interface.s_addr = htonl(INADDR_ANY);
        if (setsockopt(data_socket_, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0) {
            std::cerr << "[NatNet] multicast join failed: " << std::strerror(errno) << '\n';
            cleanup();
            return false;
        }
    }

    command_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (command_socket_ < 0) {
        std::cerr << "[NatNet] command socket failed: " << std::strerror(errno) << '\n';
        cleanup();
        return false;
    }

    if (setsockopt(command_socket_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
        std::cerr << "[NatNet] SO_REUSEADDR(command) failed: " << std::strerror(errno) << '\n';
        cleanup();
        return false;
    }

    sockaddr_in command_local{};
    command_local.sin_family = AF_INET;
    command_local.sin_port = 0;
    if (config_.local_address == "0.0.0.0") {
        command_local.sin_addr.s_addr = htonl(INADDR_ANY);
    } else if (inet_pton(AF_INET, config_.local_address.c_str(), &command_local.sin_addr) != 1) {
        std::cerr << "[NatNet] bad local_address: " << config_.local_address << '\n';
        cleanup();
        return false;
    }

    if (bind(command_socket_, reinterpret_cast<sockaddr*>(&command_local), sizeof(command_local)) < 0) {
        std::cerr << "[NatNet] command bind failed: " << std::strerror(errno) << '\n';
        cleanup();
        return false;
    }

    const bool handshake_ok = handshakeUdp();
    if (!handshake_ok) {
        std::cerr << "[NatNet] UDP server-info handshake failed; continuing to listen for "
                     "FrameOfMocapData packets. If frames arrive, this is only a server-info "
                     "reply issue.\n";
    }

    sockaddr_in server_command{};
    if (setAddress(server_command, config_.server_address, config_.command_port)) {
        sendNatCommand(command_socket_, server_command, NAT_REQUEST_FRAMEOFDATA);
    }

    running_ = true;
    receive_thread_ = std::thread(&NatNetClient::receiveLoop, this);

    std::cout << "[NatNet] started: rigid_body_id=" << joinIds(config_.rigid_body_ids)
              << ", mode=" << (config_.use_multicast ? "multicast" : "unicast") << '\n';
    return true;
}

void NatNetClient::stop()
{
    running_ = false;
    if (data_socket_ >= 0) {
        shutdown(data_socket_, SHUT_RDWR);
    }
    if (command_socket_ >= 0) {
        shutdown(command_socket_, SHUT_RDWR);
    }
    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }
    if (data_socket_ >= 0) {
        close(data_socket_);
        data_socket_ = -1;
    }
    if (command_socket_ >= 0) {
        close(command_socket_);
        command_socket_ = -1;
    }
}

std::optional<Pose> NatNetClient::latestPose() const
{
    std::lock_guard<std::mutex> lock(pose_mutex_);
    const auto it = latest_poses_.find(config_.rigid_body_ids.front());
    if (it != latest_poses_.end()) {
        return it->second;
    }
    return latest_pose_;
}

std::optional<Pose> NatNetClient::latestPoseById(int32_t rigid_body_id) const
{
    std::lock_guard<std::mutex> lock(pose_mutex_);
    const auto it = latest_poses_.find(rigid_body_id);
    if (it == latest_poses_.end()) {
        return std::nullopt;
    }
    return it->second;
}

std::vector<Pose> NatNetClient::latestPoses() const
{
    std::lock_guard<std::mutex> lock(pose_mutex_);
    std::vector<Pose> poses;
    poses.reserve(config_.rigid_body_ids.size());
    for (const auto id : config_.rigid_body_ids) {
        const auto it = latest_poses_.find(id);
        if (it != latest_poses_.end()) {
            poses.push_back(it->second);
        }
    }
    return poses;
}

bool NatNetClient::isPoseFresh(int32_t rigid_body_id, std::chrono::milliseconds max_age) const
{
    std::lock_guard<std::mutex> lock(pose_mutex_);
    const auto it = latest_poses_.find(rigid_body_id);
    if (it == latest_poses_.end()) {
        return false;
    }
    return std::chrono::steady_clock::now() - it->second.received_at <= max_age;
}

NatNetStats NatNetClient::stats() const
{
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return stats_;
}

void NatNetClient::setPoseCallback(PoseCallback callback)
{
    std::lock_guard<std::mutex> lock(pose_mutex_);
    pose_callback_ = std::move(callback);
}

bool NatNetClient::handshakeUdp()
{
    sockaddr_in server_command{};
    if (!setAddress(server_command, config_.server_address, config_.command_port)) {
        return false;
    }
    if (!sendNatCommand(command_socket_, server_command, NAT_CONNECT)) {
        return false;
    }

    uint8_t buffer[2048];
    bool sent_legacy_ping = false;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline) {
        pollfd pfd{command_socket_, POLLIN, 0};
        const int poll_result = poll(&pfd, 1, 100);
        if (poll_result < 0) {
            if (errno == EINTR) {
                continue;
            }
            return false;
        }
        if (poll_result == 0) {
            if (!sent_legacy_ping) {
                sendNatStringCommand(command_socket_, server_command, NAT_CONNECT, "Ping");
                sent_legacy_ping = true;
            }
            continue;
        }

        sockaddr_in from{};
        socklen_t from_len = sizeof(from);
        const ssize_t bytes = recvfrom(command_socket_, buffer, sizeof(buffer), 0, reinterpret_cast<sockaddr*>(&from), &from_len);
        if (bytes < 4) {
            continue;
        }
        const uint16_t message_id = readLe16(buffer);
        if (message_id != NAT_SERVERINFO) {
            std::cerr << "[NatNet] handshake received non-server-info message_id=" << message_id << " bytes=" << bytes << '\n';
            continue;
        }

        if (bytes >= 268) {
            natnet_major_ = buffer[264];
            natnet_minor_ = buffer[265];
            if (natnet_major_ >= 2 && natnet_major_ <= 5) {
                config_.protocol_major = natnet_major_;
            }
        }
        std::cout << "[NatNet] handshake OK, NatNet " << static_cast<int>(natnet_major_) << '.' << static_cast<int>(natnet_minor_) << '\n';
        return true;
    }

    return false;
}

void NatNetClient::receiveLoop()
{
    uint8_t buffer[64 * 1024];
    sockaddr_in server_command{};
    setAddress(server_command, config_.server_address, config_.command_port);
    auto last_keepalive = std::chrono::steady_clock::now();

    while (running_.load()) {
        const auto now = std::chrono::steady_clock::now();
        if (!config_.use_multicast && now - last_keepalive >= std::chrono::milliseconds(100)) {
            sendNatKeepalive(command_socket_, server_command);
            last_keepalive = now;
        }

        pollfd fds[2]{{data_socket_, POLLIN, 0}, {command_socket_, POLLIN, 0}};
        const int poll_result = poll(fds, 2, 100);
        if (poll_result < 0) {
            if (errno == EINTR) {
                continue;
            }
            break;
        }
        if (poll_result == 0) {
            continue;
        }

        for (const auto& fd : fds) {
            if ((fd.revents & POLLIN) == 0) {
                continue;
            }
            const ssize_t bytes = recv(fd.fd, buffer, sizeof(buffer), 0);
            if (bytes < 4) {
                continue;
            }

            {
                std::lock_guard<std::mutex> lock(stats_mutex_);
                ++stats_.packets_received;
            }

            if (readLe16(buffer) != NAT_FRAMEOFDATA) {
                continue;
            }

            const uint16_t payload_len = readLe16(buffer + 2);
            const size_t available = static_cast<size_t>(bytes - 4);
            if (available < payload_len || payload_len < 4) {
                continue;
            }
            parseFrameOfData(buffer + 4, available);
        }
    }
}

bool NatNetClient::parseFrameOfData(const uint8_t* payload, size_t len)
{
    const bool needs_dataset_size = (natnet_major_ > 4) || (natnet_major_ == 4 && natnet_minor_ >= 1);
    const bool has_skeleton_section = (natnet_major_ > 2) || (natnet_major_ == 2 && natnet_minor_ >= 1);

    PacketReader reader(payload, len);
    uint32_t frame_number = 0;
    int32_t count = 0;
    if (!reader.readU32(frame_number) || !reader.readI32(count) ||
        !skipDatasetSizeIfNeeded(reader, needs_dataset_size)) {
        return false;
    }

    if (count < 0 || count > 100000) {
        return false;
    }
    for (int32_t i = 0; i < count; ++i) {
        if (!skipMarkerSet(reader)) {
            return false;
        }
    }

    if (!reader.readI32(count) || !skipDatasetSizeIfNeeded(reader, needs_dataset_size)) {
        return false;
    }
    if (count < 0 || count > 1000000 || !reader.skip(static_cast<size_t>(count) * 12u)) {
        return false;
    }

    if (!reader.readI32(count) || !skipDatasetSizeIfNeeded(reader, needs_dataset_size)) {
        return false;
    }
    if (count < 0 || count > 100000) {
        return false;
    }

    NatNetStats frame_stats_update;
    frame_stats_update.last_rigid_body_count = count;

    std::vector<Pose> matched_poses;
    matched_poses.reserve(config_.rigid_body_ids.size());
    for (int32_t i = 0; i < count; ++i) {
        Pose pose;
        if (!parseRigidBodyPose(reader, config_.protocol_major, pose)) {
            return false;
        }
        pose.frame_number = frame_number;
        rememberRigidBodyId(frame_stats_update, pose.rigid_body_id);
        if (wantsRigidBody(pose.rigid_body_id)) {
            appendUniquePose(matched_poses, pose);
        }
    }

    if (has_skeleton_section) {
        if (!reader.readI32(count) || !skipDatasetSizeIfNeeded(reader, needs_dataset_size)) {
            return false;
        }
        if (count < 0 || count > 10000) {
            return false;
        }

        for (int32_t skeleton = 0; skeleton < count; ++skeleton) {
            int32_t rigid_body_count = 0;
            if (!reader.skip(4) || !reader.readI32(rigid_body_count)) {
                return false;
            }
            if (rigid_body_count < 0 || rigid_body_count > 10000) {
                return false;
            }
            frame_stats_update.last_skeleton_rigid_body_count += rigid_body_count;
            for (int32_t i = 0; i < rigid_body_count; ++i) {
                Pose pose;
                if (!parseRigidBodyPose(reader, config_.protocol_major, pose)) {
                    return false;
                }
                pose.frame_number = frame_number;
                rememberRigidBodyId(frame_stats_update, pose.rigid_body_id);
                if (wantsRigidBody(pose.rigid_body_id)) {
                    appendUniquePose(matched_poses, pose);
                }
            }
        }
    }

    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        ++stats_.frames_received;
        stats_.last_frame_number = frame_number;
        stats_.last_rigid_body_count = frame_stats_update.last_rigid_body_count;
        stats_.last_skeleton_rigid_body_count = frame_stats_update.last_skeleton_rigid_body_count;
        stats_.last_seen_rigid_body_ids = frame_stats_update.last_seen_rigid_body_ids;
        stats_.last_seen_rigid_body_id_count = frame_stats_update.last_seen_rigid_body_id_count;
    }

    if (matched_poses.empty()) {
        return false;
    }

    const auto received_at = std::chrono::steady_clock::now();
    for (auto& pose : matched_poses) {
        pose.received_at = received_at;
    }
    publishPoses(matched_poses);
    return true;
}

bool NatNetClient::wantsRigidBody(int32_t rigid_body_id) const
{
    return std::find(config_.rigid_body_ids.begin(), config_.rigid_body_ids.end(), rigid_body_id) != config_.rigid_body_ids.end();
}

void NatNetClient::publishPoses(const std::vector<Pose>& poses)
{
    PoseCallback callback;
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        for (const auto& pose : poses) {
            latest_pose_ = pose;
            latest_poses_[pose.rigid_body_id] = pose;
        }
        callback = pose_callback_;
    }
    has_pose_ = true;

    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_.poses_matched += poses.size();
    }

    if (callback) {
        for (const auto& pose : poses) {
            callback(pose);
        }
    }
}

}  // namespace mocap_subscriber
