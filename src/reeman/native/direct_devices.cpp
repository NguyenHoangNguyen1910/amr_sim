#include <gz/msgs/image.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/laserscan.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>

namespace reeman {
namespace {
constexpr int kDepthWidth = 320;
constexpr int kDepthHeight = 160;
constexpr int kHinsPort = 18090;
constexpr int kBeamsPerSector = 1600;
constexpr int kSectorBytes = 16 + kBeamsPerSector * 4;
constexpr double kDegreesToRadians = 0.017453292519943295;

class Socket {
 public:
  Socket() = default;
  Socket(const Socket &) = delete;
  Socket &operator=(const Socket &) = delete;
  ~Socket() { reset(); }
  int fd() const { return fd_; }
  void reset(int replacement = -1) {
    if (fd_ >= 0) close(fd_);
    fd_ = replacement;
  }

 private:
  int fd_ = -1;
};

sockaddr_un UnixAddress(const std::string &path) {
  sockaddr_un address{};
  address.sun_family = AF_UNIX;
  if (path.size() >= sizeof(address.sun_path)) {
    throw std::runtime_error("Direct device socket path is too long: " + path);
  }
  std::memcpy(address.sun_path, path.c_str(), path.size() + 1);
  return address;
}

bool Datagram(const Socket &socket, const sockaddr_un &destination,
              const void *data, std::size_t length) {
  return sendto(socket.fd(), data, length, MSG_DONTWAIT | MSG_NOSIGNAL,
                reinterpret_cast<const sockaddr *>(&destination),
                sizeof(destination)) == static_cast<ssize_t>(length);
}

void Store16(std::uint8_t *out, std::uint16_t value, bool big_endian) {
  out[big_endian ? 1 : 0] = static_cast<std::uint8_t>(value);
  out[big_endian ? 0 : 1] = static_cast<std::uint8_t>(value >> 8);
}

std::uint16_t Millimeters(double distance) {
  if (!std::isfinite(distance) || distance < 0 || distance > 50) return 60000;
  return static_cast<std::uint16_t>(std::nearbyint(distance * 1000));
}

void EncodeSector(std::uint8_t *out, const gz::msgs::LaserScan &scan,
                  int start, int end, std::uint64_t nanoseconds) {
  std::memcpy(out, "HISN", 4);
  Store16(out + 4, start, true);
  Store16(out + 6, end, true);
  for (int offset : {8, 10, 12}) Store16(out + offset, kBeamsPerSector, true);
  Store16(out + 14, (nanoseconds / 1000000 + start) & 0xffff, true);
  // The original ROS LaserScan intermediate stores these fields as float32.
  const double minimum = static_cast<float>(scan.angle_min());
  const double step = static_cast<float>(scan.angle_step());
  for (int index = 0; index < kBeamsPerSector; ++index) {
    const double angle = (start - 180 + index / 10.0) * kDegreesToRadians;
    const double beam = std::nearbyint((angle - minimum) / step);
    const auto range = beam >= 0 && beam < scan.ranges_size()
                           ? Millimeters(static_cast<float>(scan.ranges(static_cast<int>(beam))))
                           : 60000;
    Store16(out + 16 + 4 * index, range, false);
  }
}
}  // namespace

class DirectDevices : public gz::sim::System, public gz::sim::ISystemConfigure {
 public:
  void Configure(const gz::sim::Entity &, const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &, gz::sim::EventManager &) override {
    const std::string state = sdf->Get<std::string>("state_directory");
    const bool depth = sdf->Get<bool>("depth");
    const bool imu = sdf->Get<bool>("imu");
    const bool lidar = sdf->Get<bool>("lidar");
    datagram_.reset(socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0));
    if (datagram_.fd() < 0) throw std::runtime_error("Could not open direct device socket");
    if (depth) {
      depth_address_ = UnixAddress(state + "/openni-depth.sock");
      Subscribe("/sim/rgbd/depth_image", &DirectDevices::Depth);
    }
    if (imu) {
      imu_address_ = UnixAddress(state + "/wit901-device.sock");
      Subscribe("/sim/imu", &DirectDevices::Imu);
    }
    if (lidar) {
      timing_address_ = UnixAddress(state + "/hins-timing.sock");
      StartLidarServer();
      Subscribe("/sim/scan", &DirectDevices::Lidar);
    }
    std::cout << "Reeman direct device plugin active: depth=" << depth
              << " lidar=" << lidar << " imu=" << imu << '\n';
  }

 private:
  template <typename Message>
  void Subscribe(const std::string &topic, void (DirectDevices::*callback)(const Message &)) {
    if (!node_.Subscribe(topic, callback, this))
      throw std::runtime_error("Could not subscribe to " + topic);
  }

  void StartLidarServer() {
    server_.reset(socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0));
    int reuse = 1;
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(kHinsPort);
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    if (server_.fd() < 0 ||
        setsockopt(server_.fd(), SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0 ||
        bind(server_.fd(), reinterpret_cast<const sockaddr *>(&address), sizeof(address)) < 0 ||
        listen(server_.fd(), 1) < 0) {
      throw std::runtime_error("Could not start the virtual HINS TCP listener");
    }
  }

  void Depth(const gz::msgs::Image &image) {
    const unsigned step = image.step();
    if (image.pixel_format_type() != gz::msgs::R_FLOAT32 ||
        image.width() != kDepthWidth || image.height() != kDepthHeight ||
        step < kDepthWidth * sizeof(float) ||
        image.data().size() != static_cast<std::size_t>(step) * kDepthHeight) return;

    struct {
      std::uint64_t microseconds;
      std::array<std::uint16_t, kDepthWidth * kDepthHeight> pixels;
    } frame;
    const auto &stamp = image.header().stamp();
    frame.microseconds = stamp.sec() * 1000000ULL + stamp.nsec() / 1000;
    for (int row = 0; row < kDepthHeight; ++row) {
      const char *source = image.data().data() + row * step;
      for (int column = 0; column < kDepthWidth; ++column) {
        float meters;
        std::memcpy(&meters, source + column * sizeof(float), sizeof(float));
        frame.pixels[row * kDepthWidth + column] =
            meters > 0 && meters <= 65.535f
                ? static_cast<std::uint16_t>(std::nearbyint(meters * 1000.0f))
                : 0;
      }
    }
    Datagram(datagram_, depth_address_, &frame, sizeof(frame));
  }

  void Imu(const gz::msgs::IMU &sample) {
    const auto &q = sample.orientation();
    const auto &angular = sample.angular_velocity();
    const auto &linear = sample.linear_acceleration();
    const std::array<double, 10> values = {q.x(), q.y(), q.z(), q.w(),
        angular.x(), angular.y(), angular.z(), linear.x(), linear.y(), linear.z()};
    Datagram(datagram_, imu_address_, values.data(), sizeof(values));
  }

  void Lidar(const gz::msgs::LaserScan &scan) {
    if (!std::isfinite(scan.angle_step()) || scan.angle_step() <= 0) return;
    if (client_.fd() < 0) {
      client_.reset(accept4(server_.fd(), nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC));
      if (client_.fd() < 0) return;
    }
    const auto &stamp = scan.header().stamp();
    const std::uint64_t nanoseconds = stamp.sec() * 1000000000ULL + stamp.nsec();
    const std::array<std::uint64_t, 2> timing = {nanoseconds, token_};
    if (!Datagram(datagram_, timing_address_, timing.data(), sizeof(timing))) return;

    std::array<std::uint8_t, 2 * kSectorBytes> packets{};
    EncodeSector(packets.data(), scan, 20, 180, nanoseconds);
    EncodeSector(packets.data() + kSectorBytes, scan, 180, 340, nanoseconds);
    Store16(packets.data() + 18, 8192, false);
    for (unsigned bit = 0; bit < 12; ++bit) {
      if (token_ & (1U << bit)) Store16(packets.data() + 22 + 4 * bit, 8192, false);
    }
    token_ = (token_ + 1) & 0xfff;
    // Never queue old scans or leave a partial packet on a live stream.
    const auto sent = send(client_.fd(), packets.data(), packets.size(),
                           MSG_DONTWAIT | MSG_NOSIGNAL);
    if (sent < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return;
    if (sent != static_cast<ssize_t>(packets.size())) client_.reset();
  }

  Socket datagram_;
  Socket server_;
  Socket client_;
  sockaddr_un depth_address_{};
  sockaddr_un imu_address_{};
  sockaddr_un timing_address_{};
  unsigned token_ = 0;
  gz::transport::Node node_;
};
}  // namespace reeman

GZ_ADD_PLUGIN(reeman::DirectDevices, gz::sim::System, gz::sim::ISystemConfigure)
