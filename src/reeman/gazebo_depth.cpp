// OpenNI2 virtual depth device fed by the Gazebo bridge inside the simulation.
#include <Driver/OniDriverAPI.h>
#include <PS1080.h>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {
constexpr int width = 320;
constexpr int height = 160;
constexpr int pixels = width * height;
constexpr char endpoint[] = "/sim-state/openni-depth.sock";
constexpr char uri[] = "gazebo://depth";
constexpr char serial[] = "GAZEBO-DEPTH";
constexpr OniVideoMode mode = {ONI_PIXEL_FORMAT_DEPTH_1_MM, width, height, 10};
constexpr float horizontal_fov = 1.91986218f;

template <typename T>
OniStatus property(void* output, int* size, const T& value) {
    if (*size < static_cast<int>(sizeof(value))) return ONI_STATUS_BAD_PARAMETER;
    std::memcpy(output, &value, sizeof(value));
    *size = sizeof(value);
    return ONI_STATUS_OK;
}

class DepthStream : public oni::driver::StreamBase {
public:
    ~DepthStream() override { stop(); }
    int getRequiredFrameSize() override { return pixels * sizeof(OniDepthPixel); }

    OniStatus start() override {
        if (running_) return ONI_STATUS_OK;
        fd_ = socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC, 0);
        if (fd_ < 0) return ONI_STATUS_ERROR;
        sockaddr_un address{};
        address.sun_family = AF_UNIX;
        std::strcpy(address.sun_path, endpoint);
        if (bind(fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
            close(fd_);
            fd_ = -1;
            return ONI_STATUS_ERROR;
        }
        running_ = true;
        worker_ = std::thread(&DepthStream::receive, this);
        return ONI_STATUS_OK;
    }

    void stop() override {
        if (!running_.exchange(false)) return;
        worker_.join();
        close(fd_);
        unlink(endpoint);
        fd_ = -1;
    }

    OniStatus getProperty(int id, void* data, int* size) override {
        switch (id) {
            case ONI_STREAM_PROPERTY_VIDEO_MODE: return property(data, size, mode);
            case ONI_STREAM_PROPERTY_HORIZONTAL_FOV: return property(data, size, horizontal_fov);
            case ONI_STREAM_PROPERTY_VERTICAL_FOV: return property(data, size, 1.2402307f);
            case ONI_STREAM_PROPERTY_MAX_VALUE: return property(data, size, 6000);
            case ONI_STREAM_PROPERTY_MIN_VALUE: return property(data, size, 0);
            case ONI_STREAM_PROPERTY_STRIDE: return property(data, size, width * 2);
            case ONI_STREAM_PROPERTY_MIRRORING: return property(data, size, mirror_.load());
            default: return ONI_STATUS_NOT_SUPPORTED;
        }
    }

    OniBool isPropertySupported(int id) override {
        return id >= ONI_STREAM_PROPERTY_HORIZONTAL_FOV && id <= ONI_STREAM_PROPERTY_MIRRORING;
    }

    OniStatus setProperty(int id, const void* data, int size) override {
        if (id == ONI_STREAM_PROPERTY_MIRRORING && size == sizeof(OniBool)) {
            mirror_ = *static_cast<const OniBool*>(data);
            return ONI_STATUS_OK;
        }
        if (id == ONI_STREAM_PROPERTY_VIDEO_MODE && size == sizeof(OniVideoMode)) {
            const auto& request = *static_cast<const OniVideoMode*>(data);
            return request.resolutionX == width && request.resolutionY == height &&
                   request.fps == mode.fps && request.pixelFormat == mode.pixelFormat
                ? ONI_STATUS_OK : ONI_STATUS_NOT_SUPPORTED;
        }
        return ONI_STATUS_NOT_SUPPORTED;
    }

private:
    void receive() {
        struct Packet {
            uint64_t stamp;
            OniDepthPixel depth[pixels];
        };
        static_assert(sizeof(Packet) == 8 + pixels * 2, "Unexpected depth packet layout");
        std::vector<Packet> buffer(1);
        Packet& packet = buffer.front();
        pollfd descriptor{fd_, POLLIN, 0};
        int index = 0;
        while (running_) {
            if (poll(&descriptor, 1, 80) != 1 || !(descriptor.revents & POLLIN)) continue;
            if (recv(fd_, &packet, sizeof(packet), MSG_TRUNC) != sizeof(packet)) continue;
            auto& services = getServices();
            OniFrame* frame = services.acquireFrame();
            if (!frame) continue;
            if (frame->dataSize < getRequiredFrameSize()) {
                services.releaseFrame(frame);
                continue;
            }
            auto* output = static_cast<OniDepthPixel*>(frame->data);
            if (mirror_) {
                for (int y = 0; y < height; ++y)
                    for (int x = 0; x < width; ++x)
                        output[y * width + x] = packet.depth[y * width + width - 1 - x];
            } else {
                std::memcpy(output, packet.depth, sizeof(packet.depth));
            }
            frame->dataSize = getRequiredFrameSize();
            frame->sensorType = ONI_SENSOR_DEPTH;
            frame->timestamp = packet.stamp;
            frame->frameIndex = ++index;
            frame->width = width;
            frame->height = height;
            frame->videoMode = mode;
            frame->croppingEnabled = false;
            frame->cropOriginX = frame->cropOriginY = 0;
            frame->stride = width * sizeof(OniDepthPixel);
            raiseNewFrame(frame);
            services.releaseFrame(frame);
        }
    }

    std::thread worker_;
    std::atomic<bool> running_{false};
    std::atomic<OniBool> mirror_{false};
    int fd_ = -1;
};

class DepthDevice : public oni::driver::DeviceBase {
public:
    OniStatus getSensorInfoList(OniSensorInfo** info, int* count) override {
        *info = &depth_;
        *count = 1;
        return ONI_STATUS_OK;
    }
    oni::driver::StreamBase* createStream(OniSensorType type) override {
        return type == ONI_SENSOR_DEPTH ? new DepthStream : nullptr;
    }
    void destroyStream(oni::driver::StreamBase* stream) override { delete stream; }
    OniBool isPropertySupported(int id) override {
        return id == ONI_DEVICE_PROPERTY_SERIAL_NUMBER || id == OBEXTENSION_ID_SERIALNUMBER ||
               id == OBEXTENSION_ID_CAM_PARAMS;
    }
    OniStatus getProperty(int id, void* data, int* size) override {
        if (id == OBEXTENSION_ID_CAM_PARAMS) {
            if (*size < static_cast<int>(sizeof(OBCameraParamList))) return ONI_STATUS_BAD_PARAMETER;
            auto& list = *static_cast<OBCameraParamList*>(data);
            if (!list.pCameraParams || list.nParamSize < 1) return ONI_STATUS_BAD_PARAMETER;
            const float focal = width / (2 * std::tan(horizontal_fov / 2));
            OBCameraParam calibration{};
            calibration.depthIntrinsic = {focal, focal, (width - 1) / 2.0f,
                                          (height - 1) / 2.0f, width, height};
            calibration.rgbIntrinsic = calibration.depthIntrinsic;
            calibration.transform.rot[0] = calibration.transform.rot[4] =
                calibration.transform.rot[8] = 1.0f;
            list.pCameraParams[0] = calibration;
            list.nParamSize = 1;
            *size = sizeof(list);
            return ONI_STATUS_OK;
        }
        return isPropertySupported(id) ? property(data, size, serial) : ONI_STATUS_NOT_SUPPORTED;
    }
private:
    OniVideoMode video_ = mode;
    OniSensorInfo depth_{ONI_SENSOR_DEPTH, 1, &video_};
};

class GazeboDepth : public oni::driver::DriverBase {
public:
    using DriverBase::DriverBase;
    OniStatus initialize(oni::driver::DeviceConnectedCallback connected,
                         oni::driver::DeviceDisconnectedCallback disconnected,
                         oni::driver::DeviceStateChangedCallback changed, void* cookie) override {
        DriverBase::initialize(connected, disconnected, changed, cookie);
        std::strcpy(info_.uri, uri);
        std::strcpy(info_.vendor, "Orbbec");
        std::strcpy(info_.name, "DaBai Max (Gazebo)");
        std::strcpy(info_.serialNumber, serial);
        info_.usbVendorId = 0x2bc5;
        info_.usbProductId = 0x069a;
        deviceConnected(&info_);
        return ONI_STATUS_OK;
    }
    oni::driver::DeviceBase* deviceOpen(const char* requested, const char*, OniStatus& status) override {
        if (std::strcmp(requested, uri)) {
            status = ONI_STATUS_ERROR;
            return nullptr;
        }
        status = ONI_STATUS_OK;
        return new DepthDevice;
    }
    void deviceClose(oni::driver::DeviceBase* device) override { delete device; }
    void shutdown() override {}
private:
    OniDeviceInfo info_{};
};
}

// The bundled public SDK header expects these allocation helpers from OpenNI.
#ifndef XN_NEW
#define XN_NEW(Type, ...) new Type(__VA_ARGS__)
#endif
#ifndef XN_DELETE
#define XN_DELETE(pointer) delete (pointer)
#endif
ONI_EXPORT_DRIVER(GazeboDepth)
