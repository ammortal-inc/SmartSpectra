//
// Created by claude on 1/19/25.
// Copyright (c) 2025 Presage Technologies
//

#pragma once

// === standard library includes (if any) ===
#include <vector>
#include <memory>
#include <mutex>
#include <atomic>
// === third-party includes (if any) ===
#ifdef HAVE_PYLON_SDK
#include <pylon/PylonIncludes.h>
#include <pylon/usb/BaslerUsbInstantCamera.h>
#include <pylon/gige/BaslerGigEInstantCamera.h>
#endif
#include <absl/status/statusor.h>
#include <opencv2/opencv.hpp>
// === local includes (if any) ===
#include "camera.hpp"

namespace presage::camera::pylon {

struct PylonCameraInfo {
    std::string device_class;      // e.g., "BaslerGigE", "BaslerUsb"
    std::string model_name;        // Camera model
    std::string serial_number;     // Unique camera identifier
    std::string ip_address;        // For GigE cameras
    std::string user_defined_name; // User-friendly name
    bool is_available;             // Whether camera can be opened
};

struct PylonCameraSettings {
    // Image format settings
    int width = -1;                // -1 means use camera default
    int height = -1;               // -1 means use camera default
    std::string pixel_format = "RGB8";  // RGB8, BGR8, Mono8, etc.
    
    // Acquisition settings
    double frame_rate = -1.0;      // -1.0 means use camera default
    double exposure_time = -1.0;   // in microseconds, -1.0 means auto
    bool auto_exposure = true;     // Enable/disable auto exposure
    
    // GigE specific settings
    int packet_size = -1;          // GigE packet size, -1 means auto
    int packet_delay = -1;         // Inter-packet delay, -1 means auto
    
    // Buffer settings
    int num_buffers = 5;           // Number of acquisition buffers
};

#ifdef HAVE_PYLON_SDK
class PylonCameraManager {
public:
    PylonCameraManager();
    ~PylonCameraManager();
    
    // Camera discovery
    absl::StatusOr<std::vector<PylonCameraInfo>> DiscoverCameras();
    absl::StatusOr<PylonCameraInfo> GetCameraInfo(int device_index);
    absl::StatusOr<PylonCameraInfo> GetCameraInfoBySerial(const std::string& serial_number);
    
    // Camera access
    absl::StatusOr<std::unique_ptr<Pylon::CInstantCamera>> CreateCamera(const PylonCameraInfo& info);
    
private:
    Pylon::CTlFactory* factory_;
    bool pylon_initialized_;
};

class PylonCamera {
public:
    PylonCamera();
    ~PylonCamera() noexcept;
    
    // Camera lifecycle
    absl::Status Initialize(const PylonCameraInfo& camera_info, const PylonCameraSettings& settings);
    absl::Status StartAcquisition();
    absl::Status StopAcquisition();
    bool IsOpen() const;
    bool IsGrabbing() const;
    
    // Image acquisition
    absl::StatusOr<cv::Mat> GrabFrame(int timeout_ms = 5000);
    
    // Camera properties
    int GetWidth() const;
    int GetHeight() const;
    std::string GetPixelFormat() const;
    double GetFrameRate() const;
    
    // Camera controls
    absl::Status SetExposureTime(double exposure_time_us);
    absl::StatusOr<double> GetExposureTime() const;
    absl::Status SetAutoExposure(bool enable);
    absl::StatusOr<bool> IsAutoExposureEnabled() const;
    
    absl::Status SetFrameRate(double fps);
    absl::Status SetGain(double gain);
    absl::StatusOr<double> GetGain() const;
    
    // GigE specific controls
    absl::Status SetPacketSize(int packet_size);
    absl::Status SetPacketDelay(int delay_ticks);
    absl::StatusOr<int> GetPacketSize() const;
    
    // Image format controls
    absl::Status SetImageSize(int width, int height);
    absl::Status SetPixelFormat(const std::string& format);
    
private:
    std::unique_ptr<Pylon::CInstantCamera> camera_;
    PylonCameraSettings current_settings_;
    bool is_initialized_;
    std::unique_ptr<PylonCameraManager> camera_manager_;
    mutable std::mutex camera_mutex_;  // Protects all camera operations
    std::atomic<bool> is_shutting_down_{false};  // Flag for coordinated shutdown
    
    // Helper methods
    absl::Status ConfigureCamera(const PylonCameraSettings& settings);
    cv::Mat ConvertPylonImageToOpenCV(const Pylon::CGrabResultPtr& grab_result);
    std::string PylonPixelTypeToString(Pylon::EPixelType pixel_type);
    Pylon::EPixelType StringToPylonPixelType(const std::string& format);
};
#endif // HAVE_PYLON_SDK

// Utility functions (available even without Pylon SDK)
std::vector<PylonCameraInfo> GetMockPylonCameraList();
bool IsPylonCameraDeviceIndex(int device_index);
absl::StatusOr<std::string> GetPylonCameraName(int device_index);

} // namespace presage::camera::pylon