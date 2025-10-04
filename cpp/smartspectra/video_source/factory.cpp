//
// Created by greg on 3/1/24.
// Copyright (c) 2024 Presage Technologies
//

// === standard library includes (if any) ===
// === third-party includes (if any) ===
#include <mediapipe/framework/port/status_macros.h>
#include <mediapipe/framework/port/logging.h>
// === local includes (if any) ===
#include "factory.hpp"
#include "camera/capture_video_source.hpp"
#include "camera/camera_pylon.hpp"
#include "file_stream/file_stream.hpp"

namespace presage::smartspectra::video_source {

absl::StatusOr<std::unique_ptr<VideoSource>> BuildVideoSource(const VideoSourceSettings& settings) {
    LOG(INFO) << "=== CAMERA BACKEND SELECTION DEBUG ===";
    LOG(INFO) << "Input video path: '" << settings.input_video_path << "'";
    LOG(INFO) << "Pylon camera serial: '" << settings.pylon_camera_serial << "'";
    LOG(INFO) << "Device index: " << settings.device_index;
    
    std::unique_ptr<VideoSource> video_source;
    if (!settings.input_video_path.empty()) {
        LOG(INFO) << "Using video file input mode";
        // if timestamp txt file was provided
        if (!settings.input_video_time_path.empty()) {
            video_source = std::make_unique<capture::CaptureVideoAndTimeStampFile>();
        } else {
            video_source = std::make_unique<capture::CaptureVideoFileSource>();
        }
    } else if (!settings.file_stream_path.empty()) {
        LOG(INFO) << "Using file stream input mode";
        video_source = std::make_unique<file_stream::FileStreamVideoSource>();
    } else {
        LOG(INFO) << "Using camera input mode";
        // Camera source - check for Pylon cameras first, then fall back to standard cameras
        bool use_pylon = false;
        
#ifdef HAVE_PYLON_SDK
        LOG(INFO) << "HAVE_PYLON_SDK is defined - Pylon support is available";
        
        // Check if user specified a Pylon camera by serial number
        if (!settings.pylon_camera_serial.empty()) {
            LOG(INFO) << "Pylon camera serial specified: " << settings.pylon_camera_serial;
            use_pylon = true;
            LOG(INFO) << "Using Pylon camera with serial number: " << settings.pylon_camera_serial;
        } else if (presage::camera::pylon::IsPylonCameraDeviceIndex(settings.device_index)) {
            LOG(INFO) << "Device index " << settings.device_index << " corresponds to a Pylon camera";
            use_pylon = true;
            LOG(INFO) << "Using Pylon camera for device index " << settings.device_index;
        } else {
            LOG(INFO) << "No Pylon camera serial specified and device index " << settings.device_index << " is not a Pylon camera";
            // If device_index doesn't correspond to a Pylon camera, 
            // check if there are any Pylon cameras at all for fallback
            LOG(INFO) << "Attempting to discover Pylon cameras...";
            presage::camera::pylon::PylonCameraManager manager;
            auto cameras_or_error = manager.DiscoverCameras();
            if (cameras_or_error.ok()) {
                const auto& cameras = cameras_or_error.value();
                LOG(INFO) << "Pylon camera discovery successful. Found " << cameras.size() << " cameras";
                for (size_t i = 0; i < cameras.size(); ++i) {
                    const auto& camera = cameras[i];
                    LOG(INFO) << "  Camera " << i << ": Model=" << camera.model_name 
                             << ", Serial=" << camera.serial_number 
                             << ", IP=" << camera.ip_address;
                }
                if (!cameras.empty()) {
                    // There are Pylon cameras, but device_index might be for V4L2 camera
                    // For now, prefer V4L2 cameras if device_index doesn't match Pylon
                    LOG(INFO) << "Pylon cameras detected but device_index " << settings.device_index 
                             << " doesn't correspond to a Pylon camera. Using standard camera.";
                }
            } else {
                LOG(ERROR) << "Pylon camera discovery failed: " << cameras_or_error.status().message();
            }
        }
        
        if (use_pylon) {
            LOG(INFO) << "Creating PylonCameraSource";
            video_source = std::make_unique<capture::PylonCameraSource>();
        } else {
            LOG(INFO) << "Creating CaptureCameraSource (V4L2)";
            video_source = std::make_unique<capture::CaptureCameraSource>();
        }
#else
        LOG(INFO) << "HAVE_PYLON_SDK is NOT defined - Pylon support is not available";
        // No Pylon SDK available, use standard camera
        video_source = std::make_unique<capture::CaptureCameraSource>();
#endif
    }
    LOG(INFO) << "Initializing video source...";
    MP_RETURN_IF_ERROR(video_source->Initialize(settings));
    LOG(INFO) << "Video source initialization completed successfully";
    return video_source;
}

} // namespace presage::smartspectra::video_source
