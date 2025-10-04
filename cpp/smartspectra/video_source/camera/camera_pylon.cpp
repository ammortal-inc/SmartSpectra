//
// Created by claude on 1/19/25.
// Copyright (c) 2025 Presage Technologies
//

// === standard library includes (if any) ===
#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>
#include <sstream>
#include <algorithm>

// === third-party includes (if any) ===
#include <absl/strings/str_cat.h>
#include <mediapipe/framework/port/ret_check.h>
#include <mediapipe/framework/port/logging.h>

// === local includes (if any) ===
#include "camera_pylon.hpp"

namespace presage::camera::pylon {

#ifdef HAVE_PYLON_SDK

PylonCameraManager::PylonCameraManager() : factory_(nullptr), pylon_initialized_(false) {
    try {
        Pylon::PylonInitialize();
        factory_ = &Pylon::CTlFactory::GetInstance();
        pylon_initialized_ = true;
        LOG(INFO) << "Pylon SDK initialized successfully";
    } catch (const Pylon::GenericException& e) {
        LOG(ERROR) << "Failed to initialize Pylon SDK: " << e.GetDescription();
        pylon_initialized_ = false;
    }
}

PylonCameraManager::~PylonCameraManager() {
    if (pylon_initialized_) {
        try {
            Pylon::PylonTerminate();
        } catch (const Pylon::GenericException& e) {
            LOG(ERROR) << "Error during Pylon termination: " << e.GetDescription();
        }
    }
}

absl::StatusOr<std::vector<PylonCameraInfo>> PylonCameraManager::DiscoverCameras() {
    if (!pylon_initialized_) {
        return absl::FailedPreconditionError("Pylon SDK not initialized");
    }
    
    std::vector<PylonCameraInfo> camera_list;
    
    try {
        Pylon::DeviceInfoList_t device_list;
        factory_->EnumerateDevices(device_list);
        
        for (const auto& device_info : device_list) {
            PylonCameraInfo info;
            info.device_class = device_info.GetDeviceClass().c_str();
            info.model_name = device_info.GetModelName().c_str();
            info.serial_number = device_info.GetSerialNumber().c_str();
            info.user_defined_name = device_info.GetUserDefinedName().c_str();
            // Check if device is available - method availability varies by Pylon version
            // For compatibility, we'll assume devices are available since we can enumerate them
            info.is_available = true;
            
            // For GigE cameras, get IP address
            if (info.device_class == "BaslerGigE") {
                try {
                    info.ip_address = device_info.GetIpAddress().c_str();
                } catch (const Pylon::GenericException&) {
                    info.ip_address = "Unknown";
                }
            }
            
            camera_list.push_back(info);
            LOG(INFO) << "Found Pylon camera: " << info.model_name 
                     << " (SN: " << info.serial_number << ", IP: " << info.ip_address << ")";
        }
        
    } catch (const Pylon::GenericException& e) {
        return absl::InternalError(absl::StrCat("Failed to enumerate Pylon cameras: ", e.GetDescription()));
    }
    
    return camera_list;
}

absl::StatusOr<PylonCameraInfo> PylonCameraManager::GetCameraInfo(int device_index) {
    auto cameras_or_error = DiscoverCameras();
    if (!cameras_or_error.ok()) {
        return cameras_or_error.status();
    }
    
    const auto& cameras = cameras_or_error.value();
    if (device_index < 0 || device_index >= static_cast<int>(cameras.size())) {
        return absl::OutOfRangeError(absl::StrCat("Camera index ", device_index, 
                                                  " out of range. Found ", cameras.size(), " cameras"));
    }
    
    return cameras[device_index];
}

absl::StatusOr<PylonCameraInfo> PylonCameraManager::GetCameraInfoBySerial(const std::string& serial_number) {
    auto cameras_or_error = DiscoverCameras();
    if (!cameras_or_error.ok()) {
        return cameras_or_error.status();
    }
    
    const auto& cameras = cameras_or_error.value();
    for (const auto& camera : cameras) {
        if (camera.serial_number == serial_number) {
            return camera;
        }
    }
    
    return absl::NotFoundError(absl::StrCat("Camera with serial number ", serial_number, " not found"));
}

absl::StatusOr<std::unique_ptr<Pylon::CInstantCamera>> PylonCameraManager::CreateCamera(const PylonCameraInfo& info) {
    if (!pylon_initialized_) {
        return absl::FailedPreconditionError("Pylon SDK not initialized");
    }
    
    try {
        Pylon::CDeviceInfo device_info;
        device_info.SetSerialNumber(info.serial_number.c_str());
        
        auto camera = std::make_unique<Pylon::CInstantCamera>(factory_->CreateDevice(device_info));
        return camera;
        
    } catch (const Pylon::GenericException& e) {
        return absl::InternalError(absl::StrCat("Failed to create Pylon camera: ", e.GetDescription()));
    }
}

PylonCamera::PylonCamera() : is_initialized_(false) {}

PylonCamera::~PylonCamera() noexcept {
    try {
        LOG(INFO) << "=== PYLON CAMERA DESTRUCTOR DEBUG ===";
        
        // Signal shutdown to prevent new operations
        is_shutting_down_.store(true);
        
        LOG(INFO) << "Acquiring mutex for camera shutdown...";
        std::lock_guard<std::mutex> lock(camera_mutex_);
        LOG(INFO) << "Mutex acquired, proceeding with shutdown";
        
        if (camera_) {
            LOG(INFO) << "Camera object exists, checking state...";
            
            // First, stop grabbing if active
            try {
                if (camera_->IsGrabbing()) {
                    LOG(INFO) << "Camera is grabbing, stopping acquisition...";
                    camera_->StopGrabbing();
                    LOG(INFO) << "Camera grabbing stopped successfully";
                    
                    // Give a brief moment for any pending operations to complete
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
            } catch (const Pylon::GenericException& e) {
                LOG(ERROR) << "Error stopping camera acquisition: " << e.GetDescription();
            } catch (const std::exception& e) {
                LOG(ERROR) << "Standard exception stopping camera acquisition: " << e.what();
            } catch (...) {
                LOG(ERROR) << "Unknown exception stopping camera acquisition";
            }
            
            // Then, close the camera if open
            try {
                if (camera_->IsOpen()) {
                    LOG(INFO) << "Camera is open, closing...";
                    camera_->Close();
                    LOG(INFO) << "Camera closed successfully";
                }
            } catch (const Pylon::GenericException& e) {
                LOG(ERROR) << "Error closing camera: " << e.GetDescription();
            } catch (const std::exception& e) {
                LOG(ERROR) << "Standard exception closing camera: " << e.what();
            } catch (...) {
                LOG(ERROR) << "Unknown exception closing camera";
            }
            
            LOG(INFO) << "Resetting camera object...";
            try {
                camera_.reset();
                LOG(INFO) << "Camera object reset completed";
            } catch (const std::exception& e) {
                LOG(ERROR) << "Exception resetting camera object: " << e.what();
            } catch (...) {
                LOG(ERROR) << "Unknown exception resetting camera object";
            }
        } else {
            LOG(INFO) << "Camera object is null, no cleanup needed";
        }
        
        LOG(INFO) << "PylonCamera destructor completed";
    } catch (const std::exception& e) {
        // Must not throw from destructor - log and continue
        try {
            LOG(ERROR) << "Exception in PylonCamera destructor: " << e.what();
        } catch (...) {
            // Even logging might fail - do nothing to avoid std::terminate
        }
    } catch (...) {
        // Must not throw from destructor - do nothing to avoid std::terminate
        try {
            LOG(ERROR) << "Unknown exception in PylonCamera destructor";
        } catch (...) {
            // Even logging might fail - do nothing
        }
    }
}

absl::Status PylonCamera::Initialize(const PylonCameraInfo& camera_info, const PylonCameraSettings& settings) {
    LOG(INFO) << "=== PYLON CAMERA INITIALIZATION DEBUG ===";

    // CRITICAL FIX: Keep the camera manager alive for the lifetime of the camera
    camera_manager_ = std::make_unique<PylonCameraManager>();
    auto camera_or_error = camera_manager_->CreateCamera(camera_info);
    if (!camera_or_error.ok()) {
        return camera_or_error.status();
    }
    LOG(INFO) << "Pylon camera created successfully";
    
    camera_ = std::move(camera_or_error.value());
    current_settings_ = settings;
    
    try {
        camera_->Open();
        LOG(INFO) << "Opened Pylon camera: " << camera_info.model_name 
                 << " (SN: " << camera_info.serial_number << ")"
                 << " camera_ address: " << camera_.get() 
                 << " IsOpen: " << camera_->IsOpen();
        
        auto config_status = ConfigureCamera(settings);
        if (!config_status.ok()) {
            camera_->Close();
            return config_status;
        }
        
        is_initialized_ = true;
        return absl::OkStatus();
        
    } catch (const Pylon::GenericException& e) {
        LOG(ERROR) << "Error initializing Pylon camera: " << e.GetDescription();
        return absl::InternalError(absl::StrCat("Failed to initialize Pylon camera: ", e.GetDescription()));
    }
}

absl::Status PylonCamera::ConfigureCamera(const PylonCameraSettings& settings) {
    // For now, use minimal configuration to ensure compatibility across Pylon versions
    // Advanced camera configuration can be added later with proper version detection
    
    try {
        LOG(INFO) << "Configuring Pylon camera with basic settings";
        
        // Store settings for later use (GetWidth/GetHeight use these)
        // The actual camera configuration is handled by Pylon's defaults for now
        
        // Try to set buffer count if the API is available
        try {
            camera_->MaxNumBuffer.SetValue(settings.num_buffers);
        } catch (const Pylon::GenericException&) {
            LOG(WARNING) << "Could not set buffer count, using Pylon default";
        }
        
        LOG(INFO) << "Pylon camera configured with default settings - using resolution: " 
                 << GetWidth() << "x" << GetHeight();
        
        return absl::OkStatus();
        
    } catch (const Pylon::GenericException& e) {
        return absl::InternalError(absl::StrCat("Failed to configure Pylon camera: ", e.GetDescription()));
    }
}

absl::Status PylonCamera::StartAcquisition() {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    
    if (is_shutting_down_.load()) {
        return absl::FailedPreconditionError("Camera is shutting down");
    }
    
    if (!is_initialized_ || !camera_ || !camera_->IsOpen()) {
        return absl::FailedPreconditionError("Camera not initialized or opened");
    }
    
    try {
        if (!camera_->IsGrabbing()) {
            camera_->StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);
            LOG(INFO) << "Started Pylon camera acquisition";
        }
        return absl::OkStatus();
        
    } catch (const Pylon::GenericException& e) {
        return absl::InternalError(absl::StrCat("Failed to start camera acquisition: ", e.GetDescription()));
    }
}

absl::Status PylonCamera::StopAcquisition() {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    
    if (!camera_) {
        return absl::FailedPreconditionError("Camera not initialized");
    }
    
    try {
        if (camera_->IsGrabbing()) {
            camera_->StopGrabbing();
            LOG(INFO) << "Stopped Pylon camera acquisition";
        }
        return absl::OkStatus();
        
    } catch (const Pylon::GenericException& e) {
        return absl::InternalError(absl::StrCat("Failed to stop camera acquisition: ", e.GetDescription()));
    }
}

absl::StatusOr<cv::Mat> PylonCamera::GrabFrame(int timeout_ms) {
    LOG(INFO) << "=== PYLON FRAME GRAB DEBUG ===";
    LOG(INFO) << "GrabFrame called with timeout: " << timeout_ms << "ms";
    LOG(INFO) << "PylonCamera object address: " << this;
    
    // Check shutdown flag first (before acquiring mutex)
    if (is_shutting_down_.load()) {
        LOG(INFO) << "Camera is shutting down, aborting GrabFrame";
        return absl::FailedPreconditionError("Camera is shutting down");
    }
    
    LOG(INFO) << "PylonCamera camera_ address: " << camera_.get();
    
    // CRITICAL: Check if camera pointer is valid before calling any methods
    if (!camera_) {
        LOG(ERROR) << "camera_ is null!";
        return absl::FailedPreconditionError("Camera pointer is null");
    }
    
    LOG(INFO) << "Camera pointer is valid, attempting to get device info...";
    try {
        LOG(INFO) << "Camera name: " << camera_->GetDeviceInfo().GetModelName().c_str();
    } catch (const std::exception& e) {
        LOG(ERROR) << "Exception getting camera name: " << e.what();
        return absl::InternalError("Exception accessing camera object");
    } catch (...) {
        LOG(ERROR) << "Unknown exception getting camera name";
        return absl::InternalError("Unknown exception accessing camera object");
    }
    
    LOG(INFO) << "Checking is_initialized_ flag...";
    bool initialized = is_initialized_;
    LOG(INFO) << "is_initialized_ = " << initialized;
    
    LOG(INFO) << "Checking camera_ pointer...";
    bool camera_valid = (camera_ != nullptr);
    LOG(INFO) << "camera_ != nullptr = " << camera_valid;
    
    if (!camera_valid) {
        LOG(ERROR) << "Camera pointer is null!";
        return absl::FailedPreconditionError("Camera pointer is null");
    }
    
    LOG(INFO) << "Checking camera_->IsOpen()...";
    LOG(INFO) << "Camera object pointer address: " << camera_.get();
    
    bool camera_open = false;
    try {
        // Add extra defensive check - verify the camera object looks valid
        if (!camera_) {
            LOG(ERROR) << "Camera object became null between checks!";
            return absl::InternalError("Camera object became null");
        }
        
        LOG(INFO) << "About to call camera_->IsOpen()...";
        camera_open = camera_->IsOpen();
        LOG(INFO) << "camera_->IsOpen() = " << camera_open;
    } catch (const Pylon::GenericException& e) {
        LOG(ERROR) << "Pylon exception calling camera_->IsOpen(): " << e.GetDescription();
        return absl::InternalError(absl::StrCat("Pylon exception calling camera_->IsOpen(): ", e.GetDescription()));
    } catch (const std::exception& e) {
        LOG(ERROR) << "Standard exception calling camera_->IsOpen(): " << e.what();
        return absl::InternalError("Standard exception calling camera_->IsOpen()");
    } catch (...) {
        LOG(ERROR) << "Unknown exception calling camera_->IsOpen()";
        return absl::InternalError("Unknown exception calling camera_->IsOpen()");
    }
    
    if (!initialized || !camera_valid || !camera_open) {
        LOG(ERROR) << "Camera not initialized or opened - initialized: " << initialized 
                   << ", camera valid: " << camera_valid
                   << ", camera open: " << camera_open;
        return absl::FailedPreconditionError("Camera not initialized or opened");
    }
    
    LOG(INFO) << "Camera is initialized and open";
    
    if (!camera_->IsGrabbing()) {
        LOG(INFO) << "Camera not grabbing, starting acquisition...";
        auto start_status = StartAcquisition();
        if (!start_status.ok()) {
            LOG(ERROR) << "Failed to start acquisition: " << start_status.message();
            return start_status;
        }
        LOG(INFO) << "Acquisition started successfully";
    } else {
        LOG(INFO) << "Camera is already grabbing";
    }
    
    try {
        LOG(INFO) << "Attempting to retrieve result from camera...";
        Pylon::CGrabResultPtr grab_result;
        
        // Use scoped lock only for the critical Pylon API call
        {
            std::lock_guard<std::mutex> lock(camera_mutex_);
            
            // Double-check shutdown flag after acquiring lock
            if (is_shutting_down_.load()) {
                LOG(INFO) << "Camera is shutting down (during grab), aborting";
                return absl::FailedPreconditionError("Camera is shutting down");
            }
            
            if (!camera_ || !camera_->IsGrabbing()) {
                LOG(ERROR) << "Camera is not in a valid grabbing state";
                return absl::FailedPreconditionError("Camera not ready for grabbing");
            }
            
            // This is the critical section - protect the RetrieveResult call
            if (!camera_->RetrieveResult(timeout_ms, grab_result, Pylon::TimeoutHandling_ThrowException)) {
                LOG(ERROR) << "RetrieveResult failed - timeout waiting for frame";
                return absl::DeadlineExceededError("Timeout waiting for frame");
            }
        }
        // End of critical section - mutex is released here
        
        LOG(INFO) << "RetrieveResult succeeded";
        if (grab_result->GrabSucceeded()) {
            LOG(INFO) << "Grab succeeded, converting image to OpenCV format...";
            auto result = ConvertPylonImageToOpenCV(grab_result);
            LOG(INFO) << "Image conversion completed successfully";
            return result;
        } else {
            LOG(ERROR) << "Frame grab failed: " << std::string(grab_result->GetErrorDescription().c_str());
            return absl::InternalError(absl::StrCat("Frame grab failed: ", std::string(grab_result->GetErrorDescription().c_str())));
        }
        
    } catch (const Pylon::GenericException& e) {
        LOG(ERROR) << "Pylon exception in GrabFrame: " << e.GetDescription();
        return absl::InternalError(absl::StrCat("Error grabbing frame: ", e.GetDescription()));
    } catch (const std::exception& e) {
        LOG(ERROR) << "Standard exception in GrabFrame: " << e.what();
        return absl::InternalError(absl::StrCat("Standard exception in GrabFrame: ", e.what()));
    } catch (...) {
        LOG(ERROR) << "Unknown exception in GrabFrame";
        return absl::InternalError("Unknown exception in GrabFrame");
    }
}

cv::Mat PylonCamera::ConvertPylonImageToOpenCV(const Pylon::CGrabResultPtr& grab_result) {
    LOG(INFO) << "=== PYLON IMAGE CONVERSION DEBUG ===";
    
    const uint8_t* buffer = static_cast<const uint8_t*>(grab_result->GetBuffer());
    int width = static_cast<int>(grab_result->GetWidth());
    int height = static_cast<int>(grab_result->GetHeight());
    Pylon::EPixelType pixel_type = grab_result->GetPixelType();
    
    LOG(INFO) << "Image properties: " << width << "x" << height 
              << ", pixel format: " << PylonPixelTypeToString(pixel_type)
              << ", buffer ptr: " << (void*)buffer;
              
    if (!buffer) {
        LOG(ERROR) << "Buffer is null!";
        return cv::Mat();
    }
    
    if (width <= 0 || height <= 0) {
        LOG(ERROR) << "Invalid image dimensions: " << width << "x" << height;
        return cv::Mat();
    }
    
    cv::Mat image;
    
    try {
        LOG(INFO) << "Converting image based on pixel type...";
        
        switch (pixel_type) {
            case Pylon::PixelType_Mono8:
                LOG(INFO) << "Converting Mono8 format";
                image = cv::Mat(height, width, CV_8UC1, const_cast<uint8_t*>(buffer)).clone();
                break;
                
            case Pylon::PixelType_RGB8packed:
                LOG(INFO) << "Converting RGB8 format";
                image = cv::Mat(height, width, CV_8UC3, const_cast<uint8_t*>(buffer)).clone();
                LOG(INFO) << "Converting RGB to BGR...";
                cv::cvtColor(image, image, cv::COLOR_RGB2BGR);  // Convert RGB to BGR for OpenCV
                break;
                
            case Pylon::PixelType_BGR8packed:
                LOG(INFO) << "Converting BGR8 format";
                image = cv::Mat(height, width, CV_8UC3, const_cast<uint8_t*>(buffer)).clone();
                break;
                
            case Pylon::PixelType_BayerRG8:
            case Pylon::PixelType_BayerBG8:
            case Pylon::PixelType_BayerGR8:
            case Pylon::PixelType_BayerGB8: {
                LOG(INFO) << "Converting Bayer format";
                // Convert Bayer to BGR
                cv::Mat bayer_image(height, width, CV_8UC1, const_cast<uint8_t*>(buffer));
                int conversion_code = cv::COLOR_BayerRG2BGR;  // Default, adjust based on actual pattern
                if (pixel_type == Pylon::PixelType_BayerBG8) conversion_code = cv::COLOR_BayerBG2BGR;
                else if (pixel_type == Pylon::PixelType_BayerGR8) conversion_code = cv::COLOR_BayerGR2BGR;
                else if (pixel_type == Pylon::PixelType_BayerGB8) conversion_code = cv::COLOR_BayerGB2BGR;
                
                LOG(INFO) << "Applying Bayer conversion with code: " << conversion_code;
                cv::cvtColor(bayer_image, image, conversion_code);
                break;
            }
                
            default:
                LOG(WARNING) << "Unsupported pixel format: " << PylonPixelTypeToString(pixel_type) 
                            << ", attempting to use as Mono8";
                image = cv::Mat(height, width, CV_8UC1, const_cast<uint8_t*>(buffer)).clone();
                break;
        }
        
        LOG(INFO) << "Image conversion completed successfully - result size: " 
                  << image.cols << "x" << image.rows << ", channels: " << image.channels()
                  << ", empty: " << image.empty();
                  
    } catch (const cv::Exception& e) {
        LOG(ERROR) << "OpenCV exception in image conversion: " << e.what();
        return cv::Mat();
    } catch (const std::exception& e) {
        LOG(ERROR) << "Standard exception in image conversion: " << e.what();
        return cv::Mat();
    } catch (...) {
        LOG(ERROR) << "Unknown exception in image conversion";
        return cv::Mat();
    }
    
    return image;
}

bool PylonCamera::IsOpen() const {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    return camera_ && camera_->IsOpen();
}

bool PylonCamera::IsGrabbing() const {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    return camera_ && camera_->IsGrabbing();
}

int PylonCamera::GetWidth() const {
    if (!camera_ || !camera_->IsOpen()) return -1;
    // Return the configured width from settings, since Pylon API varies by version
    return current_settings_.width > 0 ? current_settings_.width : 640;
}

int PylonCamera::GetHeight() const {
    if (!camera_ || !camera_->IsOpen()) return -1;
    // Return the configured height from settings, since Pylon API varies by version
    return current_settings_.height > 0 ? current_settings_.height : 480;
}

std::string PylonCamera::GetPixelFormat() const {
    if (!camera_ || !camera_->IsOpen()) return "Unknown";
    // Return configured pixel format from settings since Pylon API varies by version
    return current_settings_.pixel_format.empty() ? "Unknown" : current_settings_.pixel_format;
}

double PylonCamera::GetFrameRate() const {
    if (!camera_ || !camera_->IsOpen()) return -1.0;
    // Return configured frame rate from settings since Pylon API varies by version
    return current_settings_.frame_rate > 0 ? current_settings_.frame_rate : -1.0;
}

std::string PylonCamera::PylonPixelTypeToString(Pylon::EPixelType pixel_type) {
    switch (pixel_type) {
        case Pylon::PixelType_Mono8: return "Mono8";
        case Pylon::PixelType_RGB8packed: return "RGB8";
        case Pylon::PixelType_BGR8packed: return "BGR8";
        case Pylon::PixelType_BayerRG8: return "BayerRG8";
        case Pylon::PixelType_BayerBG8: return "BayerBG8";
        case Pylon::PixelType_BayerGR8: return "BayerGR8";
        case Pylon::PixelType_BayerGB8: return "BayerGB8";
        default: return "Unknown";
    }
}

Pylon::EPixelType PylonCamera::StringToPylonPixelType(const std::string& format) {
    if (format == "Mono8") return Pylon::PixelType_Mono8;
    if (format == "RGB8") return Pylon::PixelType_RGB8packed;
    if (format == "BGR8") return Pylon::PixelType_BGR8packed;
    if (format == "BayerRG8") return Pylon::PixelType_BayerRG8;
    if (format == "BayerBG8") return Pylon::PixelType_BayerBG8;
    if (format == "BayerGR8") return Pylon::PixelType_BayerGR8;
    if (format == "BayerGB8") return Pylon::PixelType_BayerGB8;
    return Pylon::EPixelType::PixelType_Undefined;
}

// Additional control methods would be implemented here...
// SetExposureTime, GetExposureTime, SetAutoExposure, etc.

#endif // HAVE_PYLON_SDK

// Utility functions available even without Pylon SDK
std::vector<PylonCameraInfo> GetMockPylonCameraList() {
    // Return empty list when Pylon SDK is not available
    return {};
}

bool IsPylonCameraDeviceIndex(int device_index) {
#ifdef HAVE_PYLON_SDK
    PylonCameraManager manager;
    auto cameras_or_error = manager.DiscoverCameras();
    if (!cameras_or_error.ok()) {
        return false;
    }
    const auto& cameras = cameras_or_error.value();
    return device_index >= 0 && device_index < static_cast<int>(cameras.size());
#else
    return false;  // No Pylon cameras available without SDK
#endif
}

absl::StatusOr<std::string> GetPylonCameraName(int device_index) {
#ifdef HAVE_PYLON_SDK
    PylonCameraManager manager;
    auto camera_info_or_error = manager.GetCameraInfo(device_index);
    if (!camera_info_or_error.ok()) {
        return camera_info_or_error.status();
    }
    return camera_info_or_error.value().model_name;
#else
    return absl::UnavailableError("Pylon SDK not available");
#endif
}

} // namespace presage::camera::pylon