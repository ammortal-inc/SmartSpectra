# Basler Camera Integration with SmartSpectra SDK

This document outlines different approaches for integrating a Basler Dart camera (using Pylon SDK) with the Presage SmartSpectra SDK for real-time vital sign processing on Linux.

## Overview

The SmartSpectra SDK expects video input through its `video_source` system, which supports live cameras, video files, and image sequences. Since Basler cameras use the proprietary Pylon SDK, we need to bridge between Pylon's API and SmartSpectra's expected input formats.

## Integration Options

### Option 1: Custom Video Source Implementation (Recommended for Production)

**Approach**: Extend SmartSpectra's video source architecture with a custom Pylon-based video source.

#### Implementation

Create a custom video source class that inherits from `presage::smartspectra::video_source::VideoSource`:

```cpp
#include <smartspectra/video_source/video_source.hpp>
#include <pylon/PylonIncludes.h>
#include <opencv2/opencv.hpp>

class PylonVideoSource : public presage::smartspectra::video_source::VideoSource {
private:
    Pylon::CInstantCamera camera;
    Pylon::CGrabResultPtr ptrGrabResult;
    Pylon::CImageFormatConverter formatConverter;
    Pylon::CPylonImage pylonImage;
    
public:
    absl::Status Initialize(const VideoSourceSettings& settings) override {
        try {
            // Initialize Pylon
            Pylon::PylonInitialize();
            
            // Attach to first available camera
            camera.Attach(Pylon::CTlFactory::GetInstance().CreateFirstDevice());
            camera.Open();
            
            // Configure camera parameters
            if (settings.capture_width_px > 0) {
                camera.Width.SetValue(settings.capture_width_px);
            }
            if (settings.capture_height_px > 0) {
                camera.Height.SetValue(settings.capture_height_px);
            }
            
            // Set pixel format (adjust as needed for your camera)
            camera.PixelFormat.SetValue(Basler_UsbCameraParams::PixelFormat_RGB8);
            
            // Configure frame rate if needed
            camera.AcquisitionFrameRateEnable.SetValue(true);
            camera.AcquisitionFrameRate.SetValue(30.0);
            
            // Setup image format converter for OpenCV
            formatConverter.OutputPixelFormat = Pylon::PixelType_BGR8packed;
            formatConverter.OutputBitAlignment = Pylon::OutputBitAlignment_MsbAligned;
            
            // Start grabbing
            camera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);
            
            return absl::OkStatus();
            
        } catch (const Pylon::GenericException& e) {
            return absl::InternalError(std::string("Pylon error: ") + e.GetDescription());
        }
    }
    
    bool SupportsExactFrameTimestamp() const override {
        return true; // Pylon provides hardware timestamps
    }
    
    int64_t GetFrameTimestamp() const override {
        if (ptrGrabResult && ptrGrabResult->GrabSucceeded()) {
            return ptrGrabResult->GetTimeStamp(); // Hardware timestamp in ticks
        }
        return std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }
    
    int GetWidth() override {
        return camera.Width.GetValue();
    }
    
    int GetHeight() override {
        return camera.Height.GetValue();
    }
    
protected:
    void ProducePreTransformFrame(cv::Mat& frame) override {
        try {
            // Wait for and retrieve the next frame
            camera.RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
            
            if (ptrGrabResult->GrabSucceeded()) {
                // Convert Pylon image to OpenCV Mat
                formatConverter.Convert(pylonImage, ptrGrabResult);
                
                // Create OpenCV Mat from Pylon image data
                frame = cv::Mat(ptrGrabResult->GetHeight(), 
                               ptrGrabResult->GetWidth(), 
                               CV_8UC3, 
                               (uint8_t*)pylonImage.GetBuffer()).clone();
            } else {
                // Handle grab failure - could throw exception or use previous frame
                throw std::runtime_error("Frame grab failed");
            }
            
        } catch (const Pylon::GenericException& e) {
            throw std::runtime_error(std::string("Pylon grab error: ") + e.GetDescription());
        }
    }
    
    ~PylonVideoSource() {
        try {
            if (camera.IsGrabbing()) {
                camera.StopGrabbing();
            }
            camera.Close();
            Pylon::PylonTerminate();
        } catch (...) {
            // Ignore cleanup errors
        }
    }
};
```

#### Integration with SmartSpectra

You'll need to modify SmartSpectra's video source factory to use your custom implementation:

```cpp
// In your application code
auto createCustomVideoSource = []() -> std::unique_ptr<VideoSource> {
    return std::make_unique<PylonVideoSource>();
};

// Configure settings as usual, but the video source factory will use Pylon
settings.video_source.device_index = 0; // Can be used to select specific Basler camera
```

#### Pros
- **Direct integration**: No intermediate steps, optimal performance
- **Full control**: Access to all Pylon camera parameters and features
- **Hardware timestamps**: Native support for precise timing
- **Real-time performance**: Minimal latency between camera and processing

#### Cons
- **Complex implementation**: Requires understanding SmartSpectra's video source architecture
- **SDK modifications**: May need to modify SmartSpectra build system
- **Maintenance**: Updates to either SDK may require code changes

---

### Option 2: File Stream Bridge (Simplest for Prototyping)

**Approach**: Use Pylon to continuously save frames as timestamped image files, configure SmartSpectra to read from the file stream.

#### Pylon Capture Application

```cpp
#include <pylon/PylonIncludes.h>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <iomanip>
#include <sstream>

class PylonFileStreamer {
private:
    Pylon::CInstantCamera camera;
    std::string output_directory;
    
public:
    PylonFileStreamer(const std::string& output_dir) : output_directory(output_dir) {
        // Ensure output directory exists
        std::filesystem::create_directories(output_directory);
    }
    
    void Initialize() {
        Pylon::PylonInitialize();
        camera.Attach(Pylon::CTlFactory::GetInstance().CreateFirstDevice());
        camera.Open();
        
        // Configure camera
        camera.Width.SetValue(1280);
        camera.Height.SetValue(720);
        camera.PixelFormat.SetValue(Basler_UsbCameraParams::PixelFormat_RGB8);
        camera.AcquisitionFrameRateEnable.SetValue(true);
        camera.AcquisitionFrameRate.SetValue(30.0);
    }
    
    void StartStreaming() {
        camera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);
        
        Pylon::CGrabResultPtr ptrGrabResult;
        Pylon::CImageFormatConverter formatConverter;
        Pylon::CPylonImage pylonImage;
        
        formatConverter.OutputPixelFormat = Pylon::PixelType_BGR8packed;
        
        while (camera.IsGrabbing()) {
            camera.RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
            
            if (ptrGrabResult->GrabSucceeded()) {
                // Generate timestamp filename (13 digits, microseconds)
                auto timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
                
                std::ostringstream filename;
                filename << output_directory << "/frame" 
                        << std::setfill('0') << std::setw(13) << timestamp << ".png";
                
                // Convert and save
                formatConverter.Convert(pylonImage, ptrGrabResult);
                cv::Mat frame(ptrGrabResult->GetHeight(), 
                             ptrGrabResult->GetWidth(), 
                             CV_8UC3, 
                             (uint8_t*)pylonImage.GetBuffer());
                
                cv::imwrite(filename.str(), frame);
                
                // Optional: Clean up old files to prevent disk filling
                CleanOldFiles();
            }
        }
    }
    
private:
    void CleanOldFiles() {
        // Keep only last 100 frames to prevent disk overflow
        std::vector<std::filesystem::path> files;
        for (const auto& entry : std::filesystem::directory_iterator(output_directory)) {
            if (entry.path().extension() == ".png") {
                files.push_back(entry.path());
            }
        }
        
        if (files.size() > 100) {
            std::sort(files.begin(), files.end());
            for (size_t i = 0; i < files.size() - 100; ++i) {
                std::filesystem::remove(files[i]);
            }
        }
    }
};

// Usage
int main() {
    PylonFileStreamer streamer("/tmp/basler_frames");
    streamer.Initialize();
    streamer.StartStreaming();
    return 0;
}
```

#### SmartSpectra Configuration

```cpp
// Configure SmartSpectra to read from file stream
settings.video_source.file_stream_path = "/tmp/basler_frames/frame0000000000000.png";
settings.video_source.erase_read_files = true;  // Clean up processed files
settings.video_source.rescan_retry_delay_ms = 10;  // Fast scanning for real-time

// Note: Leave input_video_path empty to use file stream mode
settings.video_source.input_video_path = "";
```

#### Pros
- **No SmartSpectra modifications**: Works with existing SDK
- **Simple implementation**: Straightforward on both sides
- **Easy debugging**: Can examine saved frames manually
- **Flexible timing**: Microsecond precision timestamps in filenames
- **Quick prototyping**: Get running quickly for testing

#### Cons
- **File I/O overhead**: Disk writes and reads add latency
- **Disk space management**: Need to clean up files periodically
- **Synchronization complexity**: Two separate processes need coordination
- **Not optimal for production**: Higher latency than direct integration

---

### Option 3: Named Pipe/Shared Memory Bridge

**Approach**: Stream raw frame data through Linux IPC mechanisms for low-latency communication.

#### Named Pipe Implementation

**Pylon Producer**:
```cpp
#include <pylon/PylonIncludes.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

struct FrameHeader {
    int64_t timestamp_us;
    uint32_t width;
    uint32_t height;
    uint32_t channels;
    uint32_t data_size;
};

class PylonPipeStreamer {
private:
    Pylon::CInstantCamera camera;
    int pipe_fd;
    const std::string pipe_path = "/tmp/basler_camera_pipe";
    
public:
    void Initialize() {
        // Create named pipe
        mkfifo(pipe_path.c_str(), 0666);
        
        // Initialize camera
        Pylon::PylonInitialize();
        camera.Attach(Pylon::CTlFactory::GetInstance().CreateFirstDevice());
        camera.Open();
        
        camera.Width.SetValue(1280);
        camera.Height.SetValue(720);
        camera.PixelFormat.SetValue(Basler_UsbCameraParams::PixelFormat_RGB8);
        
        // Open pipe for writing
        pipe_fd = open(pipe_path.c_str(), O_WRONLY);
        if (pipe_fd == -1) {
            throw std::runtime_error("Failed to open pipe for writing");
        }
    }
    
    void StartStreaming() {
        camera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);
        
        Pylon::CGrabResultPtr ptrGrabResult;
        Pylon::CImageFormatConverter formatConverter;
        Pylon::CPylonImage pylonImage;
        
        formatConverter.OutputPixelFormat = Pylon::PixelType_BGR8packed;
        
        while (camera.IsGrabbing()) {
            camera.RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
            
            if (ptrGrabResult->GrabSucceeded()) {
                formatConverter.Convert(pylonImage, ptrGrabResult);
                
                // Prepare frame header
                FrameHeader header;
                header.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
                header.width = ptrGrabResult->GetWidth();
                header.height = ptrGrabResult->GetHeight();
                header.channels = 3;
                header.data_size = header.width * header.height * header.channels;
                
                // Write header and frame data
                write(pipe_fd, &header, sizeof(header));
                write(pipe_fd, pylonImage.GetBuffer(), header.data_size);
            }
        }
    }
    
    ~PylonPipeStreamer() {
        if (pipe_fd >= 0) close(pipe_fd);
        unlink(pipe_path.c_str());
    }
};
```

**Custom SmartSpectra Video Source**:
```cpp
class PipeVideoSource : public presage::smartspectra::video_source::VideoSource {
private:
    int pipe_fd;
    const std::string pipe_path = "/tmp/basler_camera_pipe";
    int64_t current_timestamp = 0;
    int frame_width = 0;
    int frame_height = 0;
    
public:
    absl::Status Initialize(const VideoSourceSettings& settings) override {
        pipe_fd = open(pipe_path.c_str(), O_RDONLY);
        if (pipe_fd == -1) {
            return absl::InternalError("Failed to open camera pipe");
        }
        return absl::OkStatus();
    }
    
    bool SupportsExactFrameTimestamp() const override { return true; }
    int64_t GetFrameTimestamp() const override { return current_timestamp; }
    int GetWidth() override { return frame_width; }
    int GetHeight() override { return frame_height; }
    
protected:
    void ProducePreTransformFrame(cv::Mat& frame) override {
        FrameHeader header;
        
        // Read frame header
        if (read(pipe_fd, &header, sizeof(header)) != sizeof(header)) {
            throw std::runtime_error("Failed to read frame header");
        }
        
        // Read frame data
        std::vector<uint8_t> buffer(header.data_size);
        if (read(pipe_fd, buffer.data(), header.data_size) != header.data_size) {
            throw std::runtime_error("Failed to read frame data");
        }
        
        // Update state
        current_timestamp = header.timestamp_us;
        frame_width = header.width;
        frame_height = header.height;
        
        // Create OpenCV Mat
        frame = cv::Mat(header.height, header.width, CV_8UC3, buffer.data()).clone();
    }
};
```

#### Pros
- **Low latency**: Direct memory transfer without file system
- **Real-time streaming**: Efficient for continuous data flow
- **Synchronization**: Built-in blocking behavior helps with timing
- **Cross-process**: Clean separation between camera and processing

#### Cons
- **Complex implementation**: Requires careful error handling and synchronization
- **Platform-specific**: Linux-specific IPC mechanisms
- **Buffer management**: Need to handle blocking/non-blocking scenarios
- **Custom video source required**: Still need to modify SmartSpectra integration

---

### Option 4: GStreamer Plugin Bridge

**Approach**: Create a GStreamer source plugin for Pylon cameras, leverage SmartSpectra's existing GStreamer backend.

#### Custom GStreamer Element

```cpp
// Simplified GStreamer plugin structure
#include <gst/gst.h>
#include <gst/base/gstpushsrc.h>
#include <pylon/PylonIncludes.h>

typedef struct _GstPylonSrc {
    GstPushSrc base_pylonsrc;
    
    // Pylon camera objects
    Pylon::CInstantCamera* camera;
    Pylon::CGrabResultPtr grab_result;
    Pylon::CImageFormatConverter* converter;
    
    // Properties
    gint device_index;
    gint width;
    gint height;
    gdouble framerate;
} GstPylonSrc;

// Implementation of create() method
static GstFlowReturn gst_pylon_src_create(GstPushSrc* src, GstBuffer** buf) {
    GstPylonSrc* pylonsrc = GST_PYLON_SRC(src);
    
    try {
        // Grab frame from Pylon
        pylonsrc->camera->RetrieveResult(5000, pylonsrc->grab_result, 
                                        Pylon::TimeoutHandling_ThrowException);
        
        if (pylonsrc->grab_result->GrabSucceeded()) {
            // Convert to GStreamer buffer
            Pylon::CPylonImage pylon_image;
            pylonsrc->converter->Convert(pylon_image, pylonsrc->grab_result);
            
            // Create GStreamer buffer
            gsize size = pylon_image.GetImageSize();
            *buf = gst_buffer_new_allocate(NULL, size, NULL);
            
            GstMapInfo map;
            gst_buffer_map(*buf, &map, GST_MAP_WRITE);
            memcpy(map.data, pylon_image.GetBuffer(), size);
            gst_buffer_unmap(*buf, &map);
            
            // Set timestamp
            GST_BUFFER_PTS(*buf) = pylonsrc->grab_result->GetTimeStamp() * GST_USECOND;
            
            return GST_FLOW_OK;
        }
    } catch (const Pylon::GenericException& e) {
        GST_ERROR_OBJECT(src, "Pylon error: %s", e.GetDescription());
        return GST_FLOW_ERROR;
    }
    
    return GST_FLOW_ERROR;
}

// Plugin registration code...
```

#### SmartSpectra Configuration

```cpp
// Configure SmartSpectra to use GStreamer backend with custom pipeline
// This would require modifications to SmartSpectra's GStreamer integration
// to accept custom pipeline descriptions

std::string gst_pipeline = "pylonsrc device=0 ! "
                           "video/x-raw,format=BGR,width=1280,height=720,framerate=30/1 ! "
                           "appsink name=sink";

// Configure SmartSpectra to use this pipeline
// (Implementation depends on SmartSpectra's GStreamer backend flexibility)
```

#### Pros
- **Professional streaming**: GStreamer is industry standard
- **Network capabilities**: Can stream over network protocols
- **Plugin ecosystem**: Access to GStreamer's extensive plugin library
- **Format flexibility**: Supports many pixel formats and codecs

#### Cons
- **Complex development**: GStreamer plugin development is non-trivial
- **Large dependency**: GStreamer adds significant complexity
- **Limited SmartSpectra support**: May need modifications to support custom pipelines
- **Learning curve**: Requires GStreamer expertise

---

## Recommendations

### For Prototyping: Option 2 (File Stream Bridge)
- **Start here** for proof-of-concept and testing
- Quick to implement and debug
- No SmartSpectra code modifications needed
- Easy to validate the integration works end-to-end

### For Production: Option 1 (Custom Video Source)
- **Migrate to this** once you've validated the approach
- Optimal performance and control
- Clean integration with SmartSpectra architecture
- Professional solution for deployment

### Development Path
1. **Phase 1**: Implement Option 2 to validate the concept
2. **Phase 2**: Benchmark performance and identify bottlenecks
3. **Phase 3**: Implement Option 1 for production deployment
4. **Phase 4**: Optimize for your specific use case

## Technical Considerations

### Camera Configuration
- **Resolution**: Match your Basler camera capabilities with SmartSpectra requirements
- **Frame Rate**: 30 FPS is typically sufficient for vital sign monitoring
- **Pixel Format**: RGB8 or BGR8 work well with SmartSpectra
- **Exposure**: Consider auto-exposure settings for varying lighting conditions

### Performance Optimization
- **Buffer Management**: Use appropriate buffer sizes to prevent frame drops
- **CPU Affinity**: Consider pinning camera capture to specific CPU cores
- **Memory Allocation**: Pre-allocate buffers to avoid runtime allocation overhead
- **Timestamp Accuracy**: Use hardware timestamps when available for precise timing

### Error Handling
- **Camera Disconnection**: Implement robust reconnection logic
- **Frame Drop Detection**: Monitor for dropped frames and handle gracefully
- **Resource Cleanup**: Ensure proper cleanup of Pylon resources
- **Logging**: Add comprehensive logging for debugging integration issues

This guide provides a comprehensive foundation for integrating Basler cameras with the SmartSpectra SDK. Start with the file stream approach for rapid prototyping, then migrate to the custom video source implementation for production use.