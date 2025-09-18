# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

SmartSpectra SDK is a multi-platform SDK for measuring vital signs (pulse, breathing, blood pressure) from camera feeds. It supports Android, iOS, and C++/Linux/macOS platforms.

## Build Commands

### C++ SDK (Primary Platform)

**Prerequisites (Ubuntu/Linux):**
```bash
# Essential build tools
sudo apt install -y build-essential git lsb-release libcurl4-openssl-dev libssl-dev pkg-config libv4l-dev libgles2-mesa-dev libunwind-dev gpg curl

# Install CMake 3.27.0 or newer
curl -L -o cmake-3.27.0-linux-x86_64.sh https://github.com/Kitware/CMake/releases/download/v3.27.0/cmake-3.27.0-linux-x86_64.sh
chmod +x cmake-3.27.0-linux-x86_64.sh
sudo ./cmake-3.27.0-linux-x86_64.sh --skip-license --prefix=/usr/local
```

**Build from source:**
```bash
cd cpp
mkdir build && cd build
cmake .. 
make -j$(nproc)
```

**Common CMake options:**
- `-DBUILD_TESTS=ON/OFF` - Build unit tests (default: ON)
- `-DBUILD_SAMPLES=ON/OFF` - Build example applications (default: ON) 
- `-DBUILD_DOCS=ON/OFF` - Build documentation (default: OFF)
- `-DENABLE_GPU=ON/OFF` - Enable GPU support (default: ON)

**Run tests:**
```bash
cd cpp/build
ctest
```

**Build documentation:**
```bash
cd cpp/build
cmake .. -DBUILD_DOCS=ON
make doxygen
```

### Android SDK

**Build:**
```bash
cd android
./gradlew build
```

**Run tests:**
```bash
cd android  
./gradlew test
```

### iOS/Swift SDK

Build using Xcode or Swift Package Manager. Open `swift/SmartSpectra.xcworkspace` in Xcode.

## Architecture Overview

### Core Components

**C++ SDK Structure (`cpp/smartspectra/`):**
- `container/` - Core processing containers (CPU/GPU, Continuous/Spot modes)
- `video_source/` - Video input handling (cameras, file streams, video files)
  - `camera/` - Live camera support (OpenCV, V4L2 backends)
  - `file_stream/` - Frame sequence processing
- `gui/` - OpenCV-based HUD and visualization components

**Key Container Types:**
- `CpuContinuousRestForegroundContainer` - Real-time vital sign monitoring
- `CpuSpotRestForegroundContainer` - Single measurement sessions

**Video Source Priorities (in order):**
1. Video file input (`input_video_path`)
2. File stream input (frame sequences with timestamps)
3. Live camera stream (`device_index`)

### Sample Applications

Located in `cpp/samples/`:
- `minimal_rest_spot_example/` - Simplest integration example
- `rest_continuous_example/` - Real-time monitoring with GUI
- `rest_spot_example/` - Single measurement with file output

### Camera Integration

**Supported video inputs:**
- Live cameras via OpenCV (V4L2, AVFoundation, GStreamer backends)
- Video files (MP4, AVI, etc.)
- Frame sequences (PNG/JPEG with microsecond timestamps)

**Supported codecs:**
- MJPG (Motion JPEG) - primary
- UYVY (Uncompressed YUV 4:2:2)

**File stream format:**
- Filenames: `frame{13-digit-microsecond-timestamp}.png`
- Example: `frame0000001234567.png` for timestamp 1.234567 seconds

### Authentication

Requires API key from https://physiology.presagetech.com

**Environment variable:**
```bash
export SMARTSPECTRA_API_KEY=your_key_here
```

**In code:**
```cpp
settings.integration.api_key = "your_key_here";
```

## Development Patterns

### Basic C++ Integration

```cpp
#include <smartspectra/container/foreground_container.hpp>
#include <smartspectra/container/settings.hpp>

// Create settings
container::settings::Settings<
    container::settings::OperationMode::Continuous,
    container::settings::IntegrationMode::Rest
> settings;

// Configure camera
settings.video_source.device_index = 0;
settings.video_source.codec = presage::camera::CaptureCodec::MJPG;

// Set API key
settings.integration.api_key = "your_api_key";

// Create container
auto container = std::make_unique<container::CpuContinuousRestForegroundContainer>(settings);

// Set callbacks
container->SetOnCoreMetricsOutput([](const presage::physiology::MetricsBuffer& metrics, int64_t timestamp) {
    // Process vital signs data
    return absl::OkStatus();
});
```

### CMake Integration

```cmake
find_package(SmartSpectra REQUIRED)
add_executable(my_app main.cpp)
target_link_libraries(my_app
    SmartSpectra::Container
    SmartSpectra::Gui  # Optional for HUD/GUI
)
```

## Testing

**C++ tests:** Located in `cpp/tests/`
**Test data:** Available in `cpp/tests/test_data/`

**Run specific test:**
```bash
cd cpp/build
ctest -R test_name
```

## Platform Support

- **Linux:** Ubuntu 22.04/Mint 21+ (amd64) - Primary supported platform
- **macOS:** arm64e - Build from source
- **Android:** API level 26+ - Via Gradle/Maven
- **iOS:** iOS 15.0+ - Via Swift Package Manager

## Dependencies

**C++ Core dependencies:**
- CMake 3.27.0+
- C++20 compiler
- OpenCV (camera/video processing)
- gRPC/Protobuf (API communication)  
- Abseil (status handling)
- V4L2 (Linux camera support)

**Package installation (Ubuntu):**
```bash
# Add Presage repository
curl -s "https://presage-security.github.io/PPA/KEY.gpg" | gpg --dearmor | sudo tee /etc/apt/trusted.gpg.d/presage-technologies.gpg >/dev/null
sudo curl -s --compressed -o /etc/apt/sources.list.d/presage-technologies.list "https://presage-security.github.io/PPA/presage-technologies.list"

# Install
sudo apt update
sudo apt install libsmartspectra-dev
```

## Custom Camera Integration

For non-standard cameras (e.g., Basler, industrial cameras):

1. **File Stream Bridge** - Convert camera frames to timestamped files
2. **Custom Video Source** - Implement VideoSource interface
3. **Named Pipe IPC** - Stream frames via system pipes
4. **GStreamer Plugin** - Integrate as GStreamer source

Detailed integration options available in `.claude/plans/basler-camera-integration-options.md`