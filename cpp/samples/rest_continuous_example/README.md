# REST Continuous Example

This example demonstrates continuous physiological monitoring with real-time visualization using the SmartSpectra C++ SDK and REST API integration.

## Overview

The REST continuous example shows how to:

- Set up continuous monitoring from a camera or video file
- Use REST API integration for physiological processing
- Display real-time vitals data overlaid on the video stream
- Handle keyboard controls for interactive operation
- Plot physiological data in real-time using OpenCV

## Key Features

- **Continuous monitoring** - Real-time measurement rather than single spot readings
- **REST API integration** - Uses HTTP REST calls to the Physiology API
- **Real-time visualization** - Live HUD display with vitals data
- **Interactive controls** - Keyboard shortcuts for recording control
- **Data plotting** - Real-time graphs of heart rate and breathing patterns

## Usage

```bash
# Build the example (from smartspectra/cpp directory)
cmake --build build --target rest_continuous_example

# Run with camera input
./build/samples/rest_continuous_example/rest_continuous_example --also_log_to_stderr --camera_device_index=0 --auto_lock=false --api_key=YOUR_API_KEY_HERE

# Run with video file input
./build/samples/rest_continuous_example/rest_continuous_example --also_log_to_stderr --input_video_path=/path/to/video.mp4 --api_key=YOUR_API_KEY_HERE

# Run with Basler GigE camera (requires Pylon SDK)
./build/samples/rest_continuous_example/rest_continuous_example --also_log_to_stderr --pylon_camera_serial=12345678 --api_key=YOUR_API_KEY_HERE

# Run with high-performance Basler camera configuration
./build/samples/rest_continuous_example/rest_continuous_example --also_log_to_stderr \
  --pylon_camera_serial=12345678 \
  --pylon_packet_size=9000 \
  --max_fps=60 \
  --capture_width_px=1920 \
  --capture_height_px=1080 \
  --api_key=YOUR_API_KEY_HERE
```

## Configuration

Before running, make sure to:

1. Set your API key via the `--api_key` parameter
2. Configure camera or video file input
3. Optionally adjust visualization and processing parameters

## Basler Camera Support

This example supports **Basler GigE Vision and USB3 Vision cameras** via the Pylon SDK:

### Prerequisites
- Install [Basler Pylon Camera Software Suite](https://www.baslerweb.com/en/software/pylon/)
- Ensure camera is connected and detected: `/opt/pylon/bin/pylonviewer`

### Pylon-Specific Parameters
- `--pylon_camera_serial=SERIAL` - Select camera by serial number
- `--pylon_pixel_format=FORMAT` - Set pixel format (RGB8, BGR8, Mono8, BayerRG8)
- `--pylon_exposure_time_us=TIME` - Manual exposure in microseconds (-1 = auto)
- `--pylon_auto_exposure=true/false` - Enable automatic exposure
- `--pylon_gain=GAIN` - Camera gain (-1 = auto)
- `--pylon_packet_size=SIZE` - GigE packet size (-1 = auto)
- `--pylon_packet_delay=DELAY` - GigE inter-packet delay (-1 = auto)
- `--pylon_buffer_count=COUNT` - Number of acquisition buffers

### Example Configurations
```bash
# Auto-detect first Pylon camera
./rest_continuous_example --api_key=YOUR_KEY

# Select specific camera and optimize for GigE
./rest_continuous_example \
  --pylon_camera_serial=12345678 \
  --pylon_packet_size=9000 \
  --pylon_auto_exposure=false \
  --pylon_exposure_time_us=10000 \
  --api_key=YOUR_KEY
```

## Keyboard Controls

During execution, use these keyboard shortcuts:

- `q` or `ESC`: Exit the application
- `s`: Start/stop recording data (streaming mode only)  
- `e`: Lock/unlock camera exposure (streaming mode only)
- `-` and `=`: Decrease/increase exposure (when locked)

## Code Structure

The example demonstrates:

1. **Settings Configuration** - Sets up continuous measurement and REST integration
2. **Container Setup** - Creates and configures the SmartSpectra processing container
3. **Callback Registration** - Sets up handlers for metrics and video output
4. **Real-time Processing** - Continuous measurement with live feedback
5. **Visualization** - HUD overlay with physiological data and trends
6. **Interactive Control** - Keyboard input handling for user interaction

This example is ideal for applications requiring continuous monitoring with immediate visual feedback, such as fitness applications, health monitoring systems, or research tools.
