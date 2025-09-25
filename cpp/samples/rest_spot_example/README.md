# REST Spot Example

This example demonstrates spot physiological measurements with file output using the SmartSpectra C++ SDK and REST API integration.

## Overview

The REST spot example shows how to:

- Perform single spot measurements (30 seconds by default)
- Use REST API integration for physiological processing
- Process video from camera or file input
- Output results to both console and files
- Handle comprehensive command-line configuration

## Key Features

- **Spot measurement mode** - Single measurement session rather than continuous monitoring
- **REST API integration** - Uses HTTP REST calls to the Physiology API
- **File output** - Saves results to disk for analysis
- **Flexible input** - Supports both camera and video file sources
- **Comprehensive CLI** - Extensive command-line options for configuration

## Usage

```bash
# Build the example (from smartspectra/cpp directory)
cmake --build build --target rest_spot_example

# Run with camera input (30-second measurement)
./build/samples/rest_spot_example/rest_spot_example --also_log_to_stderr --camera_device_index=0 --auto_lock=false --api_key=YOUR_API_KEY_HERE

# Run with video file input
./build/samples/rest_spot_example/rest_spot_example --also_log_to_stderr --input_video_path=/path/to/video.mp4 --api_key=YOUR_API_KEY_HERE

# Run with custom measurement duration (60 seconds)
./build/samples/rest_spot_example/rest_spot_example --also_log_to_stderr --camera_device_index=0 --spot_duration=60.0 --api_key=YOUR_API_KEY_HERE

# Run with Basler GigE camera (requires Pylon SDK)
./build/samples/rest_spot_example/rest_spot_example --also_log_to_stderr --pylon_camera_serial=12345678 --api_key=YOUR_API_KEY_HERE

# Run with high-resolution Basler camera
./build/samples/rest_spot_example/rest_spot_example --also_log_to_stderr \
  --pylon_camera_serial=12345678 \
  --capture_width_px=2048 \
  --capture_height_px=1536 \
  --pylon_packet_size=9000 \
  --spot_duration=45.0 \
  --api_key=YOUR_API_KEY_HERE
```

## Configuration

Key configuration options include:

- `--api_key`: Your Physiology REST API key (required)
- `--spot_duration`: Duration of measurement in seconds (default: 30)
- `--camera_device_index`: Camera to use for input (default: 0)
- `--input_video_path`: Path to video file for processing
- `--output_file_path`: Path for output files

## Basler Camera Support

This example supports **Basler GigE Vision and USB3 Vision cameras** via the Pylon SDK:

### Prerequisites
- Install [Basler Pylon Camera Software Suite](https://www.baslerweb.com/en/software/pylon/)
- Verify camera connection: `/opt/pylon/bin/pylonviewer`

### Pylon Camera Configuration
Use the same Pylon-specific flags as the continuous example:
- `--pylon_camera_serial=SERIAL` - Select specific camera
- `--pylon_exposure_time_us=TIME` - Manual exposure control  
- `--pylon_packet_size=SIZE` - Optimize GigE performance
- `--max_fps=FPS` - Set maximum frame rate

### Example for High-Resolution Spot Measurements
```bash
# 2MP measurement with optimized settings
./rest_spot_example \
  --pylon_camera_serial=12345678 \
  --capture_width_px=1920 --capture_height_px=1200 \
  --pylon_packet_size=9000 \
  --pylon_auto_exposure=true \
  --spot_duration=45.0 \
  --api_key=YOUR_KEY
```

For complete options, run:

```bash
./rest_spot_example --help=main
```

## Output

The example generates:

1. **Console output** - Real-time logging and final results
2. **JSON files** - Structured physiological data
3. **Log files** - Detailed processing information
4. **Status files** - Processing status indicators

## Code Structure

The example demonstrates:

1. **Command-line Processing** - Comprehensive argument parsing with Abseil flags
2. **Settings Configuration** - Spot measurement and REST integration setup
3. **Container Management** - SmartSpectra processing container lifecycle
4. **File I/O Operations** - Reading input and writing output files
5. **Error Handling** - Robust error checking and status reporting
6. **Data Processing** - Physiological metrics extraction and formatting

This example is ideal for batch processing applications, research workflows, or scenarios where you need to analyze pre-recorded video content for physiological data.
