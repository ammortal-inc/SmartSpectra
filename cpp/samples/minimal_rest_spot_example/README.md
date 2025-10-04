# Minimal REST Spot Example

This example demonstrates the simplest possible implementation of the SmartSpectra C++ SDK for spot measurements using the REST API integration.

## Overview

The minimal REST spot example shows how to:

- Configure basic spot measurement settings
- Set up REST API integration
- Perform a single 30-second physiological measurement
- Handle basic callbacks for metrics output

## Key Features

- **Minimal code footprint** - Demonstrates the absolute minimum required to get started
- **REST API integration** - Uses HTTP REST calls instead of gRPC
- **Spot measurement mode** - Single measurement rather than continuous monitoring
- **Basic error handling** - Simple logging and error reporting

## Usage

```bash
# Build the example (from smartspectra/cpp directory)
cmake --build build --target minimal_rest_spot_example

# Run with your API key
./build/samples/minimal_rest_spot_example/minimal_rest_spot_example
```

## Configuration

Before running, make sure to set your API key in the source code:

```cpp
settings.integration.api_key = "YOUR_API_KEY_HERE";
```

## Basler Camera Support

To use a **Basler GigE Vision or USB3 Vision camera**, uncomment and configure the Pylon settings in the source code:

```cpp
// Optional: Configure for Basler GigE camera
settings.video_source.pylon_camera_serial = "12345678";  // Your camera's serial number
settings.video_source.pylon_pixel_format = "RGB8";
settings.video_source.pylon_auto_exposure = true;
settings.video_source.max_fps = 30.0;

// Advanced GigE optimization (optional)
settings.video_source.pylon_packet_size = 9000;
settings.video_source.pylon_packet_delay = 1000;
settings.video_source.capture_width_px = 1920;
settings.video_source.capture_height_px = 1080;
```

### Prerequisites
- Install [Basler Pylon Camera Software Suite](https://www.baslerweb.com/en/software/pylon/)
- Find your camera's serial number: `/opt/pylon/bin/pylonviewer`

The SDK will automatically detect and use the configured Basler camera instead of the default system camera.

## Code Structure

The example consists of a single `main.cc` file that:

1. **Initializes logging** - Sets up Google logging for debug output
2. **Configures settings** - Sets up spot measurement and REST integration parameters  
3. **Creates container** - Instantiates the SmartSpectra processing container
4. **Starts measurement** - Begins the 30-second spot measurement
5. **Handles callbacks** - Processes physiological metrics as they're generated
6. **Cleanup** - Properly shuts down the measurement system

This example serves as the starting point for developers who want to integrate basic physiological measurements into their applications with minimal overhead.
