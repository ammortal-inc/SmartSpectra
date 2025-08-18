# iOS UI Tests - Demo App

This directory contains UI tests for the SmartSpectra demo app, covering core functionality including measurement flows, camera controls, and app navigation.

## Overview

The UI test suite includes:

- **Core App Functionality Tests** - Basic app controls and navigation
- **Continuous Measurement Flow** - Full workflow for continuous vital signs monitoring
- **Spot Measurement Flow** - Complete 30-second measurement workflow with results validation
- **Launch Tests** - App startup and initial state verification
- **Measurement Preparation** - Helper tests for manual testing setup

## Prerequisites

### Required Tools

- Xcode 15.0+
- iOS Simulator or physical iOS device (iOS 15.0+)
- Camera permissions (for real device testing)

### Setup

1. Open `SmartSpectra.xcworkspace` in Xcode (or just `demo-app.xcodeproj` in Xcode and add sdk dependency)
2. Ensure the demo-app scheme is selected
3. For real device testing:
   - Connect your iOS device
   - Enable Developer Mode in device settings
   - Grant camera permissions when prompted

## Running Tests

### Via Xcode

1. Open the project in Xcode
2. Select Product → Test (⌘+U) to run all tests
3. Or use Test Navigator to run individual test classes:
   - `demo_app_UITests` - Core functionality
   - `demo_app_MeasurementTests` - Full measurement workflows
   - `demo_app_UITestsLaunchTests` - App launch verification

### Via Command Line

**NOTE** Currently not tested. will be updated with tested commands in future merge requests
<!--
```bash
# Run all UI tests
xcodebuild test -workspace SmartSpectra.xcworkspace -scheme demo-app -destination 'platform=iOS Simulator,name=iPhone 15,OS=17.0'

# Run specific test class
xcodebuild test -workspace SmartSpectra.xcworkspace -scheme demo-app -destination 'platform=iOS Simulator,name=iPhone 15,OS=17.0' -only-testing:demo-app-UITests/demo_app_MeasurementTests

# Run on physical device (replace with your device ID)
xcodebuild test -workspace SmartSpectra.xcworkspace -scheme demo-app -destination 'platform=iOS,id=YOUR_DEVICE_ID'
```

### Via Fastlane
The project includes Fastlane configuration for CI/CD. Tests can be integrated into release workflows.
 -->

## Test Structure

### Base Classes

- **`UITestBase`** (`UITestHelpers.swift`)
  - Base test class with common setup and utility methods
  - Handles app launch, screenshots, and device detection
  - Provides helper methods for waiting and interaction

### Test Files

#### `demo_app_UITests.swift`

- **`testCoreAppFunctionality()`** - Tests basic app controls:
  - Camera switching (front/back)
  - Mode switching (Continuous/Spot)
  - Measurement duration controls
  - Tab navigation (iOS 16+)

#### `demo_app_MeasurementTests.swift`

- **`testContinuousMeasurementFlow()`** - Full continuous measurement:
  - Tutorial flow handling
  - 10-second recording session
  - Results validation

- **`testSpotMeasurementFlow()`** - Complete spot measurement:
  - Duration setup (30 seconds)
  - Full measurement cycle with countdown
  - Results validation (BPM, charts, sections)
  - 60-second timeout for API response

- **`testMeasurementPreparation()`** - Setup helper for manual testing

#### `demo_app_UITestsLaunchTests.swift`

- **`testLaunchScreenshot()`** - Verifies app launches successfully

### Helper Extensions

- **`XCUIElement`** extensions for enhanced interaction
- **`XCUIApplication`** extensions for device detection and permissions

## Test Scenarios

### Continuous Mode Testing

1. Ensures app is in Continuous mode
2. Handles tutorial flow if present
3. Initiates 10-second recording
4. Validates measurement UI components
5. Captures results screenshots

### Spot Mode Testing

1. Switches to Spot mode
2. Sets 30-second duration
3. Handles tutorial and permissions
4. Runs full measurement cycle (countdown + measurement + processing)
5. Validates results (pulse, breathing, blood pressure sections)
6. Checks for BPM values and chart titles

### Tutorial Flow Handling

Tests automatically handle the onboarding tutorial:

- Detects tutorial images (tutorial_image1-7)
- Swipes through tutorial screens
- Accepts terms and conditions
- Handles camera permission dialogs

## Device Considerations

### Simulator Testing (Incomplete: Does not support all features yet)

- Faster execution
- No camera permissions required
- Limited camera simulation capabilities
- Ideal for UI flow validation

### Real Device Testing

- Full camera functionality
- Requires permission handling
- More realistic testing environment
- Longer execution times due to actual measurements

### Permission Handling

The tests include automatic handling of:

- Camera access permissions
- System alert dialogs
- Tutorial acceptance flows

## Troubleshooting

### Common Issues

**Test timeouts on real devices:**

- Spot measurements allow 60 seconds for API response
- Ensure stable network connection
- Check server endpoint availability

**Permission dialogs:**

- Tests automatically handle standard permission dialogs
- Manual intervention may be needed for first-time device setup

**Element not found errors:**

- Check accessibility identifiers in app code
- Verify UI hierarchy hasn't changed
- Use `takeScreenshot()` for debugging UI state

**Tutorial flow issues:**

- Tutorial only appears on first app launch
- Reset simulator or reinstall app to test tutorial flow
- Tutorial detection is based on specific image identifiers

### Debug Helpers

- All test classes inherit screenshot capabilities
- Screenshots are automatically captured for key test points
- Use `takeScreenshot(name:)` for custom debug points

## Adding New Tests

### Best Practices

1. Extend `UITestBase` for common functionality
2. Use descriptive test method names
3. Include timeout handling for network operations
4. Add screenshot capture for verification points
5. Handle both simulator and device scenarios

### Example New Test

```swift
@MainActor
func testNewFeature() throws {
    XCTAssertTrue(waitForAppToLoad(), "App should launch")

    // Your test logic here
    let newButton = app.buttons["New Feature"]
    XCTAssertTrue(newButton.waitAndTap(), "Should find and tap new feature")

    takeScreenshot(name: "New Feature - Result")
}
```

### Element Identification

- Prefer accessibility identifiers over text-based selectors
- Use `containing()` predicates for dynamic content
- Test both iOS versions if using iOS-specific features

## CI/CD Integration

### Xcode Cloud (TODO: not implemented yet, but can be used from public github repo)

Tests are configured to run in Xcode scheme and can be integrated with Xcode Cloud workflows.

### Fastlane Integration (TODO: not implemented yet)

The project includes Fastlane configuration (`fastlane/Fastfile`) that can be extended to include UI testing:

```ruby
lane :run_ui_tests do
  run_tests(
    project: "demo-app.xcodeproj",
    scheme: "demo-app",
    devices: ["iPhone 15"],
    only_testing: ["demo-app-UITests"]
  )
end
```

### GitHub Actions Example (TODO: not implemented yet)

```yaml
- name: Run UI Tests
  run: |
    xcodebuild test \
      -project demo-app.xcodeproj \
      -scheme demo-app \
      -destination 'platform=iOS Simulator,name=iPhone 15,OS=17.0' \
      -resultBundlePath TestResults
```

## Maintenance

### Regular Updates Needed

- Update device/OS versions in test destinations
- Verify accessibility identifiers after UI changes
- Update timeout values based on API performance
- Refresh tutorial flow handling for onboarding changes

### Performance Considerations

- Spot measurement tests take 60+ seconds
- Continuous tests complete in ~15 seconds
- Consider parallel execution for faster CI builds
- Use simulator for faster feedback loops during development

---

**Note:** These tests require camera permissions and work best on real devices for complete functionality validation. For development and quick feedback, simulator testing provides good coverage of UI flows and basic functionality.
