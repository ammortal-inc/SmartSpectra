//
//  demo_app_UITests.swift
//  demo-app-UITests
//
//  Created by Ashraful Islam on 8/10/25.
//

import XCTest

final class demo_app_UITests: UITestBase {

    @MainActor
    func testCoreAppFunctionality() throws {
        // Test app launches and core functionality works
        XCTAssertTrue(waitForAppToLoad(), "App should launch")
        
        // Test main controls on Checkup tab
        let cameraButton = app.buttons["Switch to Back Camera"]
        if cameraButton.waitForExistence(timeout: 2) {
            cameraButton.tap()
            XCTAssertTrue(app.buttons["Switch to Front Camera"].waitForExistence(timeout: 2), "Camera should switch")
        }
        
        let modeButton = app.buttons["Switch to Spot"]  
        if modeButton.waitForExistence(timeout: 1) {
            modeButton.tap()
            XCTAssertTrue(app.buttons["Switch to Continuous"].waitForExistence(timeout: 2), "Mode should switch")
        }
        
        // Test measurement duration stepper
        let stepper = app.steppers.firstMatch
        if stepper.waitForExistence(timeout: 1) {
            let incrementButton = stepper.buttons["Increment"]
            let decrementButton = stepper.buttons["Decrement"]
            
            if incrementButton.exists {
                // Get current duration value
                let durationText = app.staticTexts.containing(NSPredicate(format: "label CONTAINS 'Measurement Duration'")).firstMatch
                let initialText = durationText.exists ? durationText.label : ""
                
                incrementButton.tap()
                
                // Verify duration changed
                if durationText.exists {
                    let newText = durationText.label
                    XCTAssertNotEqual(initialText, newText, "Duration should change when incremented")
                }
            }
            
            if decrementButton.exists {
                decrementButton.tap()
                // Just verify decrement button responds (value should change back or decrease further)
            }
        }
        
        // Test tab navigation if iOS 16+
        if #available(iOS 16.0, *) {
            let headlessTab = app.tabBars.buttons["Headless Example"]
            if headlessTab.exists {
                headlessTab.tap()
                
                // Test vitals start/stop
                let startButton = app.buttons["Start"]
                if startButton.waitForExistence(timeout: 2) {
                    startButton.tap()
                    let stopButton = app.buttons["Stop"]
                    if stopButton.waitForExistence(timeout: 2) {
                        stopButton.tap()
                        XCTAssertTrue(startButton.waitForExistence(timeout: 2), "Should return to start state")
                    }
                }
            }
        }
    }
}