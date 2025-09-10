//
//  SmartSpectraProcessor.swift
//  SmartSpectraSwiftSDK
//
//  Created by Ashraful Islam on 3/3/25.
//

import Foundation
import PresagePreprocessing
import CoreImage
import UIKit
import SwiftUI

/// Indicates the current state of the preprocessing pipeline.
public enum PresageProcessingStatus {
    case idle
    case processing
    case processed
    case error
}

/// Utility for converting ``CVPixelBuffer`` images to ``UIImage`` asynchronously.
final class ImageConverter {
    private let sharedContext = SharedCIContext.shared
    private let queue = DispatchQueue(label: "image.convert.queue", qos: .userInteractive)
    private var isProcessing = false
    private let lock = NSLock()

    /// Converts a pixel buffer to a `UIImage` off the main thread.
    /// - Parameters:
    ///   - pixelBuffer: The buffer to convert.
    ///   - completion: Closure invoked with the resulting image or `nil` on failure.
    func convertAsync(pixelBuffer: CVPixelBuffer, completion: @escaping (UIImage?) -> Void) {
        // Skip frame if already processing to prevent backlog
        lock.lock()
        if isProcessing {
            lock.unlock()
            return // Drop frame to maintain smooth UI
        }
        isProcessing = true
        lock.unlock()

        // Extract dimensions before async - these are fast, synchronous operations
        let width = CVPixelBufferGetWidth(pixelBuffer)
        let height = CVPixelBufferGetHeight(pixelBuffer)

        queue.async { [weak self] in
            guard let self = self else { return }

            defer {
                self.lock.lock()
                self.isProcessing = false
                self.lock.unlock()
            }

            autoreleasepool {
                let ciImage = CIImage(cvPixelBuffer: pixelBuffer)
                let rect = CGRect(x: 0, y: 0, width: width, height: height)

                self.sharedContext.createCGImage(ciImage, from: rect) { cgImage in
                    guard let cgImage = cgImage else {
                        completion(nil)
                        return
                    }
                    completion(UIImage(cgImage: cgImage))
                }
            }
        }
    }
}

/// Lower level API for controlling real‑time processing.
///
/// Most apps can rely on ``SmartSpectraView``, but this class provides
/// granular control for headless operation or custom UIs.
public class SmartSpectraVitalsProcessor: NSObject, ObservableObject {
    public static let shared = SmartSpectraVitalsProcessor()
    /// Live camera preview image updated in real-time during processing.
    ///
    /// This property provides access to the processed camera frames as `UIImage` objects.
    /// Images are only available when image output is enabled via ``SmartSpectraSwiftSDK/setImageOutputEnabled(_:)``.
    ///
    /// ## Usage Examples
    ///
    /// ### Display Camera Feed
    /// ```swift
    /// if let image = vitalsProcessor.imageOutput {
    ///     Image(uiImage: image)
    ///         .resizable()
    ///         .aspectRatio(contentMode: .fit)
    /// }
    /// ```
    ///
    /// ### Observe Frame Updates
    /// ```swift
    /// @State private var frameCount = 0
    ///
    /// // Observe frame updates
    /// .onReceive(vitalsProcessor.$imageOutput) { image in
    ///     if let frame = image {
    ///         frameCount += 1
    ///         // Custom frame processing
    ///         analyzeFrame(frame)
    ///     }
    /// }
    /// ```
    ///
    /// - Note: This property is `nil` when image output is disabled for performance optimization.
    /// - Tip: Use `onReceive` to react to frame updates
    @Published public var imageOutput: UIImage?
    /// Current status of the underlying `PresagePreprocessing` engine.
    @Published public var processingStatus: PresageProcessingStatus = .idle
    /// Countdown timer value for spot measurements.
    @Published public var counter: Double = 0
    /// Estimated frames per second of incoming video.
    @Published public var fps: Int = 0
    /// Latest status code emitted by the preprocessing engine.
    @Published public var lastStatusCode: StatusCode = .processingNotStarted
    /// Human readable hint string for the current status code.
    @Published public var statusHint: String = ""
    /// Indicates whether recording is currently active.
    @Published public var isRecording: Bool = false
    var presageProcessing: PresagePreprocessing = PresagePreprocessing()
    var lastTimestamp: Int?
    var fpsValues: [Int] = []
    let movingAveragePeriod = 10
    private var sdk: SmartSpectraSwiftSDK
    private var avCaptureDeviceManager: AVCaptureDeviceManager = AVCaptureDeviceManager.shared
    @State private var coreIsRunning: Bool = false // TODO: perhaps add a processing status for not ready
    private let imageConverter = ImageConverter()
    private weak var authHandler: AuthHandler?

    private override init(){
        sdk = SmartSpectraSwiftSDK.shared
        authHandler = AuthHandler.shared
        super.init()
        presageProcessing.delegate = self
    }

    internal func setRecordingState(_ state: Bool) {
        presageProcessing.buttonStateChanged(inFramework: state)
        DispatchQueue.main.async {
            self.isRecording = state
        }
    }

    internal func changeProcessingMode(_ mode: SmartSpectraMode) {
        guard presageProcessing.mode != mode.presageMode else { return }
        presageProcessing.mode = mode.presageMode
        sdk.config.smartSpectraMode = mode
    }

    internal func setProcessingCameraPosition(_ position: AVCaptureDevice.Position) {
        guard presageProcessing.cameraPosition != position else { return }
        presageProcessing.cameraPosition = position
        sdk.config.cameraPosition = position
    }

    internal func setApiKey(_ apiKey: String) {
        presageProcessing.apiKey = apiKey
        sdk.config.apiKey = apiKey
    }

    internal func setSpotDuration(_ duration: Double) {
        presageProcessing.spotDuration = duration
        sdk.config.measurementDuration = duration
    }

    private func setupProcessing() {
        let oauth_enabled  = authHandler?.isOauthEnabled ?? false

        if !oauth_enabled {
            guard let apiKey = sdk.config.apiKey, !apiKey.isEmpty else {
                print("ERROR: API key missing - cannot proceed with processing")
                DispatchQueue.main.async {
                    self.processingStatus = .error
                }
                return
            }
            setApiKey(apiKey)
        }

        changeProcessingMode(sdk.config.smartSpectraMode)
        setProcessingCameraPosition(sdk.config.cameraPosition)
        if(sdk.config.smartSpectraMode == .spot) {
            setSpotDuration(sdk.config.measurementDuration)
        }
    }

    /// Initializes processing pipelines and starts streaming frames.
    public func startProcessing() {
        guard let authHandler = authHandler else {
            print("AuthHandler is not available.")
            return
        }

        authHandler.startAuthWorkflow { [weak self] error in
            guard let self = self else { return }

            if let error = error {
                print("Authentication failed with error: \(error)")
                DispatchQueue.main.async {
                    self.processingStatus = .error
                }
                return
            }

            if self.coreIsRunning {
                self.stopProcessing()
            }
            DispatchQueue.main.async {
                self.processingStatus = .idle
            }
            DispatchQueue.global(qos: .userInitiated).async {
                self.setupProcessing()
                self.presageProcessing.start()
                DispatchQueue.main.async {
                    self.coreIsRunning = true
                }
            }
        }
    }

    /// Stops processing and cleans up resources.
    public func stopProcessing() {
        DispatchQueue.global(qos: .userInitiated).async {
            self.presageProcessing.stop()
            DispatchQueue.main.async {
                self.processingStatus = .idle
                self.imageOutput = nil
                self.coreIsRunning = false
            }
        }
    }

    /// Begins recording vitals. Call after ``startProcessing()``.
    public func startRecording() {
        UIApplication.shared.isIdleTimerDisabled = true
        avCaptureDeviceManager.lockCameraSettings()
        setRecordingState(true)
    }

    /// Stops recording vitals and unlocks camera settings.
    public func stopRecording() {
        UIApplication.shared.isIdleTimerDisabled = false
        avCaptureDeviceManager.unlockCameraSettings()
        setRecordingState(false)
    }

}

extension SmartSpectraVitalsProcessor: PresagePreprocessingDelegate {

    /// Called when a new camera frame is available for processing.
    ///
    /// When image output is disabled, this method skips image conversion to optimize performance and reduce memory pressure.
    /// When image output is enabled, frames are converted to UIImage and published via ``imageOutput`` for UI display.
    ///
    /// - Parameters:
    ///   - tracker: The preprocessing instance that captured the frame
    ///   - pixelBuffer: Raw camera frame data as CVPixelBuffer
    ///   - timestamp: Frame timestamp in milliseconds
    public func frameWillUpdate(_ tracker: PresagePreprocessing!, didOutputPixelBuffer pixelBuffer: CVPixelBuffer!, timestamp: Int) {
        // Skip UI image conversion when image output is disabled to optimize performance and reduce memory usage
        guard sdk.config.imageOutputEnabled else { return }

        // Convert the pixel buffer to UIImage asynchronously and publish it for UI display

        //TODO: Need better approach here: Conversion to UIImage is not very efficient, and causes a lot of memory pressure.
        imageConverter.convertAsync(pixelBuffer: pixelBuffer) { image in
            if let image = image {
                DispatchQueue.main.async {
                    self.imageOutput = image
                }
            }
        }

    }

    public func frameDidUpdate(_ tracker: PresagePreprocessing!, didOutputPixelBuffer pixelBuffer: CVPixelBuffer!) {
        // TODO: figure out if we need to keep this or remove it
    }

    // Status delivered as serialized StatusValue protobuf for Swift consumers.
    public func statusBufferChanged(_ tracker: PresagePreprocessing!, serializedBytes: Data) {
        do {
            let status = try Presage_Physiology_StatusValue(serializedBytes: serializedBytes)
            let code: StatusCode = status.value
            if code != lastStatusCode {
                DispatchQueue.main.async {
                    self.lastStatusCode = code
                    self.statusHint = tracker.getStatusHint(fromCodeValue: Int(code.rawValue))
                }
            }
            if sdk.config.smartSpectraMode == .spot && sdk.config.showFps {
                // update fps based on status code in spot mode
                updateFps()
            }
        } catch {
            print("Failed to deserialize StatusValue: \(error.localizedDescription)")
        }
    }

    /// Delegate callback providing processed metrics.
    public func metricsBufferChanged(_ tracker: PresagePreprocessing!, serializedBytes: Data) {
        do {
            // Deserialize the data directly into the Swift Protobuf object
            let metricsBuffer = try MetricsBuffer(serializedBytes: serializedBytes)
            // print("Received metrics buffer. metadata: \(String(describing: metricsBuffer.metadata))")
            //            print("Pulse: \(String(describing: metricsBuffer.pulse.rate.last?.value)), Breathing: \(String(describing: metricsBuffer.breathing.rate.last?.value))")
            // update metrics buffer
            if sdk.config.smartSpectraMode == .spot {
                DispatchQueue.main.async {
                    self.processingStatus = .processed
                }
            }

            DispatchQueue.main.async {
                self.sdk.metricsBuffer = metricsBuffer
            }

//            if sdk.config.smartSpectraMode == .continuous && sdk.config.showFps {
//                //update fps based on metricsBuffer in continuous mode
//                updateFps()
//            }

        } catch {
            print("Failed to deserialize MetricsBuffer: \(error.localizedDescription)")
        }
    }


    /// Delegate callback providing real-time edge metrics for continuous mode.
    public func edgeMetricsChanged(_ tracker: PresagePreprocessing!, serializedBytes: Data) {
        do {
            // Deserialize the data directly into the Swift Protobuf object
            let edgeMetrics = try Metrics(serializedBytes: serializedBytes)
            // print("Received metrics buffer. metadata: \(String(describing: metricsBuffer.metadata))")
            //            print("Pulse: \(String(describing: metricsBuffer.pulse.rate.last?.value)), Breathing: \(String(describing: metricsBuffer.breathing.rate.last?.value))")
            // update metrics buffer
            if sdk.config.smartSpectraMode == .spot {
                DispatchQueue.main.async {
                    self.processingStatus = .processed
                }
            }

            DispatchQueue.main.async {
                self.sdk.edgeMetrics = edgeMetrics
            }

            if sdk.config.smartSpectraMode == .continuous && sdk.config.showFps {
                //update fps based on edgeMetrics in continuous mode
                updateFps()
            }

        } catch {
            print("Failed to deserialize Metrics: \(error.localizedDescription)")
        }
    }

    /// Delegate callback providing countdown updates in seconds.
    public func timerChanged(_ timerValue: Double) {
        if counter != timerValue {
            DispatchQueue.main.async {
                self.counter = timerValue
                if self.counter == 0.0 && self.processingStatus == .idle {
                    self.processingStatus = .processing
                }
            }
        }
    }


    /// Called when the preprocessing graph reports an unrecoverable error.
    public func handleGraphError(_ error: Error) {
        print("Error in vital processing: \(error)")
        self.sdk.updateErrorText("Internal error occurred. Check your internet connection and retry. If it happens repeatedly contact customer support.")
        //clear metrics buffer
        DispatchQueue.main.async {
            self.processingStatus = .error
            self.sdk.metricsBuffer = nil
        }
    }

    /// Calculates a moving-average FPS value based on delegate callbacks.
    fileprivate func updateFps() {
        let currentTime = Int(Date().timeIntervalSince1970 * 1000)

        if let lastTimestamp = lastTimestamp {
            let deltaTime = currentTime - lastTimestamp

            fpsValues.append(deltaTime)
            if fpsValues.count > movingAveragePeriod {
                fpsValues.removeFirst()
            }
            // TODO: 10/28/24: Fix this further upstream so this isn't necessary
            let averageDeltaTime = max(Double(fpsValues.reduce(0, +)) / Double(max(fpsValues.count, 1)), 0.0001)

            DispatchQueue.main.async {
                self.fps = Int(round(1000 / averageDeltaTime))
            }
        }
        lastTimestamp = currentTime
    }
}
