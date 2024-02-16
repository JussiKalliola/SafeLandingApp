//
//  ARReceiver.swift
//  DigitalMicroAirport
//
//  Created by Jussi Kalliola (TAU) on 29.9.2022.
//

import Foundation
import SwiftUI
import Combine
import ARKit
import ModelIO
import MetalKit
import MapKit
import CoreLocation


// Receive the newest AR data from an `ARReceiver`.
protocol ARDataReceiver: AnyObject {
    func onNewARData(arData: ARData)
    func onNewAnchor(arData: ARData)
}


final class ARReceiver: NSObject, ARSessionDelegate, ARSCNViewDelegate, CLLocationManagerDelegate{
    var arData = ARData()
    var arSession = ARSession()
    var config: ARWorldTrackingConfiguration = ARWorldTrackingConfiguration()
    weak var delegate: ARDataReceiver?
    
    //var sceneView: ARSCNView?
    
    var node:SCNNode = SCNNode()
    
    var tempArAnchors: [ARAnchor] = []
    
    var scanState: Bool = true
    
    let VERTEX_BUFFER_SIZE: Int = 10_000_000
    
    var now: Date?
    var locNow: CLLocationCoordinate2D?
    var originGps: CLLocationCoordinate2D? = nil
    
    var df: DateFormatter?
    
    let d1 = Date()
    
    let locationManager = CLLocationManager()
    
    var frameId = 0
    
    var prevARPointcloud: [vector_float3]?
    var currentARPointCloud: [vector_float3]?
    
    // Configure and start the ARSession.
    override init() {
        super.init()
        arSession.delegate = self
        
        guard ARWorldTrackingConfiguration.supportsFrameSemantics([.sceneDepth, .smoothedSceneDepth]) else { return }
        // Enable both the `sceneDepth` and `smoothedSceneDepth` frame semantics.
        self.config.environmentTexturing = .automatic
        self.config.planeDetection = [.horizontal, .vertical]
        self.config.isAutoFocusEnabled = true
        self.config.sceneReconstruction = .mesh
        self.config.worldAlignment = .gravity //.gravityAndHeading
        
        //self.config.videoHDRAllowed = true
        self.now = Date.now - ProcessInfo.processInfo.systemUptime
        print(self.config.worldAlignment.rawValue)
        self.df = DateFormatter()
        self.df!.dateFormat = "y-MM-dd H:mm:ss.SSSS"
        //self.config.worldAlignment = .gravity
        
        //sceneView = ARSCNView()
    
        //print(sceneView?.pointOfView?.camera?.focalLength, sceneView?.pointOfView?.camera?.sensorHeight)
        self.config.frameSemantics = [.sceneDepth, .smoothedSceneDepth, .personSegmentationWithDepth]
        
        let metalDevice = MTLCreateSystemDefaultDevice()
        CVMetalTextureCacheCreate(nil, nil, metalDevice!, nil, &arData.textureCache)
        
        
        start()
    }
    
    // Configure and start the ARKit session.
    func start() {
        arSession.run(self.config)
        
        //print(arSession.delegateQueue)
        
        // Create a vertex buffer for million points.
        let dataSize = VERTEX_BUFFER_SIZE * MemoryLayout<RenderedPointCloud>.stride
        arData.renderVertexBuffer = EnvironmentVariables.shared.metalDevice.makeBuffer(length: dataSize, options: .storageModeShared)
        
        self.locationManager.delegate = self;
        self.locationManager.desiredAccuracy = kCLLocationAccuracyBest
        self.locationManager.requestAlwaysAuthorization()
        self.locationManager.startUpdatingLocation()
        //self.locationManager.
        print(self.locationManager)
        
    }
    
    // Pause the ARKit session.
    func pause() {
        arSession.pause()
    }
    
    func locationManager(_ manager: CLLocationManager, didUpdateLocations locations: [CLLocation]) {
        //print(locations)
        guard let locValue: CLLocationCoordinate2D = manager.location?.coordinate else { return }
        //print("locations = \(locValue.latitude) \(locValue.longitude)")
        self.locNow = locValue
        
        if self.originGps == nil {
            self.originGps = locValue
            
        }
    }
       
    
    // If anchor is updated
    func session(_ session: ARSession, didUpdate anchors: [ARAnchor]) {
        if scanState {
            arData.arAnchors = session.currentFrame!.anchors
            tempArAnchors = arData.arAnchors
        }
        
        delegate?.onNewAnchor(arData: arData)
    }
    
    // If anchor is added
    func session(_ session: ARSession, didAdd anchors: [ARAnchor]) {
        
        if scanState {
            arData.arAnchors = session.currentFrame!.anchors
            tempArAnchors = arData.arAnchors
        }

        delegate?.onNewAnchor(arData: arData)
    }
    
    
    func calculateDist3D(prevPoints: [vector_float3], curPoints: [vector_float3]) {
        var distances: [Double] = []
        for i in 0..<prevPoints.count {
            let p1 = prevPoints[i]
            guard (curPoints.count > i) else {
                break
            }
            let p2 = curPoints[i]
            
            let distance = Double(sqrt( pow(p2.x - p1.x, 2) + pow(p2.y-p1.y,2) + pow(p2.z-p1.z,2)))
            distances.append(distance)
        }
        let distSum = distances.reduce(0, +)
        let distAvg = distSum / Double(distances.count)
        print("Average distance: " + String(distAvg))
        print("P1.count: " + String(prevPoints.count) + " --- P2.count: " + String(curPoints.count))
        
    }
    
    
    // Ran on every frame
    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        guard (frame.sceneDepth != nil) && (frame.smoothedSceneDepth != nil) else {
            return
        }
        
        frameId+=1
        
        //frame.displayTransform(for: .portrait, viewportSize: <#T##CGSize#>)
        
        //print(frame.geoTrackingStatus?.)
        
        arData.depthImage = frame.smoothedSceneDepth?.depthMap
        arData.colorImage = frame.capturedImage
        arData.confidenceImage = frame.smoothedSceneDepth?.confidenceMap
        arData.cameraResolution = frame.camera.imageResolution
        arData.sampleTime = df!.string(from: Date(timeInterval: frame.timestamp, since: self.now!))
        arData.worldPose = frame.camera.transform
        arData.eulerAngles = frame.camera.eulerAngles
        arData.worldToCamera = frame.camera.viewMatrix(for: .portrait)
        arData.camera = frame.camera
        arData.frame = frame
        arData.cameraIntrinsics = frame.camera.intrinsics
        arData.trackingState = frame.camera.trackingState
        arData.frameId = frameId
        node.simdTransform = frame.camera.transform
        
        self.currentARPointCloud = frame.rawFeaturePoints?.points
            

//        if arProvider.websocket != nil{
//            let translation = frame.camera.transform.columns.3
//            let quaternion = simd_quaternion(frame.camera.transform)
//            let d2 = Date()
//            let elapsed = Float(d2.timeIntervalSince(d1))
//            let floorInt = floor(elapsed)
//            //print(elapsed)
//            let nanosec = Int((elapsed-floorInt)*1e+6)
//
//            arProvider.websocket?.publish(json: ["op": "publish",
//                                                 "topic": "/iphone/pose",
//                                                 "msg": ["header": ["stamp": ["sec": floorInt, "nanosec": nanosec], "frame_id": "iphone"],
//                                                         "pose": ["position": ["x": translation.x, "y": translation.y, "z": translation.z],
//                                                                  "orientation": ["x": quaternion.imag.x, "y": quaternion.imag.y, "z": quaternion.imag.z, "w": quaternion.real]]
//                                                        ]
//                                                ])
//        }
        
        
        //print(df!.string(from: Date(timeInterval: frame.timestamp, since: self.now!)))
        //print(Date(timeInterval: frame.timestamp, since: self.now!).formatted(.dateTime))
        //print(Date(timeInterval: frame.timestamp, since: self.now!).formatted(.iso8601))
        //print("new frame")
        //print(frame.camera.viewMatrix(for: .portrait))
        //print(frame.camera.transform.inverse)
        //print(frame.camera.viewMatrix(for: .portrait).inverse)
        //print(frame.camera.transform)
        //print(frame.camera.transform.inverse)
        //print(node.simdEulerAngles * (180/Float.pi), node.simdWorldOrientation, node.simdWorldPosition)
        //print(arData.cameraResolution)
        
        
//        print(UIDevice.current.orientation.isPortrait)
//        print(frame.camera.transform)
        //print(frame.camera.transform.columns.3.x, frame.camera.transform.columns.3.y, frame.camera.transform.columns.3.z)
        //print(frame.camera.transform.columns.0, frame.camera.transform.columns.1, frame.camera.transform.columns.2)
        
        if !scanState {
            arData.arAnchors = tempArAnchors
        }
        
        delegate?.onNewARData(arData: arData)
    }
}
