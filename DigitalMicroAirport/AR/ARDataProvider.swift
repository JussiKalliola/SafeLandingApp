//
//  ARDataProvider.swift
//  DigitalMicroAirport
//
//  Created by Jussi Kalliola (TAU) on 29.9.2022.
//

import Foundation
import SwiftUI
import Combine
import ARKit
import Accelerate
import MetalPerformanceShaders
import MetalKit

// Wrap the `MTLTexture` protocol to reference outputs from ARKit.
final class MetalTextureContent {
    var texture: MTLTexture?
}

// Collect AR data using a lower-level receiver. This class converts AR data
// to a Metal texture, optionally upscaling depth data using a guided filter,
// and implements `ARDataReceiver` to respond to `onNewARData` events.
final class ARProvider: ARDataReceiver, ObservableObject {
    
    static let shared: ARProvider = {
        let instance = ARProvider()
        return instance!
    }()
    
    let metalDevice: MTLDevice?
    let commandQueue: MTLCommandQueue!
    
    // Set the destination resolution for the upscaled algorithm.
    let downscaledWidth = 256
    let downscaledHeight = 192
    
    // Set the destination resolution for the upscaled algorithm.
    let upscaledWidth = 256
    let upscaledHeight = 192

    // Set the original depth size.
    let origDepthWidth = 256
    let origDepthHeight = 192

    // Set the original color size.
    let origColorWidth = 1920
    let origColorHeight = 1440
    
    // Set the guided filter constants.
    let guidedFilterEpsilon: Float = 0.004
    let guidedFilterKernelDiameter = 5
    
    // Guided filter vars
    let guidedFilter: MPSImageGuidedFilter?
    let mpsScaleFilter: MPSImageBilinearScale?
    
    // ARKit data, from ARReceiver
    let arReceiver = ARReceiver()
    @Published var lastArData: ARData?
    
    // Upscaled texture inits
    let coefTexture: MTLTexture
    let upscaledDepthTexture: MTLTexture
    let upscaledConfTexture: MTLTexture
    let downscaledRGBTexture: MTLTexture
    let RGBTexture: MTLTexture
    let depthRGBATexture: MTLTexture
    let confRGBATexture: MTLTexture
    
    var scanState: Bool = false
    var timer = Timer()
    let fileManager: FileManagerHelper?
    var fileCounter: Int = 0
    var confSelection: Int = 0
    
    
    var computePipelineState: MTLComputePipelineState!
    var vertexPipeline: MTLComputePipelineState!
    
    var pointUniformBufferIndex: Int = 0
    var pointUniformBuffer: MTLBuffer!
    var pointUniformBufferOffset: Int = 0
    var pointUniformBufferAddress: UnsafeMutableRawPointer!
    
    var curvatureBuffer: MTLBuffer!
    
    let queue = OperationQueue() //DispatchQueue(label: "com.KDTree", qos: .default)
    let globalFilterQueue = DispatchQueue(label: "com.GlobalVoxelFilter", qos: .userInitiated)
    let lock = NSLock()
    
    var voxelFilterLeafSize: Double = 0.01
    var rgbNames: [String] = []
    
    var useICP: Int = 1
    
    var savedFrames: [SavedFrame] = []
    var savedARPointclouds: [(vector_float3, vector_float3)] = []
    
    var useRos: Bool = false
    var rosIpAddress:String = ""
    
    var websocket: WebSocket?
    
    // Create an empty texture.
    static func createTexture(metalDevice: MTLDevice, width: Int, height: Int, usage: MTLTextureUsage, pixelFormat: MTLPixelFormat) -> MTLTexture {
        let descriptor: MTLTextureDescriptor = MTLTextureDescriptor()
        descriptor.pixelFormat = pixelFormat
        descriptor.width = width
        descriptor.height = height
        descriptor.usage = usage
        let resTexture = metalDevice.makeTexture(descriptor: descriptor)
        return resTexture!
    }
    
    
    init?() {
        do {
            metalDevice = EnvironmentVariables.shared.metalDevice
            
            guidedFilter = MPSImageGuidedFilter(device: metalDevice!, kernelDiameter: guidedFilterKernelDiameter)
            guidedFilter?.epsilon = guidedFilterEpsilon
            mpsScaleFilter = MPSImageBilinearScale(device: metalDevice!)
            
            commandQueue = EnvironmentVariables.shared.metalCommandQueue
            
            // Create upscaled empty textures
            upscaledDepthTexture = ARProvider.createTexture(metalDevice: metalDevice!,
                                                            width: origDepthWidth,
                                                            height: origDepthHeight,
                                                            usage: [.shaderRead, .shaderWrite],
                                                            pixelFormat: .r32Float)
            
            upscaledConfTexture = ARProvider.createTexture(metalDevice: metalDevice!,
                                                           width: origDepthWidth,
                                                           height: origDepthHeight,
                                                           usage: [.shaderRead, .shaderWrite],
                                                           pixelFormat: .r8Unorm)
            
            coefTexture = ARProvider.createTexture(metalDevice: metalDevice!,
                                                   width: upscaledWidth,
                                                   height: upscaledHeight,
                                                   usage: [.shaderRead, .shaderWrite],
                                                   pixelFormat: .r32Float)
            
            downscaledRGBTexture = ARProvider.createTexture(metalDevice: metalDevice!,
                                                           width: origDepthWidth,
                                                           height: origDepthHeight,
                                                           usage: [.shaderRead, .shaderWrite],
                                                           pixelFormat: .rgba8Unorm)
            
            RGBTexture = ARProvider.createTexture(metalDevice: metalDevice!,
                                                  width: origColorWidth,
                                                  height: origColorHeight,
                                                  usage: [.shaderRead, .shaderWrite],
                                                  pixelFormat: .rgba8Unorm)
            
            confRGBATexture = ARProvider.createTexture(metalDevice: metalDevice!,
                                                       width: origDepthWidth,
                                                       height: origDepthHeight,
                                                       usage: [.shaderRead, .shaderWrite],
                                                       pixelFormat: .rgba8Unorm)
            
            depthRGBATexture = ARProvider.createTexture(metalDevice: metalDevice!,
                                                        width: origDepthWidth,
                                                        height: origDepthHeight,
                                                        usage: [.shaderRead, .shaderWrite],
                                                        pixelFormat: .rgba8Unorm)
            
            fileManager = FileManagerHelper(suffix: "test", useRos: self.useRos)
            
            
            let pointUniformBufferSize = MemoryLayout<CameraIntrinsics>.stride

            pointUniformBuffer = EnvironmentVariables.shared.metalDevice.makeBuffer(length: pointUniformBufferSize, options: .storageModeShared)
            
            
            curvatureBuffer = metalDevice!.makeBuffer(length: MemoryLayout<CurvatureNormalization>.stride, options: .storageModeShared)
            
            // Setup compute pipeline
            if let computeFunction = EnvironmentVariables.shared.metalLibrary.makeFunction(name: "YUVColorConversion") {
                do {
                    computePipelineState = try metalDevice?.makeComputePipelineState(function: computeFunction)
                } catch let error as NSError {
                    fatalError("Error: \(error.localizedDescription)")
                }
            } else {
                fatalError("Kernel function not found at runtime.")
            }
            
            queue.qualityOfService = .utility
            //self.websocket = WebSocket()
                        
            // Set the delegate for ARKit callbacks.
            arReceiver.delegate = self
            
        } catch {
            print("Unexpected error: \(error).")
            return nil
        }
    }
    
    // Start or resume the stream from ARKit.
    func start() {
        arReceiver.start()
    }
    
    // Pause the stream from ARKit.
    func pause() {
        arReceiver.pause()
    }
    
    func initialize() {
        lastArData?.finalPointCloud = []
        self.fileCounter = 0
    }
    
    func startTimer() {
        self.timer = Timer.scheduledTimer(withTimeInterval: 2, repeats: true, block: { _ in
            
            self.captureFrameData(filteringPhase: 1, projectDepthMap: false)

        })
    }
    
    /// Collect the following data from the frame: Confidence map, depthmap, rgb image, camera pose, and pointcloud.
    func captureFrameData(filteringPhase: Int, projectDepthMap: Bool, currentPointCentroid: vector_float3 = vector_float3()) {
        let camPosition = vector_float3((lastArData?.frame?.camera.transform.columns.3.x)!, (lastArData?.frame?.camera.transform.columns.3.y)!, (lastArData?.frame?.camera.transform.columns.3.z)!)
        
        if currentPointCentroid.x != 0 && currentPointCentroid.y != 0 && currentPointCentroid.z != 0 {
            self.savedARPointclouds.append((currentPointCentroid, camPosition))
        } else {
            let curPoint = calculatePointcloudMean(points: lastArData?.frame?.rawFeaturePoints?.points ?? [vector_float3()])
            if curPoint.x != 0 && curPoint.y != 0 && curPoint.z != 0 {
                self.savedARPointclouds.append((curPoint, camPosition))
            }
        }
        
        arReceiver.scanState = true

        guard let cmdBuffer = EnvironmentVariables.shared.metalCommandQueue.makeCommandBuffer() else { return }
        
        self.mpsScaleFilter?.encode(commandBuffer: cmdBuffer, sourceTexture: self.lastArData!.colorRGBTexture.texture! ,
                                    destinationTexture: self.downscaledRGBTexture)
        
        cmdBuffer.commit()
        
        self.lastArData!.downscaledRGBTexture.texture = self.downscaledRGBTexture
        
        //self.fileManager!.writeImage(metalTexture: self.lastArData!.downscaledRGBTexture.texture,
        //                             imageIdx: self.fileCounter,
        //                             imageIdentifier: "downscaledImage")
        
        
        //let depthPath = self.fileManager!.writeImage(metalTexture: self.lastArData!.depthRGBATexture.texture,
        //                             imageIdx: self.fileCounter,
        //                             imageIdentifier: "depth")
        
        let confPathBin = self.fileManager!.writeConfidence(pixelBuffer: self.lastArData!.confidenceImage!, imageIdx: self.fileCounter)
        
        let depthPathBin = self.fileManager!.writeDepth(pixelBuffer: self.lastArData!.depthImage!, imageIdx: self.fileCounter)
        
        let rgbPath = self.fileManager!.writeImage(metalTexture: self.lastArData!.colorRGBTexture.texture,
                                     imageIdx: self.fileCounter,
                                     imageIdentifier: "image")
        
        //let confPath = self.fileManager!.writeImage(metalTexture: self.lastArData!.confRGBATexture.texture,
        //                             imageIdx: self.fileCounter,
        //                             imageIdentifier: "confidence")
        
        
        let curCamPose = self.getCameraPose()
        
        let curOrientation = UIDevice.current.orientation
        
        let savedFrameData = SavedFrame(pose: curCamPose,
                                        rgbPath: rgbPath!,
                                        rgbResolution: [self.lastArData!.colorRGBTexture.texture!.height,
                                                        self.lastArData!.colorRGBTexture.texture!.width],
                                        //depthPath: depthPath!,
                                        depthPathBin: depthPathBin!,
                                        depthResolution: [self.lastArData!.depthImageTexture.texture!.height,
                                                          self.lastArData!.depthImageTexture.texture!.width],
                                        //confPath: confPath!,
                                        confPathBin: confPathBin!,
                                        confResolution: [self.lastArData!.confidenceImageTexture.texture!.height,
                                                         self.lastArData!.confidenceImageTexture.texture!.width],
                                        orientation: curOrientation)
        
        self.savedFrames.append(savedFrameData)
        
        self.lastArData?.cameraPoses.append(curCamPose)
        self.lastArData?.sampleTimes.append(self.lastArData!.sampleTime!)
        
        if projectDepthMap {
            self.depthMapToPointcloud(filteringPhase: 1)
        }
        
        self.fileCounter += 1
        
    }
    
    func stopTimer() {
        arReceiver.scanState = scanState
        self.timer.invalidate()
    }
    
    func calculateDist3D(curPoint: vector_float3, camPos: vector_float3) -> Bool {
        var distances: [Double] = []
        
        if savedARPointclouds.count == 0 {
            print("No points in the Array, append..")
            
            return true
        }
        
        for (prevPoint, prevCamPos) in savedARPointclouds {
            
            
            let centroidDist = Double(sqrt( pow(curPoint.x - prevPoint.x, 2) + pow(curPoint.y-prevPoint.y,2) + pow(curPoint.z-prevPoint.z,2)))
            let camDist = Double(sqrt( pow(camPos.x - prevCamPos.x, 2) + pow(camPos.y-prevCamPos.y,2) + pow(camPos.z-prevCamPos.z,2)))
            if centroidDist < 1.5 && camDist < 1 {
                print()
                print("Distance " + String(centroidDist) + " < 1 and CamDist " + String(camDist) + " < 1, so the points exists. Continue.")
                print(prevPoint, curPoint)
                print()
                return false
            }
        }
        
        print("Points saved and current points are added to the Array.")

        return true

        
    }
    
    func calculatePointcloudMean(points: [vector_float3]) -> vector_float3 {
        var sumX: Float = 0
        var sumY: Float = 0
        var sumZ: Float = 0
        
        
        for p in points {
            sumX += p.x
            sumY += p.y
            sumZ += p.z
        }
        
        let result = vector_float3(x: sumX / Float(points.count), y: sumY / Float(points.count), z: sumZ / Float(points.count))
            //.reduce(0, +)
        return result
    }
    
    // Stream data from ARReceiver.
    func onNewARData(arData: ARData) {
        lastArData = arData
        processLastArData()
        
        updateWorldMeshAnchors(lastArData!.arAnchors)
        
        // Test for automatically capture frames based on the view
        //if (arData.frameId % 5 == 0 && arReceiver.currentARPointCloud != nil) {
        //    let currentPointCentroid = calculatePointcloudMean(points: arReceiver.currentARPointCloud!)
        //    let camPosition = vector_float3((lastArData?.frame?.camera.transform.columns.3.x)!, (lastArData?.frame?.camera.transform.columns.3.y)!, (lastArData?.frame?.camera.transform.columns.3.z)!)
        //
        //    if self.calculateDist3D(curPoint: currentPointCentroid, camPos: camPosition) {
        //        self.captureFrameData(filteringPhase: 0, projectDepthMap: true, currentPointCentroid: currentPointCentroid)
        //    }
        //}
    }
    
    // Stream data from ARReceiver.
    func onNewAnchor(arData: ARData) {
        
        lastArData?.arAnchors = arData.arAnchors
        processLastArData()
        
        updateWorldMeshAnchors(lastArData!.arAnchors)
    }
    
    func getCameraPose() -> CameraPose {
        var camPos = CameraPose()
        
        camPos.eulerAngles = lastArData!.eulerAngles
        camPos.worldPose = lastArData!.worldPose
        camPos.intrinsics = lastArData!.cameraIntrinsics
        camPos.worldToCamera = lastArData!.worldToCamera
        camPos.translation = simd_float3(lastArData!.worldPose.columns.3.x, lastArData!.worldPose.columns.3.y, lastArData!.worldPose.columns.3.z)
        camPos.numberOfPoints = 0
        camPos.gpsLocation = self.arReceiver.locNow
        print(self.arReceiver.locNow)
        
        return camPos
    }
    
    func updateBufferStates() {
        pointUniformBufferOffset = MemoryLayout<CameraIntrinsics>.stride
        
        pointUniformBufferAddress = pointUniformBuffer.contents()
    }
    
    func calcRotation() -> simd_float4x4 {
        let roll = lastArData!.eulerAngles.x
        let pitch = lastArData!.eulerAngles.y
        var yaw = lastArData!.eulerAngles.z
        
        let yawDeg = yaw * 180.0 / .pi
        let rollDeg = roll * 180.0 / .pi
        
        let rollRotMat = simd_float3x3([[1, 0, 0], [0, cos(roll), sin(roll)], [0, -sin(roll), cos(roll)]])
        let pitchRotMat = simd_float3x3([[cos(pitch), 0, -sin(pitch)], [0, 1, 0], [sin(pitch), 0, cos(pitch)]])
        let yawRotMat = simd_float3x3([[cos(yaw), sin(yaw), 0], [-sin(yaw), cos(yaw), 0], [0,0,1]])
        
        let rotationMatrix = yawRotMat * pitchRotMat * rollRotMat
        
        let translation = simd_float4(lastArData!.worldPose.columns.3);
        
        var Rt = simd_float4x4()
        
        Rt.columns.0 = simd_float4(rotationMatrix.columns.0, 0.0)
        Rt.columns.1 = simd_float4(rotationMatrix.columns.1, 0.0)
        Rt.columns.2 = simd_float4(rotationMatrix.columns.2, 0.0)
        Rt.columns.3 = simd_float4(1.0, 1.0, 1.0, 1.0)
        
        let yawRotMat2 = simd_float3x3([[cos((-yawDeg)), sin((-yawDeg)), 0], [-sin((-yawDeg)), cos((-yawDeg)), 0], [0,0,1]])
        let rollRotMat2 = simd_float3x3([[1, 0, 0], [0, cos((-1*rollDeg)), sin((-1*rollDeg))], [0, -sin((-1*rollDeg)), cos((-1*rollDeg))]])
        let rotMat = simd_float3x3(simd_float3(lastArData!.worldPose.columns.0.x, lastArData!.worldPose.columns.0.y, lastArData!.worldPose.columns.0.z), simd_float3(lastArData!.worldPose.columns.1.x, lastArData!.worldPose.columns.1.y, lastArData!.worldPose.columns.1.z), simd_float3(lastArData!.worldPose.columns.2.x, lastArData!.worldPose.columns.2.y, lastArData!.worldPose.columns.2.z))
        
        let newRotMat = rotMat*yawRotMat2
        
        Rt.columns.0 = simd_float4(newRotMat.columns.0, 0.0)
        Rt.columns.1 = simd_float4(newRotMat.columns.1, 0.0)
        Rt.columns.2 = simd_float4(newRotMat.columns.2, 0.0)
        Rt.columns.3 = translation

        return Rt

    }
    
    func updateFrameUniforms() {
        //let Rt = calcRotation()
        
        var cameraIntrinsics = lastArData!.cameraIntrinsics
        
        let depthResolution = simd_float2(x: Float(lastArData!.depthImageTexture.texture!.width), y: Float(lastArData!.depthImageTexture.texture!.height))
        let cameraResolution = simd_float2(x: Float( lastArData!.cameraResolution.width),
                                           y: Float(lastArData!.cameraResolution.height))
        let scaleRes = simd_float2(x: Float( lastArData!.cameraResolution.width) / depthResolution.x,
                                                y: Float(lastArData!.cameraResolution.height) / depthResolution.y )
        cameraIntrinsics[0][0] /= scaleRes.x
        cameraIntrinsics[1][1] /= scaleRes.y

        cameraIntrinsics[2][0] /= scaleRes.x
        cameraIntrinsics[2][1] /= scaleRes.y
        
        let flipYZ = matrix_float4x4(
            [1, 0, 0, 0],
            [0, -1, 0, 0],
            [0, 0, -1, 0],
            [0, 0, 0, 1] )

        

        var invCameraIntrinsics = cameraIntrinsics.inverse
        
        let uniforms = pointUniformBufferAddress.assumingMemoryBound(to: CameraIntrinsics.self)
        
        uniforms.pointee.cameraIntrinsics = invCameraIntrinsics
        uniforms.pointee.worldPose = lastArData!.worldPose * flipYZ // lastArData!.worldPose * flipYZ
        
    }
    
    
    /// Convert the depthmap to pointcloud
    func depthMapToPointcloud(filteringPhase: Int) {
        guard lastArData!.depthImageTexture.texture != nil, lastArData!.cameraIntrinsics != simd_float3x3(), lastArData!.worldPose != simd_float4x4() else { return }
        
        let depthWidth = lastArData!.depthImageTexture.texture!.width
        let depthHeight = lastArData!.depthImageTexture.texture!.height
        
        let dataSize = depthWidth * depthHeight * MemoryLayout<PointCloud>.stride
        lastArData?.vertexBuffer = metalDevice?.makeBuffer(length: dataSize, options: .storageModeShared)
        lastArData?.cameraFrameVertexBuffer = metalDevice?.makeBuffer(length: dataSize, options: .storageModeShared)
        
        
        if let computeFunction = EnvironmentVariables.shared.metalLibrary.makeFunction(name: "pointCloudVertexShader") {
            do {
                computePipelineState = try metalDevice?.makeComputePipelineState(function: computeFunction)
            } catch let error as NSError {
                fatalError("Error: \(error.localizedDescription)")
            }
        } else {
            fatalError("Kernel function not found at runtime.")
        }
        
        guard let cmdBuffer = EnvironmentVariables.shared.metalCommandQueue.makeCommandBuffer() else { return }
        guard let computeEncoder = cmdBuffer.makeComputeCommandEncoder() else { return }
        
        updateBufferStates()
        updateFrameUniforms()
        
        computeEncoder.setComputePipelineState(computePipelineState)
        
        computeEncoder.setTexture(lastArData?.depthImageTexture.texture, index: 0)
        computeEncoder.setTexture(lastArData?.confidenceImageTexture.texture, index: 1)
        computeEncoder.setTexture(lastArData!.colorRGBTexture.texture, index: 2)
        computeEncoder.setBuffer(pointUniformBuffer, offset: 0, index: 0)
        computeEncoder.setBytes(&confSelection, length: MemoryLayout<Int>.stride, index: 1)
        computeEncoder.setBuffer(lastArData?.vertexBuffer, offset: 0, index: 2)
        computeEncoder.setBuffer(lastArData?.cameraFrameVertexBuffer, offset: 0, index: 3)
        
        let threadgroupSize = MTLSizeMake(computePipelineState!.threadExecutionWidth,
                                    computePipelineState!.maxTotalThreadsPerThreadgroup / computePipelineState!.threadExecutionWidth, 1)
        
        let threadgroupCount = MTLSize(width: Int(ceil(Float(lastArData!.depthImageTexture.texture!.width) / Float(threadgroupSize.width))),
                                       height: Int(ceil(Float(lastArData!.depthImageTexture.texture!.height) / Float(threadgroupSize.height))),
                                       depth: 1)
        computeEncoder.dispatchThreadgroups(threadgroupCount, threadsPerThreadgroup: threadgroupSize)
        
        computeEncoder.endEncoding()

        cmdBuffer.addCompletedHandler({ [self] _ in
            //fileManager?.writeFramePointcloud(pointcloudBuffer: lastArData!.cameraFrameVertexBuffer, imageIdx: self.fileCounter)
            let rawPointer = lastArData?.vertexBuffer.contents()
            let typedPointer = rawPointer?.bindMemory(to: PointCloud.self, capacity: dataSize)
            let bufferedPointer = UnsafeBufferPointer(start: typedPointer, count: dataSize)
        
                        

            var tempPointCloud: [PointCloud] = []
            var rosPc: Array<UInt8> = []

            for i in 0..<(depthWidth * depthHeight) {
                let pos = bufferedPointer[i].position
                let color = bufferedPointer[i].color
                let classColor = bufferedPointer[i].classColor
                let classIdx = bufferedPointer[i].classIdx
                let terrainComplexity = bufferedPointer[i].terrainComplexity
                let regionColor = bufferedPointer[i].regionColor
                let region = bufferedPointer[i].region
                let texCoords = bufferedPointer[i].texCoords
                let normal = bufferedPointer[i].normal
                let curvature = bufferedPointer[i].curvature
                let depth = bufferedPointer[i].depth

                // Filter points which are closer than 1cm or further than 2.5m
                if abs(depth) > 0.01 && abs(depth) < 4.0 {
                    rosPc.append(contentsOf:pos.x.toUint8Array(maxRange: 1000.0))
                    rosPc.append(contentsOf:pos.y.toUint8Array(maxRange: 1000.0))
                    rosPc.append(contentsOf:pos.z.toUint8Array(maxRange: 1000.0))
                    rosPc.append(contentsOf:color.x.toUint8Array(maxRange: 1000.0))
                    rosPc.append(contentsOf:color.y.toUint8Array(maxRange: 1000.0))
                    rosPc.append(contentsOf:color.z.toUint8Array(maxRange: 1000.0))
                    
                    
                    tempPointCloud += [PointCloud(position: pos,
                                                   color: color,
                                                   classColor: classColor,
                                                   classIdx: classIdx,
                                                   terrainComplexity: terrainComplexity,
                                                   regionColor: regionColor,
                                                   region: region,
                                                   texCoords: texCoords,
                                                   normal: normal,
                                                   curvature: curvature,
                                                   depth: depth)]
                }

            }
            

            
            
            publishPointcloudData(depthWidth: tempPointCloud.count, rosPc: rosPc)
            
            // Perform in another thread so that the arkit is not affected.
            queue.addOperation {
                let translation = self.lastArData!.cameraPoses.last!.worldPose.columns.3
                let viewpoint = SIMD3<Float>(x: translation.x, y: translation.y, z: translation.z)
                
                var pointNormalCloud = computePointCloudNormals(pointCloud: tempPointCloud, viewpoint: viewpoint, icp: useICP)
                
                addToRenderVertexBuffer(pointNormalCloud)
            }
        })
        cmdBuffer.commit()
        
    }
    
    func publishPointcloudData(depthWidth: Int, rosPc: Array<UInt8>) -> Void {
        
        let depthWidth: Int = depthWidth
        let depthHeight: Int = 1
        let rosData = [UInt8](rosPc)
        let data = Data(bytes: rosData, count: depthWidth * MemoryLayout<Float>.stride * 6)
        let base64String = data.base64EncodedString()
        
        //let imageData:NSData = try NSData(data: data)//NSData(bytes: src, length: imageByteCount)
        //let strBase64:String = imageData.base64EncodedString(options: .lineLength64Characters)
        
        let fields = [["name": "x", "offset": 0, "datatype": 7, "count": 1],
                     ["name": "y", "offset": MemoryLayout<Float>.stride, "datatype": 7, "count": 1],
                     ["name": "z", "offset": MemoryLayout<Float>.stride*2, "datatype": 7, "count": 1],
                     ["name": "r", "offset": MemoryLayout<Float>.stride*3, "datatype": 7, "count": 1],
                     ["name": "g", "offset": MemoryLayout<Float>.stride*4, "datatype": 7, "count": 1],
                     ["name": "b", "offset": MemoryLayout<Float>.stride*5, "datatype": 7, "count": 1]
                     ]
        
        let d2 = Date()
        let elapsed = Float(d2.timeIntervalSince(arReceiver.d1))
        let floorInt = floor(elapsed)
        print(elapsed)
        let nanosec = Int((elapsed-floorInt)*1e+6)
        
        //arProvider.websocket?.publish(json: ["op": "publish",
        //                                     "topic": "/iphone/lidar",
        //                                     "msg": ["header": ["stamp": ["sec": Int(floorInt), "nanosec": nanosec], "frame_id": "iphone"],
        //                                             "height": 1,
        //                                             "width": depthWidth,
        //                                             "fields": fields,
        //                                             "is_bigendian": false,
        //                                             "point_step": MemoryLayout<Float>.stride * 6,
        //                                             "row_step": MemoryLayout<Float>.stride * 6 * depthWidth * depthHeight,
        //                                             "data": base64String,
        //                                             "is_dense": true]
        //                                    ])
        
    }
    
    func resetArData() -> Void {
        lastArData!.originalPointCloud = []
        lastArData!.finalPointCloud = []
        lastArData!.processedPointCloud = []
        lastArData!.sampleTimes = []
        lastArData!.cameraPoses = []
        lastArData!.arAnchors = []
        lastArData!.worldMeshes = []

        let dataSize = arReceiver.VERTEX_BUFFER_SIZE * MemoryLayout<RenderedPointCloud>.stride
        lastArData!.renderVertexBuffer = metalDevice!.makeBuffer(length: dataSize, options: .storageModeShared)

        CVMetalTextureCacheCreate(nil, nil, metalDevice!, nil, &lastArData!.textureCache)
    }
    
    /// Init the pointcloud and vertexBuffer with new data
    func initRenderVertexBuffer(pointCloud: [PointCloud]) {
        print("Initializing vertex buffer.")
        
        print("max curvature.")
        let maxCurvature = pointCloud.max { $0.curvature < $1.curvature }!.curvature
        print("min curvature.")
        let minCurvature = pointCloud.min { $0.curvature < $1.curvature }!.curvature
        
        print("deallocate curvature buffer.")
        curvatureBuffer = nil
        print("allocate new curvature buffer.")
        curvatureBuffer = metalDevice!.makeBuffer(length: MemoryLayout<CurvatureNormalization>.stride, options: .storageModeShared)
        print("put data into curvature buffer.")
        var curPtr = curvatureBuffer!.contents().assumingMemoryBound(to: CurvatureNormalization.self).advanced(by: 0)
        curPtr.pointee.self = CurvatureNormalization(minCurvature: minCurvature ?? 0, maxCurvature: maxCurvature ?? 0)
        
        print("deallocate vertex buffer.")
        lastArData!.renderVertexBuffer = nil
        print("allocate new vertex buffer.")
        
        let dataSize = arReceiver.VERTEX_BUFFER_SIZE * MemoryLayout<RenderedPointCloud>.stride
        lastArData!.renderVertexBuffer = metalDevice!.makeBuffer(length: dataSize, options: .storageModeShared)
        
        let startIdx: Int = 0
        let endIdx: Int = pointCloud.count //filteredPointCloud.count
        
        print("add data into buffer.")
        for i in startIdx..<endIdx {
            //print(pointCloud[i])
            var point = lastArData!.renderVertexBuffer!.contents().assumingMemoryBound(to: RenderedPointCloud.self).advanced(by: i)
            var renPointCloud = RenderedPointCloud(position: pointCloud[i].position,
                                                   color: pointCloud[i].color,
                                                   classColor: pointCloud[i].classColor,
                                                   regionColor: pointCloud[i].regionColor,
                                                   region: pointCloud[i].region,
                                                   curvature: Float(pointCloud[i].curvature))
        
            
            point.pointee.self = renPointCloud
        }
        
        print("Initialization done, add points into finalPointCloud.")
        lastArData!.finalPointCloud = pointCloud
        lastArData!.processedPointCloud = pointCloud
    }
    
    func modifyRenderVertexBufferAtIndex(idx: Int, point: PointCloud) {
        var pointer = self.lastArData!.renderVertexBuffer!.contents().assumingMemoryBound(to: RenderedPointCloud.self).advanced(by: idx)
        let renPointCloud = RenderedPointCloud(position: point.position,
                                               color: point.color,
                                               classColor: point.classColor,
                                               regionColor: point.regionColor,
                                               region: Int32(point.region),
                                               curvature: Float(point.curvature))
        pointer.pointee.self = renPointCloud
    }
    
    
    /// Update the vertex buffer after depthmap to pointcloud conversion
    func addToRenderVertexBuffer(_ pointCloud: [PointCloud]) {
        guard pointCloud.count > 0 else { return }
        let pointCount = pointCloud.count
        
        queue.addOperation {
            
            
            self.lastArData!.cameraPoses[self.lastArData!.cameraPoses.endIndex-1].numberOfPoints = pointCount
            
            // Curvature normalization buffer
            let maxCurvature = pointCloud.max { $0.curvature < $1.curvature }!.curvature
            let minCurvature = pointCloud.min { $0.curvature < $1.curvature }!.curvature
            
            var curPtr = self.curvatureBuffer!.contents().assumingMemoryBound(to: CurvatureNormalization.self).advanced(by: 0)
            curPtr.pointee.self = CurvatureNormalization(minCurvature: minCurvature, maxCurvature: maxCurvature)
            
            let startIdx: Int = self.lastArData!.finalPointCloud.count
            let endIdx: Int = self.lastArData!.finalPointCloud.count + pointCloud.count
            print(startIdx, endIdx, pointCloud.count)
            for i in startIdx..<endIdx {
                var point = self.lastArData!.renderVertexBuffer!.contents().assumingMemoryBound(to: RenderedPointCloud.self).advanced(by: i)
                var renPointCloud = RenderedPointCloud(position: pointCloud[i - startIdx].position,
                                                       color: pointCloud[i - startIdx].color,
                                                       classColor: pointCloud[i - startIdx].classColor,
                                                       regionColor: pointCloud[i - startIdx].regionColor,
                                                       region: Int32(pointCloud[i - startIdx].region),
                                                       curvature: Float(pointCloud[i - startIdx].curvature))
                point.pointee.self = renPointCloud
            }
            
            self.lastArData?.finalPointCloud += pointCloud
            self.lastArData?.originalPointCloud += pointCloud
            self.lastArData?.processedPointCloud += pointCloud
        }

    }
    
    // Update the ARKit anchors everytime anchor is added or updated.
    func updateWorldMeshAnchors(_ arAnchors: [ARAnchor]) {
        
        guard let anchors: [ARMeshAnchor] = arAnchors.filter({ $0 is ARMeshAnchor }) as? [ARMeshAnchor] else { return }
        
        lastArData?.worldMeshes = anchors.map { anchor in
            let aTrans = SCNMatrix4(anchor.transform)
            
            let meshGeometry = anchor.geometry
            let vertices: ARGeometrySource = meshGeometry.vertices
            let normals: ARGeometrySource = meshGeometry.normals
            let submesh: ARGeometryElement = meshGeometry.faces
            var transform = anchor.transform

            let worldMesh = WorldMesh(transform: transform,
                                      vertices: vertices,
                                      normals: normals,
                                      submesh: submesh)
            return worldMesh
        }
    }
    
    func processLastArData() {
        /// RGBA
        lastArData?.colorYTexture.texture = lastArData?.colorImage?.texture(withFormat: .r8Unorm,
                                                                            planeIndex: 0,
                                                                            addToCache: lastArData!.textureCache)!
        lastArData?.colorCbCrTexture.texture = lastArData?.colorImage?.texture(withFormat: .rg8Unorm,
                                                                               planeIndex: 1,
                                                                               addToCache: lastArData!.textureCache)!
        lastArData?.colorRGBTexture.texture = lastArData?.colorImage?.texture(withFormat: .rgba8Unorm,
                                                                              planeIndex: 0,
                                                                              addToCache: lastArData!.textureCache!)!
        
        /// Depth
        lastArData?.depthImageTexture.texture = lastArData?.depthImage?.texture(withFormat: .r32Float,
                                                                                planeIndex: 0,
                                                                                addToCache: lastArData!.textureCache!)!
        
        lastArData?.depthRGBATexture.texture = lastArData?.depthImage?.texture(withFormat: .r32Float,
                                                                                planeIndex: 0,
                                                                                addToCache: lastArData!.textureCache!)!
        
        /// Confidence
        lastArData?.confidenceImageTexture.texture = lastArData?.confidenceImage?.texture(withFormat: .r8Unorm,
                                                                                          planeIndex: 0,
                                                                                          addToCache: lastArData!.textureCache!)!
        
        lastArData?.confRGBATexture.texture = lastArData?.confidenceImage?.texture(withFormat: .r8Unorm,
                                                                                          planeIndex: 0,
                                                                                          addToCache: lastArData!.textureCache!)!
        
        
        guard let cmdBuffer = EnvironmentVariables.shared.metalCommandQueue.makeCommandBuffer() else { return }
    
        
        /// FOR RGBA
        if let computeFunction = EnvironmentVariables.shared.metalLibrary.makeFunction(name: "YUVColorConversion") {
            do {
                computePipelineState = try metalDevice?.makeComputePipelineState(function: computeFunction)
            } catch let error as NSError {
                fatalError("Error: \(error.localizedDescription)")
            }
        } else {
            fatalError("Kernel function not found at runtime.")
        }
        
        guard let computeEncoder = cmdBuffer.makeComputeCommandEncoder() else { return }
        guard lastArData?.depthImageTexture.texture != nil else { return }
        
        computeEncoder.setComputePipelineState(computePipelineState)
        
        computeEncoder.setTexture(lastArData?.colorYTexture.texture, index: 0)
        computeEncoder.setTexture(lastArData?.colorCbCrTexture.texture, index: 1)
        computeEncoder.setTexture(RGBTexture, index: 2)
        
        var threadgroupSize = MTLSizeMake(computePipelineState!.threadExecutionWidth,
                                          computePipelineState!.maxTotalThreadsPerThreadgroup / computePipelineState!.threadExecutionWidth, 1)
        
        var threadgroupCount = MTLSize(width: Int(ceil(Float(RGBTexture.width) / Float(threadgroupSize.width))),
                                       height: Int(ceil(Float(RGBTexture.height) / Float(threadgroupSize.height))),
                                       depth: 1)
        computeEncoder.dispatchThreadgroups(threadgroupCount, threadsPerThreadgroup: threadgroupSize)
        
        computeEncoder.endEncoding()
        
        
        
        
        
        /// FOR DEPTH: FROM ONE CHANNEL TO THREE
        if let computeFunction = EnvironmentVariables.shared.metalLibrary.makeFunction(name: "depthRGBAConversion") {
            do {
                computePipelineState = try metalDevice?.makeComputePipelineState(function: computeFunction)
            } catch let error as NSError {
                fatalError("Error: \(error.localizedDescription)")
            }
        } else {
            fatalError("Kernel function not found at runtime.")
        }
        
        
        guard let computeEncoder = cmdBuffer.makeComputeCommandEncoder() else { return }
        guard lastArData?.depthImageTexture.texture != nil else { return }
        
        computeEncoder.setComputePipelineState(computePipelineState)
        
        computeEncoder.setTexture(lastArData?.depthImageTexture.texture, index: 0)
        computeEncoder.setTexture(depthRGBATexture, index: 1)
        
        threadgroupSize = MTLSizeMake(computePipelineState!.threadExecutionWidth,
                                          computePipelineState!.maxTotalThreadsPerThreadgroup / computePipelineState!.threadExecutionWidth, 1)
        
        threadgroupCount = MTLSize(width: Int(ceil(Float(depthRGBATexture.width) / Float(threadgroupSize.width))),
                                       height: Int(ceil(Float(depthRGBATexture.height) / Float(threadgroupSize.height))),
                                       depth: 1)
        computeEncoder.dispatchThreadgroups(threadgroupCount, threadsPerThreadgroup: threadgroupSize)
        
        computeEncoder.endEncoding()
        
        
        
        
        
        /// FOR CONFIDENCE: FROM ONE CHANNEL TO THREE
        if let computeFunction = EnvironmentVariables.shared.metalLibrary.makeFunction(name: "confRGBAConversion") {
            do {
                computePipelineState = try metalDevice?.makeComputePipelineState(function: computeFunction)
            } catch let error as NSError {
                fatalError("Error: \(error.localizedDescription)")
            }
        } else {
            fatalError("Kernel function not found at runtime.")
        }
        
        
        guard let computeEncoder = cmdBuffer.makeComputeCommandEncoder() else { return }
        guard lastArData?.depthImageTexture.texture != nil else { return }
        
        computeEncoder.setComputePipelineState(computePipelineState)
        
        computeEncoder.setTexture(lastArData?.confidenceImageTexture.texture, index: 0)
        computeEncoder.setTexture(confRGBATexture, index: 1)
        
        threadgroupSize = MTLSizeMake(computePipelineState!.threadExecutionWidth,
                                    computePipelineState!.maxTotalThreadsPerThreadgroup / computePipelineState!.threadExecutionWidth, 1)
        
        threadgroupCount = MTLSize(width: Int(ceil(Float(confRGBATexture.width) / Float(threadgroupSize.width))),
                                       height: Int(ceil(Float(confRGBATexture.height) / Float(threadgroupSize.height))),
                                       depth: 1)
        computeEncoder.dispatchThreadgroups(threadgroupCount, threadsPerThreadgroup: threadgroupSize)
        
        computeEncoder.endEncoding()
        
    
        
    
        
        

        // Upscale the confidence data. Pass in the target resolution.
        mpsScaleFilter?.encode(commandBuffer: cmdBuffer, sourceTexture: lastArData!.depthImageTexture.texture! ,
                               destinationTexture: upscaledDepthTexture)
        
        mpsScaleFilter?.encode(commandBuffer: cmdBuffer, sourceTexture: RGBTexture ,
                               destinationTexture: downscaledRGBTexture)
        
        mpsScaleFilter?.encode(commandBuffer: cmdBuffer, sourceTexture: lastArData!.confidenceImageTexture.texture! ,
                               destinationTexture: upscaledConfTexture)


        cmdBuffer.commit()

        lastArData!.colorRGBTexture.texture = RGBTexture
        lastArData!.downscaledRGBTexture.texture = downscaledRGBTexture
        lastArData!.confRGBATexture.texture = confRGBATexture
        lastArData!.depthRGBATexture.texture = depthRGBATexture
        
        lastArData!.depthImageTexture.texture = upscaledDepthTexture
        lastArData!.confidenceImageTexture.texture = upscaledConfTexture
        
        
    }
    
}
