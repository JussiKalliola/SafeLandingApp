//
//  ARStructures.swift
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



// Structure for textures
struct TextureFrame {
    var key: String       // date/time/anything
    var frame: ARFrame    // saved frame, MAYBE CHANGE THIS TO CAPTUREDFRAME TEXTURE AND NOT WHOLE FRAME?
    var pos: SCNVector3   // location in reference to bBox
}


// STRUCTURE MESHDATA FOR SHOWING "SCENEUNDERSTANDING"
struct WorldMesh {
    let transform: float4x4
    let vertices: ARGeometrySource
    let normals: ARGeometrySource
    let submesh: ARGeometryElement
}

struct PCRegion {
    var points: [PointCloud]
    var regionColor: vector_float3
}

// STRUCTURE FOR THE POINT CLOUD
struct PointCloud: Equatable {
    var position:           vector_float3
    var color:              vector_float3
    var classColor:         vector_float3
    var classIdx:           Int
    var terrainComplexity:  Float
    var regionColor:        vector_float3
    var region:             Int32
    var texCoords:          vector_float2
    var normal:             vector_float3
    var curvature:          Float
    var depth:              Float
    
    static func ==(lhs: PointCloud, rhs: PointCloud) -> Bool {
        return lhs.position == rhs.position
    }
}

struct RenderedPointCloud {
    var position: vector_float3
    var color: vector_float3
    var classColor: vector_float3
    var regionColor: vector_float3
    var region: Int32
    var curvature: Float
};

// For curvature visualization
struct CurvatureNormalization {
    var minCurvature: Float
    var maxCurvature: Float
};

struct KDPoint {
    var position: vector_float3
}

struct Settings {
    @State var colorSelection: Int = 0
}

struct Parameters {
    var selectedConfidence: Int
    var colorSelection: Int
    var pointSize: Double
    
    var voxelLeafSize: Double
    var filteringPhase: Int
    
    var useICP: Int
    
    var K: Double
    
    var regGrowSlope: Double
    var regGrowCurPerc: Double
    
    var minClusterSize: Double
    var maxClusterSize: Double
    
    var landingRadius: Double
    var terCompSlope: Double
    var terCompRough: Double
    var terCompRel: Double
    
    var originGps: CLLocationCoordinate2D
}

struct CameraPose {
    var worldPose = simd_float4x4()
    var eulerAngles = simd_float3()
    var translation = simd_float3()
    var intrinsics  = simd_float3x3()
    var worldToCamera = simd_float4x4()
    var numberOfPoints: Int = 0
    var gpsLocation: CLLocationCoordinate2D!
}

struct SavedFrame {
    var pose: CameraPose!
    var rgbPath: String
    var rgbResolution: [Int]
    //var depthPath: String
    var depthPathBin: String
    var depthResolution: [Int]
    //var confPath: String
    var confPathBin: String
    var confResolution: [Int]
    var pointCloud: [PointCloud]?
    var orientation: UIDeviceOrientation!
}

//- Tag: ARData
// Store AR data.
final class ARData {
    //Raw Pixel buffers from ARKit
    var depthImage: CVPixelBuffer?
    var depthSmoothImage: CVPixelBuffer?
    var colorImage: CVPixelBuffer?
    var confidenceImage: CVPixelBuffer?
    var confidenceSmoothImage: CVPixelBuffer?
    // Textures
    var colorRGBTexture: MetalTextureContent = MetalTextureContent()
    var downscaledRGBTexture: MetalTextureContent = MetalTextureContent()
    var depthImageTexture: MetalTextureContent = MetalTextureContent()
    var depthSmoothImageTexture: MetalTextureContent = MetalTextureContent()
    var colorYTexture: MetalTextureContent = MetalTextureContent()
    var colorCbCrTexture: MetalTextureContent = MetalTextureContent()
    var confidenceImageTexture: MetalTextureContent = MetalTextureContent()
    var confidenceSmoothImageTexture: MetalTextureContent = MetalTextureContent()
    var depthRGBATexture: MetalTextureContent = MetalTextureContent()
    var confRGBATexture: MetalTextureContent = MetalTextureContent()
    // All of the data: Frame
    // TRY TO FIND A WAY TO NOT USE THIS SINCE ALL THE OTHER DATA IS USELESS THEN.
    // Need to transfer it no because of the .displayTransform which requires view.drawableSize
    // Which is not available outside of the MTKview
    var frame: ARFrame?
    // Camera params
    var cameraIntrinsics = simd_float3x3()
    var cameraResolution = CGSize()
    // 3D data from ARKit
    var arAnchors: [ARAnchor] = []
    var worldMeshes: [WorldMesh] = []
    var pointCloud: [PointCloud] = []
    var originalPointCloud: [PointCloud] = []
    var processedPointCloud: [PointCloud] = []
    var finalPointCloud: [PointCloud] = []
    // Vertex buffers
    var vertexBuffer: MTLBuffer!
    var cameraFrameVertexBuffer: MTLBuffer!
    var renderVertexBuffer: MTLBuffer!
    var inputVertexBuffer: MTLBuffer!
    // Texture storage
    var textureCloud: [TextureFrame] = []
    var textureCache: CVMetalTextureCache!
    // Additional vars
    var sampleTime : String?
    var sampleTimes: [String] = []
    var worldPose = simd_float4x4()
    var eulerAngles = simd_float3()
    var intrinsics  = simd_float3x3()
    var worldToCamera = simd_float4x4()
    var camera: ARCamera?
    var cameraPoses: [CameraPose] = []
    // AR Tracking state
    var trackingState: ARCamera.TrackingState = ARCamera.TrackingState.notAvailable
    var frameId: Int = 0
}


// Define unfiforms for each frame. Uniforms do not change from one shader within a rendering call.
// Data is constant within a frame (do not change when running individual vertex)
// CHECK: https://www.haroldserrano.com/blog/before-using-metal-computer-graphics-basics
struct FrameUniforms {
    var projectionMatrix: float4x4
    var viewMatrix: float4x4
}

// Define uniform structure for fragments (pixels)
struct FragmentUniforms {
    var ambientLightColor: SIMD3<Float>
    var directionalLightDirection: SIMD3<Float>
    var directionalLightColor: SIMD3<Float>
    var materialShininess: Float
}

struct InstanceUniforms {
    var modelMatrix: float4x4
}

struct CameraIntrinsics {
    var cameraIntrinsics: float3x3
    var worldPose: float4x4
    var cameraTransformMat: float3x3
}
