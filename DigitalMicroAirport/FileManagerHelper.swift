//
//  FileManagerHelper.swift
//  DigitalMicroAirport
//
//  Created by Jussi Kalliola (TAU) on 4.10.2022.
//

import Foundation
import ARKit
import Metal
import SwiftUI
import MetalKit
import VideoToolbox
import SceneKit.ModelIO

class FileManagerHelper {
    var path: URL?
    var fileCounter: Int = 0
    let fileManager: FileManager = FileManager.default
    var suffix: String = ""
    var useRos: Bool = false
    
    init(suffix: String = "test", useRos: Bool = false) {
        self.useRos=useRos
        self.suffix=suffix
        createDirectory()
    }
    
    
    // Create new directory in the temporary directory where all the data from the session is captured.
    public func createDirectory() {
        let currDate = Date()
        let dateFormatter = DateFormatter()
        dateFormatter.dateFormat = "yyyy-MM-dd_HH-mm-ss"
        let currDateString = dateFormatter.string(from : currDate)
        
        let TemporaryDirectory = URL(string: NSTemporaryDirectory())!
        let DirPath = TemporaryDirectory.appendingPathComponent("bundle-" + currDateString + "_" + self.suffix + "/")
        
        do {
            try fileManager.createDirectory(atPath: DirPath.path, withIntermediateDirectories: true, attributes: nil)
            try fileManager.createDirectory(atPath: DirPath.path + "/depth/", withIntermediateDirectories: true, attributes: nil)
            try fileManager.createDirectory(atPath: DirPath.path + "/confidence/", withIntermediateDirectories: true, attributes: nil)
            try fileManager.createDirectory(atPath: DirPath.path + "/mapping/", withIntermediateDirectories: true, attributes: nil)
            try fileManager.createDirectory(atPath: DirPath.path + "/poses/", withIntermediateDirectories: true, attributes: nil)
            try fileManager.createDirectory(atPath: DirPath.path + "/pointclouds/", withIntermediateDirectories: true, attributes: nil)
            try fileManager.createDirectory(atPath: DirPath.path + "/meshes/", withIntermediateDirectories: true, attributes: nil)
            try fileManager.createDirectory(atPath: DirPath.path + "/colmap/", withIntermediateDirectories: true, attributes: nil)
        } catch let error as NSError {
            print("Unable to create directory \(error.debugDescription)")
        }
        
        self.path = URL(fileURLWithPath: DirPath.path)
        print(self.path)
    }
    
    func get_quaternion_from_euler(roll: Float, pitch: Float, yaw: Float) -> SIMD4<Float> {
        var quartenion: SIMD4<Float> = SIMD4<Float>(0.0, 0.0, 0.0, 0.0)
        quartenion.x = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
        quartenion.y = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
        quartenion.z = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
        quartenion.w = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
        
        return quartenion
    }
    
    func convert_rotation_matrix_to_quaternion(worldPose: float4x4) -> (SIMD4<Float>, SIMD3<Float>) {
        var qvec = SIMD4<Float>(0.0, 0.0, 0.0, 0.0)
        var tvec = SIMD3<Float>(0.0, 0.0, 0.0)
        var rotation = simd_float3x3(SIMD3<Float>(worldPose.columns.0.x, worldPose.columns.0.y, worldPose.columns.0.z),
                                     SIMD3<Float>(worldPose.columns.1.x, worldPose.columns.1.y, worldPose.columns.1.z),
                                     SIMD3<Float>(worldPose.columns.2.x, worldPose.columns.2.y, worldPose.columns.2.z))
        var translation = SIMD3<Float>(worldPose.columns.3.x, worldPose.columns.3.y, worldPose.columns.3.z)
        // simd_float3(1.0, 1.0, 1.0)
        var q = simd_quatf(rotation) // simd_quatf(ix: 1.0, iy: 0.0, iz: 1.0, r: 0.0)
        
        var normQ = q.normalized
        
        let (x,y,z,w) = (normQ.imag.x, normQ.imag.y, normQ.imag.z, normQ.real)
        
        let tx: Float = 2.0 * x
        let ty: Float = 2.0 * y
        let tz: Float = 2.0 * z
        let twx: Float = tx * w
        let twy: Float = ty * w
        let twz: Float = tz * w
        let txx: Float = tx * x
        let txy: Float = ty * x
        let txz: Float = tz * x
        let tyy: Float = ty * y
        let tyz: Float = tz * y
        let tzz: Float = tz * z
        let one: Float = 1.0
        
        let qM = float4x4(rows: [simd_float4((one - (tyy + tzz)), (txy-twz), (txz + twy), translation.x),
                                simd_float4((txy + twz), (one - (txx+tzz)), (tyz - twx), translation.y),
                                simd_float4((txz - twy), (tyz+twx), (one-(txx+tyy)), translation.z),
                                simd_float4(0.0, 0.0, 0.0, 1.0)])
        
        
        // Flip the y and z axes
        let flipYZ = matrix_float4x4(
            [1, 0, 0, 0],
            [0, -1, 0, 0],
            [0, 0, -1, 0],
            [0, 0, 0, 1])
        
        var flipR = simd_mul(qM, flipYZ)
        
        var R = simd_float3x3(SIMD3<Float>(flipR.columns.0.x, flipR.columns.0.y, flipR.columns.0.z),
                         SIMD3<Float>(flipR.columns.1.x, flipR.columns.1.y, flipR.columns.1.z),
                         SIMD3<Float>(flipR.columns.2.x, flipR.columns.2.y, flipR.columns.2.z))
        var t = SIMD3<Float>(flipR.columns.3.x, flipR.columns.3.y, flipR.columns.3.z)
        
        // Project from world to camera coordinate system
        var R_inv = R.inverse//R.inverse
        var negRinv = -R_inv
        
        for i in 0..<3 {
            print(negRinv[0][i], negRinv[1][i], negRinv[2][i])
            tvec[i] = negRinv[0][i] * t.x + negRinv[1][i] * t.y + negRinv[2][i] * t.z
        }
        
        
        // Convert 3x3 rotation matrix to 4D quaternion vector
        
        var (m00, m01, m02, m10, m11, m12, m20, m21, m22) = (Float(R_inv[0][0]), Float(R_inv[1][0]), Float(R_inv[2][0]), Float(R_inv[0][1]), Float(R_inv[1][1]), Float(R_inv[2][1]), Float(R_inv[0][2]), Float(R_inv[1][2]), Float(R_inv[2][2]))
        var trace = m00 + m11 + m22
        var eps = Float(1.0e-8)
        
        if trace > 0.0 {
            trace_pos_cond()
            
        } else {
            if (m00 > m11) && (m00 > m22) {
                cond1()
                
            } else {
                if m11 > m22 {
                    cond2()
                    
                } else {
                    cond3()
                    
                }
            }
        }
        
        func trace_pos_cond(){
            print("trace_pos_cond")
            var sq = sqrt(trace + 1.0 + eps) * 2.0
            var qw = 0.25 * sq
            var qx = (m21-m12)/sq
            var qy = (m02-m20)/sq
            var qz = (m10-m01)/sq
            qvec = SIMD4<Float>(qx, qy, qz, qw)
        }
        
        func cond1(){
            print("cond1")
            var sq = sqrt(1.0 + m00 - m11 - m22 + eps) * 2.0
            var qw = (m21-m12)/sq
            var qx = 0.25 * sq
            var qy = (m01+m10)/sq
            var qz = (m02+m20)/sq
            qvec = SIMD4<Float>(qx, qy, qz, qw)
        }
        
        func cond2(){
            print("cond2")
            var sq = sqrt(1.0 + m11 - m00 - m22 + eps) * 2.0
            var qw = (m02-m20)/sq
            var qx = (m01+m10)/sq
            var qy = 0.25 * sq
            var qz = (m12+m21)/sq
            qvec = SIMD4<Float>(qx, qy, qz, qw)
        }
        
        func cond3(){
            print("cond3")
            var sq = sqrt(1.0 + m22 - m00 - m11 + eps) * 2.0
            var qw = (m10-m01)/sq
            var qx = (m02+m20)/sq
            var qy = (m12+m21)/sq
            var qz = 0.25 * sq
            qvec = SIMD4<Float>(qx, qy, qz, qw)
        }

        print(q.axis.x, q.axis.y, q.axis.z, q.angle)
        print(qvec)
        
//        qvec.x = q.axis.x
//        qvec.y = q.axis.y
//        qvec.z = q.axis.z
//        qvec.w = q.angle
        
        return (qvec, tvec)
    }
    
    func writeParameters(params: Parameters) {
        guard let fileUrl = NSURL(fileURLWithPath: self.path!.path).appendingPathComponent("parameters.txt") else { return }
        //var params = Parameters(selectedConfidence: self.selectedConfidence,
        //                        colorSelection: self.colorSelection,
        //                        pointSize: self.pointSize,
        //                        voxelLeafSize: self.voxelLeafSize,
        //                        filteringPhase: self.filteringPhase,
        //                        useICP: self.useICP,
        //                        K: self.K,
        //                        regGrowSlope: self.regGrowSlope,
        //                        regGrowCurPerc: self.regGrowCurPerc,
        //                        minClusterSize: self.minClusterSize,
        //                        maxClusterSize: self.maxClustersize,
        //                        landingRadius: self.landingRadius,
        //                        terCompSlope: self.terCompSlope,
        //                        terCompRough: self.terCompRough,
        //                        terCompRel: self.terCompRel)
        var csvString = "selectedConfidence \(params.selectedConfidence)\n"
        csvString += "colorSelection \(params.colorSelection)\n"
        csvString += "pointSize \(params.pointSize)\n"
        csvString += "voxelLeafSize \(params.voxelLeafSize)\n"
        csvString += "filteringPhase \(params.filteringPhase)\n"
        csvString += "K \(params.K)\n"
        csvString += "regGrowSlope \(params.regGrowSlope)\n"
        csvString += "regGrowCurPerc \(params.regGrowCurPerc)\n"
        csvString += "minClusterSize \(params.minClusterSize)\n"
        csvString += "maxClusterSize \(params.maxClusterSize)\n"
        csvString += "landingRadius \(params.landingRadius)\n"
        csvString += "terCompSlope \(params.terCompSlope)\n"
        csvString += "terCompRough \(params.terCompRough)\n"
        csvString += "terCompRel \(params.terCompRel)\n"
        csvString += "originGps \(params.originGps.latitude) \(params.originGps.longitude)"
        
        do {
            try csvString.write(to: fileUrl, atomically: true, encoding: .utf8)
        } catch {
            print("error creating file")
        }

    }
    
    func writeCameraPose(cameraPoses: [CameraPose], sampleTimes: [String]) {
        for i in 0...cameraPoses.count - 1 {
            do {
                guard let fileUrl = NSURL(fileURLWithPath: self.path!.path + "/poses/").appendingPathComponent("\(i)_camera_info.txt") else { return }
                //var csvString = "\("worldPose"),\("eulerAngles"),\("translation"),\("intrinsics"),\("worldToCamera")\n\n"
                
                var csvString = "timestamp \(sampleTimes[i] )\n"
                
                let flatWorldPose = (0..<4).flatMap { x in (0..<4).map { y in cameraPoses[i].worldPose[x][y] } }
                csvString += "worldPose \(flatWorldPose)\n"
                
                //let flatWorldPose = (0..<4).flatMap { x in (0..<4).map { y in cameraPoses[i].worldPose[x][y] } }
                csvString += "gps \(cameraPoses[i].gpsLocation.latitude) \(cameraPoses[i].gpsLocation.longitude)\n"
                
                
                let flatEulerAngles = (0..<3).flatMap { x in cameraPoses[i].eulerAngles[x] }
                csvString += "eulerAngles \(flatEulerAngles)\n"
                
                let quartenion = get_quaternion_from_euler(roll: cameraPoses[i].eulerAngles.x,
                                                           pitch: cameraPoses[i].eulerAngles.y,
                                                           yaw: cameraPoses[i].eulerAngles.z)
                let flatQuartenion = (0..<4).flatMap { x in quartenion[x] }
                csvString += "quartenion \(flatQuartenion)\n"
                
                let (quartenion2, translation2) = convert_rotation_matrix_to_quaternion(worldPose: cameraPoses[i].worldPose)
                let flatQuartenion2 = (0..<4).flatMap { x in quartenion2[x] }
                csvString += "quartenion2 \(flatQuartenion2)\n"
                
                let flatTranslation2 = (0..<3).flatMap { x in translation2[x] }
                csvString += "translation2 \(flatTranslation2)\n"
                
                let flatTranslation = (0..<3).flatMap { x in cameraPoses[i].translation[x] }
                csvString += "translation \(flatTranslation)\n"
                
                let flatIntrinsics = (0..<3).flatMap { x in (0..<3).map { y in cameraPoses[i].intrinsics[x][y] } }
                csvString += "intrinsics \(flatIntrinsics)\n"
                
                let flatWorldToCamera = (0..<4).flatMap { x in (0..<4).map { y in cameraPoses[i].worldToCamera[x][y] } }
                csvString += "worldToCamera \(flatWorldToCamera)\n"
                //"worldPose \(flattenedWorldPose)"
                try csvString.write(to: fileUrl, atomically: true, encoding: .utf8)
            } catch {
                print("error creating file")
            }
//            "\("eulerAngles"),\("translation"),\("intrinsics"),\("worldToCamera")\n\n"
            
        }
    }
    
    public func writeColmapFiles(cameraPoses: [CameraPose], fileNames: [String]) {
        func createCamerasFile() {
            do {
                guard let fileUrl = NSURL(fileURLWithPath: self.path!.path + "/colmap/").appendingPathComponent("images.txt") else { return }
                
                var str = ""
                
                // ID, QW, QX, QY, QZ, TX, TY, TZ, CAM_ID, IMG_NAME
                // Every other line empty
                for i in 0...cameraPoses.count-1 {
                    let (qvec, tvec) = convert_rotation_matrix_to_quaternion(worldPose: cameraPoses[i].worldPose)
                    str += "\(i+1) \(qvec.w) \(qvec.x) \(qvec.y) \(qvec.z) \(tvec.x) \(tvec.y) \(tvec.z) 1 \(fileNames[i])\n\n"
                }
                
                try str.write(to: fileUrl, atomically: true, encoding: .utf8)
            } catch {
                print("error creating file")
            }
        }
        
        func createPointsFile() {
            do {
                guard let fileUrl = NSURL(fileURLWithPath: self.path!.path + "/colmap/").appendingPathComponent("points3D.txt") else { return }
                var str = ""
                try str.write(to: fileUrl, atomically: true, encoding: .utf8)
            } catch {
                print("error creating file")
            }
        }
        
        func createImagesFile() {
            do {
                guard let fileUrl = NSURL(fileURLWithPath: self.path!.path + "/colmap/").appendingPathComponent("cameras.txt") else { return }
                
                var str = ""
                
                // Every other line empty
                
                let flatIntrinsics = (0..<3).flatMap { x in (0..<3).map { y in cameraPoses[0].intrinsics[x][y] } }

                let xFovDegrees = 2 * atan(Float(1920)/(2 * cameraPoses[0].intrinsics[0,0])) * 180/Float.pi
                let yFovDegrees = 2 * atan(Float(1440)/(2 * cameraPoses[0].intrinsics[1,1])) * 180/Float.pi
                let focal = 1920 / (2.0 * tan(xFovDegrees * .pi / 360.0))
                
                str += "1 SIMPLE_RADIAL 1920 1440 \(focal) \(1920 / 2) \(1440 / 2) 0.0\n\n" //  \(flatIntrinsics[0]) \(flatIntrinsics[6]) \(flatIntrinsics[7])
                
                try str.write(to: fileUrl, atomically: true, encoding: .utf8)
            } catch {
                print("error creating file")
            }
        }
        
        createImagesFile()
        createCamerasFile()
        createPointsFile()
        
        
        
    }
    
    private func distanceTo(_ p1: simd_float3, _ p2: simd_float3) -> Float {
        return sqrt(pow(p1.x - p2.x, 2) + pow(p1.x - p2.y, 2) + pow(p1.z - p2.z, 2));
    }
    
    private func writePCAdditionalDataTxt(pointCloud: [PointCloud], path: String, fileName: String) -> Void {
        guard let fileTxt = NSURL(fileURLWithPath: path + "/pointclouds/").appendingPathComponent(fileName) else { return }
        
        var pointString = ""
        let headers = ["txt", "format ascii 1.0", "element vertex \(pointCloud.count)", "property int id", "property int class_id", "property int region_id", "property float terrain_complexity", "property float texCoord_x", "property float texCoord_y", "end_header"]
        
        for header in headers {
            pointString += header
            pointString += "\r\n"
        }
        
        for i in 0..<pointCloud.count {
            let classId = pointCloud[i].classIdx
            let regionId = pointCloud[i].region
            let tc = pointCloud[i].terrainComplexity
            let texCoords = pointCloud[i].texCoords
            
            pointString += "\(i) \(classId) \(regionId) \(tc) \(texCoords.x) \(texCoords.y) \n"
            
        }
        
        try? pointString.write(toFile: fileTxt.path, atomically: false, encoding: String.Encoding.utf8)
    }
    
    private func writePointCloudAsPLY(pointCloud: [PointCloud], path: String, fileName: String) -> Void {

        // 1
        var originalPointCloudString = ""
        let headers = ["ply", "format ascii 1.0", "element vertex \(pointCloud.count)", "property int id", "property float x", "property float y", "property float z", "property float Nx", "property float Ny", "property float Nz", "property float c", "property uchar red", "property uchar green", "property uchar blue", "property uchar alpha", "element face 0", "property list uchar int vertex_indices", "end_header"]
        for header in headers {
            originalPointCloudString += header
            originalPointCloudString += "\r\n"
        }
        
        
        // 2
        for i in 0..<pointCloud.count {
        
            // 3
            let point = pointCloud[i].position
            let colors = pointCloud[i].color
            let normal = pointCloud[i].normal
            let curvature = pointCloud[i].curvature
            
            // 4
            let red = colors.x
            let green = colors.y
            let blue = colors.z
            
            // 5
            let pvValue = "\(i) \(point.x) \(point.y) \(point.z) \(normal.x) \(normal.y) \(normal.z) \(curvature) \(Int(red)) \(Int(green)) \(Int(blue)) 255"
            originalPointCloudString += pvValue
            originalPointCloudString += "\r\n"
        }
        
        // 6
        guard let filePly = NSURL(fileURLWithPath: path + "/pointclouds/").appendingPathComponent(fileName) else { return }
        
        do {
        
            // 7
            try originalPointCloudString.write(to: filePly, atomically: true, encoding: String.Encoding.ascii)
        } catch {
            print("Failed to write PLY file", error)
        }
    }
    
    
    // Write depth data into temporary file storage
    public func writeFramePointcloud(pointcloudBuffer: MTLBuffer, imageIdx: Int) {

//        guard let srcPtr = CVPixelBufferGetBaseAddress(pixelBuffer) else {
//            print("Failed to retrieve depth pointer.")
//            return
//        }

        //let rowBytes : Int = pointcloudBuffer.allocatedSize//CVPixelBufferGetBytesPerRow(pixelBuffer)
//        let width = Int(CVPixelBufferGetWidth(pixelBuffer))
//        let height = Int(CVPixelBufferGetHeight(pixelBuffer))
        let capacity = pointcloudBuffer.allocatedSize //CVPixelBufferGetDataSize(pixelBuffer)
        let numPoints = capacity / MemoryLayout<PointCloud>.stride
//        let uint8Pointer = srcPtr.bindMemory(to: UInt8.self, capacity: capacity)
//
//
        let s = "pointcloud_local_\(String(format: "%05d", imageIdx))"
        let fileURL = URL(fileURLWithPath: s, relativeTo: self.path).appendingPathExtension("bin")
//
        guard let stream = OutputStream(url: fileURL, append: false) else {
            print("Failed to open depth stream.")
            return
        }
        stream.open()
//
//        for y in 0 ..< numPoints{
//            //stream.write(uint8Pointer + (y * rowBytes), maxLength: Int(rowBytes))
//            stream.write(pointcloudBuffer.contents(), maxLength: capacity)
//        }
//
        stream.write(pointcloudBuffer.contents(), maxLength: capacity)
        stream.close()
        print("frame saved.")
        
    }
    
    // Process mesh and save it to temporary directory
    func processPointCloud(pointCloud:[PointCloud], filteredPointCloud: [PointCloud]) -> Void {
        
        //guard let fileTxt = NSURL(fileURLWithPath: self.path!.path).appendingPathComponent("pointcloud.txt") else { return }
        //var pointString = ""
        
        //pointCloud.forEach{(vert) in
            
        //    pointString += "\(vert.position.x) \(vert.position.y) \(vert.position.z) \(vert.color.x) \(vert.color.y) \(vert.color.z) \(vert.normal.x) \(vert.normal.y) \(vert.normal.z) \(vert.curvature)\n"
            
        //}
        
        //try? pointString.write(toFile: fileTxt.path, atomically: false, encoding: String.Encoding.utf8)
        
        
        writePointCloudAsPLY(pointCloud: pointCloud, path: self.path!.path, fileName: "pointcloud_\(pointCloud.count)_XYZNormalRGBA.ply")
        
        if pointCloud.count != filteredPointCloud.count {
            writePointCloudAsPLY(pointCloud: filteredPointCloud, path: self.path!.path, fileName: "filterd_pointcloud_\(filteredPointCloud.count)_XYZNormalRGBA.ply")
            writePCAdditionalDataTxt(pointCloud: filteredPointCloud, path: self.path!.path, fileName: "filterd_pointcloud_\(filteredPointCloud.count)_additional_data.txt")
        }
    }

    
    // Process mesh and save it to temporary directory
    func processMesh(arAnchors:[ARAnchor]) -> Void {
        
        let meshAnchors = arAnchors.compactMap({ $0 as? ARMeshAnchor })
        // Fetch the default MTLDevice to initialize a MetalKit buffer allocator with
        let device = EnvironmentVariables.shared.metalDevice
        
        // Using the Model I/O framework to export the scan, so we're initialising an MDLAsset object,
        // which we can export to a file later, with a buffer allocator
        let allocator = MTKMeshBufferAllocator(device: device)
        let asset = MDLAsset(bufferAllocator: allocator)
        
        // Convert the geometry of each ARMeshAnchor into a MDLMesh and add it to the MDLAsset
        for meshAncor in meshAnchors {
            
            
            // Some short handles, otherwise stuff will get pretty long in a few lines
            let geometry = meshAncor.geometry
            let vertices = geometry.vertices
            let normals = geometry.normals
            let faces = geometry.faces
            let verticesPointer = vertices.buffer.contents()
            let facesPointer = faces.buffer.contents()
            
            // Converting each vertex of the geometry from the local space of their ARMeshAnchor to world space
            for vertexIndex in 0..<vertices.count {
                
                // Extracting the current vertex with an extension method provided by Apple in Extensions.swift
                let vertex = geometry.vertex(at: UInt32(vertexIndex))
                
                // Building a transform matrix with only the vertex position
                // and apply the mesh anchors transform to convert into world space
                var vertexLocalTransform = matrix_identity_float4x4
                vertexLocalTransform.columns.3 = SIMD4<Float>(x: vertex.x, y: vertex.y, z: vertex.z, w: 1)
                
                let vertexWorldPosition = (meshAncor.transform * vertexLocalTransform).position()
                
                // Writing the world space vertex back into it's position in the vertex buffer
                let vertexOffset = vertices.offset + vertices.stride * vertexIndex
                let componentStride = vertices.stride / 3
                verticesPointer.storeBytes(of: vertexWorldPosition.x, toByteOffset: vertexOffset, as: Float.self)
                verticesPointer.storeBytes(of: vertexWorldPosition.y, toByteOffset: vertexOffset + componentStride, as: Float.self)
                verticesPointer.storeBytes(of: vertexWorldPosition.z, toByteOffset: vertexOffset + (2 * componentStride), as: Float.self)
            }
            
            // HERE WE CAN RETURN STUFF FOR TEXTURE THINGS
            
            
            // Initializing MDLMeshBuffers with the content of the vertex and face MTLBuffers
            let byteCountVertices = vertices.count * vertices.stride
            let byteCountFaces = faces.count * faces.indexCountPerPrimitive * faces.bytesPerIndex
            let vertexBuffer = allocator.newBuffer(with: Data(bytesNoCopy: verticesPointer, count: byteCountVertices, deallocator: .none), type: .vertex)
            let indexBuffer = allocator.newBuffer(with: Data(bytesNoCopy: facesPointer, count: byteCountFaces, deallocator: .none), type: .index)
            
            // Creating a MDLSubMesh with the index buffer and a generic material
            let indexCount = faces.count * faces.indexCountPerPrimitive
            //MDLTExture
            
            let material = MDLMaterial(name: "mat2", scatteringFunction: MDLPhysicallyPlausibleScatteringFunction())
            let submesh = MDLSubmesh(indexBuffer: indexBuffer, indexCount: indexCount, indexType: .uInt32, geometryType: .triangles, material: material)
            
            // Creating a MDLVertexDescriptor to describe the memory layout of the mesh
            let vertexFormat = MTKModelIOVertexFormatFromMetal(vertices.format)
            let vertexDescriptor = MDLVertexDescriptor()
            vertexDescriptor.attributes[0] = MDLVertexAttribute(name: MDLVertexAttributePosition, format: vertexFormat, offset: 0, bufferIndex: 0)
            vertexDescriptor.layouts[0] = MDLVertexBufferLayout(stride: meshAncor.geometry.vertices.stride)
            
            // Finally creating the MDLMesh and adding it to the MDLAsset
            let mesh = MDLMesh(vertexBuffer: vertexBuffer, vertexCount: meshAncor.geometry.vertices.count, descriptor: vertexDescriptor, submeshes: [submesh])
            
            asset.add(mesh)
        }
        
        
        writeMesh(asset: asset, assetName: "scan_mesh.obj")
    }
    
    func writeMesh(asset: MDLAsset, assetName: String) -> Void {
        
        let file = assetName
        let dir = NSURL(fileURLWithPath: self.path!.path + "/meshes/").appendingPathComponent(file)

        do {
            try asset.export(to: dir!)
            print("Mesh saved.")
        } catch {
            fatalError(error.localizedDescription)
        }
    }
    
    func moveFileToFolder(sourcePath: String, targetPath: String) {
        do {
             try fileManager.moveItem(atPath: sourcePath, toPath: targetPath)
            
         } catch {
             print(error.localizedDescription)
         }
    }
    
    
    func writeConfidence(pixelBuffer: CVPixelBuffer, imageIdx: Int) -> String? {
        // Depth map is 32 bit float
        
        guard CVPixelBufferLockBaseAddress(pixelBuffer, .readOnly) == noErr else { return nil }
        defer { CVPixelBufferUnlockBaseAddress(pixelBuffer, .readOnly) }
        
        guard let srcPtr = CVPixelBufferGetBaseAddress(pixelBuffer) else {
            print("Failed to retrieve depth pointer.")
            return nil
        }
    
        let rowBytes : Int = CVPixelBufferGetBytesPerRow(pixelBuffer)
        let width = Int(CVPixelBufferGetWidth(pixelBuffer))
        let height = Int(CVPixelBufferGetHeight(pixelBuffer))
        let capacity = CVPixelBufferGetDataSize(pixelBuffer)
        let uint8Pointer = srcPtr.bindMemory(to: UInt8.self, capacity: capacity)
        
        let s =  "c_\(width)x\(height)_\(String(format: "%05d", imageIdx))"
        let fileURL = URL(fileURLWithPath: s, relativeTo: self.path).appendingPathExtension("bin")
        
        guard let stream = OutputStream(url: fileURL, append: false) else {
            print("Failed to open depth stream.")
            return nil
        }
        stream.open()
        
        for y in 0 ..< height{
            stream.write(uint8Pointer + (y * rowBytes), maxLength: Int(rowBytes))
        }
        
        stream.close()
        if(self.useRos && arProvider.websocket!.connected) {
            print("Publish raw confidence image to ROS")
            do {
                let imageData:NSData = try NSData(bytes: srcPtr, length: capacity)
            
                let strBase64:String = imageData.base64EncodedString(options: .lineLength64Characters)
            
                arProvider.websocket?.publish(json: ["op": "publish", "topic": "/iphone/confidence/image_raw",
                                                     "msg": ["height": height, "width": width, "encoding": "32FC1", "is_bigendian": 0, "step": rowBytes, "data": strBase64]])
            
                
                //print(strBase64)
            } catch let error as NSError {
                fatalError("Error: \(error.localizedDescription)")
            }
        }
        
        return s  + ".bin"
    }
    
    
    // Write depth data into temporary file storage
    public func writeDepth(pixelBuffer: CVPixelBuffer, imageIdx: Int) -> String? {
        
        guard CVPixelBufferLockBaseAddress(pixelBuffer, .readOnly) == noErr else { return nil }
        defer { CVPixelBufferUnlockBaseAddress(pixelBuffer, .readOnly) }
        
        
        guard let srcPtr = CVPixelBufferGetBaseAddress(pixelBuffer) else {
            print("Failed to retrieve depth pointer.")
            return nil
        }

        let rowBytes : Int = CVPixelBufferGetBytesPerRow(pixelBuffer)
        let width = Int(CVPixelBufferGetWidth(pixelBuffer))
        let height = Int(CVPixelBufferGetHeight(pixelBuffer))
        let capacity = CVPixelBufferGetDataSize(pixelBuffer)
        let uint8Pointer = srcPtr.bindMemory(to: UInt8.self, capacity: capacity)


        let s =  "d_\(width)x\(height)_\(String(format: "%05d", imageIdx))"
        let fileURL = URL(fileURLWithPath: s, relativeTo: self.path).appendingPathExtension("bin")

        guard let stream = OutputStream(url: fileURL, append: false) else {
            print("Failed to open depth stream.")
            return nil
        }
        stream.open()

        for y in 0 ..< height{
            stream.write(uint8Pointer + (y * rowBytes), maxLength: Int(rowBytes))
        }

        stream.close()
        print("Depth image saved.")
        if(useRos && arProvider.websocket!.connected){
            print("Publish raw depth image to ROS")
            do {
                let imageData:NSData = try NSData(bytes: srcPtr, length: capacity)

                let strBase64:String = imageData.base64EncodedString(options: .lineLength64Characters)

                arProvider.websocket?.publish(json: ["op": "publish", "topic": "/iphone/depth/image_raw", "msg": ["height": height, "width": width, "encoding": "32FC1", "is_bigendian": 0, "step": rowBytes, "data": strBase64]])


                //print(strBase64)
            } catch let error as NSError {
                fatalError("Error: \(error.localizedDescription)")
            }
        }
        
        //["op": "publish", "topic": "/test_image/compressed", "msg": ["format": "jpeg", "data": strBase64]])
        
        return s  + ".bin"
        
    }
    
    
    // Write RGB image to temporary directory.
    func writeImage(metalTexture: MTLTexture?, imageIdx: Int, imageIdentifier: String) -> String? {
        
        guard let texture = metalTexture else { return nil }
        //MTLTexture.get
        //let bytes = metalTexture.getBytes(T##pixelBytes: UnsafeMutableRawPointer##UnsafeMutableRawPointer, bytesPerRow: <#T##Int#>, from: <#T##MTLRegion#>, mipmapLevel: <#T##Int#>)
        
        let width = texture.width
        let height = texture.height
        
        let bytesPerPixel = 4

        // The total number of bytes of the texture
        let imageByteCount = width * height * bytesPerPixel

        // The number of bytes for each image row
        let bytesPerRow = width * bytesPerPixel

        // An empty buffer that will contain the image
        var src = [UInt8](repeating: 0, count: Int(imageByteCount))

        // Gets the bytes from the texture
        let region = MTLRegionMake2D(0, 0, width, height)
        texture.getBytes(&src, bytesPerRow: bytesPerRow, from: region, mipmapLevel: 0)

        
        
        let data = Data(bytes: src, count: imageByteCount)
        var s: String = ""
        
        if imageIdentifier == "depth" {
            s =  "depth_\(texture.width)x\(texture.height)_\(String(format: "%05d", imageIdx))"
        } else if imageIdentifier == "confidence" {
            s = "conf_\(texture.width)x\(texture.height)_\(String(format: "%05d", imageIdx))"
        } else if imageIdentifier == "image" {
            s = "rgb_\(texture.width)x\(texture.height)_\(String(format: "%05d", imageIdx))"
            arProvider.rgbNames.append(s + ".jpeg")
        } else if imageIdentifier == "downscaledImage" {
            s = "downscaled_rgb_\(texture.width)x\(texture.height)_\(String(format: "%05d", imageIdx))"
        }
        
        let fileURL = URL(fileURLWithPath: s, relativeTo: self.path).appendingPathExtension("bin")
        
        if(useRos && arProvider.websocket!.connected) {
            print("Publish raw image with ROS")
            do {
                try? data.write(to: fileURL)
                let imageData:NSData = try NSData(data: data)//NSData(bytes: src, length: imageByteCount)
            
                let strBase64:String = imageData.base64EncodedString(options: .lineLength64Characters)

                arProvider.websocket?.publish(json: ["op": "publish",
                                                     "topic": "/iphone/rgb/image_raw",
                                                     "msg": ["height": height,
                                                             "width": width,
                                                             "encoding": "bgra8",
                                                             "is_bigendian": 0,
                                                             "step": bytesPerRow,
                                                             "data": strBase64]
                                                    ])


                //print(strBase64)
            } catch let error as NSError {
                fatalError("Error: \(error.localizedDescription)")
            }
        }
        
        return s + ".bin"
    }
    
    
    // Write YUV image to temporary directory.
    func writeImageYUV(pixelBuffer: CVPixelBuffer, imageIdx: Int) {
        // Image is 2 Plane YUV, shape HxW, H/2 x W/2

        guard CVPixelBufferLockBaseAddress(pixelBuffer, .readOnly) == noErr else { return }
        defer { CVPixelBufferUnlockBaseAddress(pixelBuffer, .readOnly) }
        
        guard let srcPtrP0 = CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 0) else {
          return
        }
        guard let srcPtrP1 = CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 1) else {
          return
        }

        let rowBytesP0 : Int = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 0)
        let rowBytesP1 : Int = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 1)
        let widthP0 = Int(CVPixelBufferGetWidthOfPlane(pixelBuffer, 0))
        let widthP1 = Int(CVPixelBufferGetWidthOfPlane(pixelBuffer, 1))
        let heightP0 = Int(CVPixelBufferGetHeightOfPlane(pixelBuffer, 0))
        let heightP1 = Int(CVPixelBufferGetHeightOfPlane(pixelBuffer, 1))

        let uint8PointerP0 = srcPtrP0.bindMemory(to: UInt8.self, capacity: heightP0 * rowBytesP0)
        let uint8PointerP1 = srcPtrP1.bindMemory(to: UInt8.self, capacity: heightP1 * rowBytesP1)

        let s = "image_P0_\(widthP0)x\(heightP0)_P1_\(widthP1)x\(heightP1)_\(imageIdx)"
        let fileURL = URL(fileURLWithPath: s, relativeTo: self.path).appendingPathExtension("bin")

        let stream = OutputStream(url: fileURL, append: false)
        stream?.open()

        for y in 0 ..< heightP0{
            stream?.write(uint8PointerP0 + (y * rowBytesP0), maxLength: Int(rowBytesP0))
        }

        for y in 0 ..< heightP1{
            stream?.write(uint8PointerP1 + (y * rowBytesP1), maxLength: Int(rowBytesP1))
        }

        stream?.close()
        
        print("YUV image saved.")
    }
    
    func mask(from data: [UInt8], width: Int, height: Int) -> UIImage? {
        guard data.count >= 8 else {
            print("data too small")
            return nil
        }

        let colorSpace = CGColorSpaceCreateDeviceRGB() //Gray()

        let bpc = 8
        let bpr = width * 4
        let colorSpace3: CGColorSpace = CGColorSpace(name: CGColorSpace.sRGB)!
        let bmpinfo = CGBitmapInfo(rawValue: CGImageAlphaInfo.premultipliedLast.rawValue)
        var context = CGContext(data: nil,
                                width: width,
                                height: height,
                                bitsPerComponent: bpc,
                                bytesPerRow: bpr,
                                space: colorSpace3,
                                bitmapInfo: bmpinfo.rawValue)
        var buffer = context!.data!.bindMemory(to: UInt8.self, capacity: width * height * 4)
        
        
        for index in 0 ..< width * height * 4 {
            buffer[index] = data[index]
        }

        return context!.makeImage().flatMap { UIImage(cgImage: $0) }
    }
    
    func convertBinToPly(sourceFilePath: String, targetFilePath: String, targetFileName: String) {
        
        do {
            let sourceFilePathURL = URL(fileURLWithPath: sourceFilePath)
            let targetFilePathURL = URL(fileURLWithPath: targetFilePath)
            print(sourceFilePath, targetFilePath)
            
            let rawData: Data = try Data(contentsOf: sourceFilePathURL)
            let byteArray = [UInt8](rawData)
            
            let numPoints = rawData.count / MemoryLayout<PointCloud>.stride
            
            //_data.withUnsafeBytes { $0.load(as: dataStruct.self) }
            
            var pointCloud: [PointCloud] = []
            
            
            for index in 0..<numPoints {
                var converted : PointCloud = rawData.advanced(by: index * MemoryLayout<PointCloud>.stride).withUnsafeBytes { $0.load(as: PointCloud.self) }
                pointCloud.append(converted)
                //print(converted)
            }
            
            writePointCloudAsPLY(pointCloud: pointCloud, path: targetFilePath, fileName: targetFileName)
            
            //var buffer = context!.data!.bindMemory(to: UInt8.self, capacity: width * height * 4)
            
            //let intRawData = [UInt8](rawData)
            //let image = mask(from: byteArray, width: width, height: height)
            
            
            
            
//            guard let data = image?.jpegData(compressionQuality: 1.0) else { return }
//
//            try? data.write(to: targetFilePathURL)
            
            try? fileManager.removeItem(at: URL(fileURLWithPath: sourceFilePath))
            
        } catch {
            fatalError("Couldnt read the file.")
        }
    }
    
    func convertBinToJpeg(sourceFilePath: String, targetFilePath: String, width: Int, height: Int, orientation: UIDeviceOrientation) {
        
        print(sourceFilePath, targetFilePath, width, height, orientation)
        
        do {
            let sourceFilePathURL = URL(fileURLWithPath: sourceFilePath)
            let targetFilePathURL = URL(fileURLWithPath: targetFilePath)
            print(sourceFilePath, targetFilePath)
            
            let rawData: Data = try Data(contentsOf: sourceFilePathURL)
            let byteArray = [UInt8](rawData)
            
            //let intRawData = [UInt8](rawData)
            let image = mask(from: byteArray, width: width, height: height)
            var rotationDeg: CGFloat = 0
            
            //UIDeviceOrientation
            
            if orientation == .portrait {
                rotationDeg = 90
            }
            
            guard let data = image?.imageRotated(on: rotationDeg).jpegData(compressionQuality: 1.0) else { return }
            
            try? data.write(to: targetFilePathURL)
            
            moveFileToFolder(sourcePath: sourceFilePath, targetPath: targetFilePath)
            
            //try? fileManager.removeItem(at: URL(fileURLWithPath: sourceFilePath))
            
        } catch let error as NSError {
            fatalError("\(error.localizedDescription)")
            fatalError("Couldnt read the file.")
        }
    }
    
    
    public func removeFile(targetFile: URL) {
        
    }
    
    // Clear the temporary folder if images are saved somewhere else.
    public func clearTempFolder() -> Void {
        print(self.path!.relativeString)
        print(self.path!.absoluteString)
        do {
            let folderPaths = try self.fileManager.contentsOfDirectory(atPath: NSTemporaryDirectory())
            

            for folderPath in folderPaths {
                
                let combinedFolderPath = NSTemporaryDirectory() + folderPath
                
                let filePaths = try self.fileManager.contentsOfDirectory(atPath: NSTemporaryDirectory() + folderPath)
                
                for filePath in filePaths {
                    try fileManager.removeItem(at: URL(fileURLWithPath: NSTemporaryDirectory() + folderPath + "/" + filePath))
                }
            }
            
            
        } catch {
            print("Could not clear temp folder: \(error)\n\n")
            
        }
        
        // Create a new folder into temporary directory for new data.
        self.createDirectory()
    }
    
}
