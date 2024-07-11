//
//  PointCloudProcessing.swift
//  DigitalMicroAirport
//
//  Created by Jussi Kalliola (TAU) on 1.11.2022.
//

import Foundation
import simd
import Vision
import Accelerate

var regionIndices: [Int32] = []
var numberOfRegions: Int = 0
let queue = OperationQueue()

/// Converts xyz rgb points into PCLPoint3D which can be used in PCL functions.
private func convertPointsToPCLPoint3D(points: [SIMD3<Float>], colors: [SIMD3<Float>]) -> [PCLPoint3D] {
    var pointsPtr: [PCLPoint3D] = []
    for (i, val) in points.enumerated() {
        pointsPtr.append(PCLPoint3D(x: Double(val.x),
                                    y: Double(val.y),
                                    z: Double(val.z),
                                    r: Double(colors[i].x),
                                    g: Double(colors[i].y),
                                    b: Double(colors[i].z)))
    }
    
    return pointsPtr
}


/// Converts xyz rgb normal points into PCLPointNormal3D which can be used in PCL functions.
private func convertPointsToPCLPointNormal3D(points: [SIMD3<Float>],
                                             colors: [SIMD3<Float>],
                                             normals: [SIMD3<Float>],
                                             curvatures: [Float]) -> [PCLPointNormal3D] {
    var pointsPtr: [PCLPointNormal3D] = []
    for (i, val) in points.enumerated() {
        pointsPtr.append(PCLPointNormal3D(x: Double(val.x),
                                         y: Double(val.y),
                                         z: Double(val.z),
                                         r: Double(colors[i].x),
                                         g: Double(colors[i].y),
                                         b: Double(colors[i].z),
                                         n_x: Double(normals[i].x),
                                         n_y: Double(normals[i].y),
                                         n_z: Double(normals[i].z),
                                         c: Double(curvatures[i])))
    }
    
    return pointsPtr
}

/// Converts xyz viewpoints points into PCLViewPoint3D which can be used in PCL functions.
private func convertViewpointsToPCLViewPoint3D(viewpoints: [SIMD3<Float>]) -> [PCLViewPoint3D] {
    var pointsPtr: [PCLViewPoint3D] = []
    
    for (_, val) in viewpoints.enumerated() {
        pointsPtr.append(PCLViewPoint3D(x: Double(val.x),
                                        y: Double(val.y),
                                        z: Double(val.z)))
    }
    
    return pointsPtr
}

/// Compute normal and curvature for each point in the pointcloud
func computePointCloudNormals(pointCloud: [PointCloud], viewpoint: SIMD3<Float>, icp: Int) -> [PointCloud] {
    
    let points = pointCloud.compactMap({ $0.position })
    let colors = pointCloud.compactMap({ $0.color })
    
    let pclPoints = convertPointsToPCLPoint3D(points: points, colors: colors)
    let pclViewpoints = convertViewpointsToPCLViewPoint3D(viewpoints: [viewpoint])
    let pclFrameLengths = [Int32(points.count)]
    
    let pclPointCloud = PCLPointCloud(numPoints: Int32(pclPoints.count),
                                      points: pclPoints,
                                      numFrames: Int32(pclViewpoints.count),
                                      pointFrameLengths: pclFrameLengths,
                                      viewpoints: pclViewpoints)
    
    let pclPointNormalCloud = computeNormals(pclPointCloud, Int32(icp))
    
    defer {
        //  Points pointers were allocated in C++ so need to be freed here
        free(pclPointNormalCloud.points)
    }
    
    let pointBufferPointer = UnsafeBufferPointer(start: pclPointNormalCloud.points, count: Int(pclPointNormalCloud.numPoints))
    var pointNormalCloud: [PointCloud] = []
    
    for (index, value) in pointBufferPointer.enumerated() {
        pointNormalCloud.append(PointCloud(position: SIMD3<Float>(x: Float(value.x), y: Float(value.y), z: Float(value.z)),
                                           color: SIMD3<Float>(x: Float(value.r), y: Float(value.g), z: Float(value.b)),
                                           classColor: pointCloud[index].classColor,
                                           classIdx: pointCloud[index].classIdx,
                                           terrainComplexity: pointCloud[index].terrainComplexity,
                                           regionColor: pointCloud[index].regionColor,
                                           region: pointCloud[index].region,
                                           texCoords: pointCloud[index].texCoords,
                                           normal: SIMD3<Float>(x: Float(value.n_x), y: Float(value.n_y), z: Float(value.n_z)),
                                           curvature: Float(value.c),
                                           depth: pointCloud[index].depth))
    }
    
    
    return pointNormalCloud
}

/// Downsample the pointcloud using voxelGridFiltering from PCL
func voxelGridSampling(pointCloud: [PointCloud]) -> [PointCloud] {
    
    let points = pointCloud.compactMap({ $0.position })
    let colors = pointCloud.compactMap({ $0.color })
    let normals = pointCloud.compactMap({ $0.normal })
    let curvatures = pointCloud.compactMap({ $0.curvature })
    
    let pclNormalPoints = convertPointsToPCLPointNormal3D(points: points, colors: colors, normals: normals, curvatures: curvatures)
    
    let pclPointCloud = PCLInputPointNormalCloud(numPoints: Int32(pclNormalPoints.count), points: pclNormalPoints)
    
    // Call C++ voxel grid filtering function using C Wrapper
    let pclFilteredPointCloud = voxelGridFilterPointCloud(pclPointCloud)
    
    defer {
        // points pointers were allocated in C++ so need to be freed here
        free(pclFilteredPointCloud.points)
    }
    
    let pointBufferPointer = UnsafeBufferPointer(start: pclFilteredPointCloud.points,
                                                 count: Int(pclFilteredPointCloud.numPoints))
    var filteredPointCloud: [PointCloud] = []
    
    for (index, value) in pointBufferPointer.enumerated() {
        filteredPointCloud.append(PointCloud(position: SIMD3<Float>(Float(value.x), Float(value.y), Float(value.z)),
                                             color: SIMD3<Float>(Float(value.r), Float(value.g), Float(value.b)),
                                             classColor: SIMD3<Float>(1.0, 0.0, 0.0),
                                             classIdx: -1,
                                             terrainComplexity: 99999.0,
                                             regionColor: SIMD3<Float>(0.0, 0.0, 0.0),
                                             region: -1,
                                             texCoords: SIMD2<Float>(0.0, 0.0),
                                             normal: SIMD3<Float>(Float(value.n_x), Float(value.n_y), Float(value.n_z)),
                                             curvature: Float(value.c),
                                             depth: 0.0))
    }
    
    
    return filteredPointCloud
}


/// Cluster points into regions using region growing segmentation from pcl
func regionGrowingSegmentation(pointCloud: [PointCloud], percentile: Double) -> [PointCloud] {
    
    let points = pointCloud.compactMap({ $0.position })
    let colors = pointCloud.compactMap({ $0.color })
    let normals = pointCloud.compactMap({ $0.normal })
    let curvatures = pointCloud.compactMap({ $0.curvature })
    
    let doubleCurvatures = curvatures.compactMap({Double($0)})
    let cThreshold = Sigma.percentile(doubleCurvatures, percentile: percentile)
    
    let pclNormalPoints = convertPointsToPCLPointNormal3D(points: points, colors: colors, normals: normals, curvatures: curvatures)
    
    let pclPointCloud = PCLInputPointNormalCloud(numPoints: Int32(pclNormalPoints.count),
                                                 points: pclNormalPoints)
    
    let pclClusterIndices = pclRegionGrowingSegmentation(pclPointCloud, cThreshold!)
    
    defer {
        // points pointers were allocated in C++ so need to be freed here
        free(pclClusterIndices.points)
    }
    
    let pointBufferPointer = UnsafeBufferPointer(start: pclClusterIndices.points,
                                                 count: Int(pclClusterIndices.numPoints))
    
    var pointCloudWithRegions: [PointCloud] = []
    var regions: [Int32] = []
    var regionColors: [SIMD3<Float>] = []
    
    for (index, value) in pointBufferPointer.enumerated() {
        
        let regionIdx = value.region
        
        if !regions.contains(obj: regionIdx) {
            if regionIdx == -1 {
                let r = Float(0.0)
                let g = Float(0.0)
                let b = Float(0.0)
                let randomColor = SIMD3<Float>(r, g, b)
                
                regionColors.append(randomColor)
                regions.append(regionIdx)
            } else {
                let r = Float(round(drand48() * 255.0))
                let g = Float(round(drand48() * 255.0))
                let b = Float(round(drand48() * 255.0))
                let randomColor = SIMD3<Float>(r, g, b)
                
                regionColors.append(randomColor)
                regions.append(regionIdx)
            }
        }
        pointCloudWithRegions.append(PointCloud(position: pointCloud[index].position,
                                                color: pointCloud[index].color,
                                                classColor: pointCloud[index].classColor,
                                                classIdx: pointCloud[index].classIdx,
                                                terrainComplexity: pointCloud[index].terrainComplexity,
                                                regionColor: regionColors[regions.firstIndex(of: regionIdx)!],
                                                region: regionIdx,
                                                texCoords: pointCloud[index].texCoords,
                                                normal: pointCloud[index].normal,
                                                curvature: Float(pointCloud[index].curvature),
                                                depth: pointCloud[index].depth))
    }
    
    regionIndices = regions
    numberOfRegions = regions.count
    
    return pointCloudWithRegions
}


private func planeFitting(pointCloud: [PointCloud]) -> ([Int], [Float]) {
    let points = pointCloud.compactMap({ SIMD3<Float>($0.position.x, $0.position.z, $0.position.y) })
    let colors = pointCloud.compactMap({ $0.color })
    let normals = pointCloud.compactMap({ $0.normal })
    let curvatures = pointCloud.compactMap({ $0.curvature })
    
    let doubleCurvatures = curvatures.compactMap({Double($0)})
    
    let pclNormalPoints = convertPointsToPCLPointNormal3D(points: points, colors: colors, normals: normals, curvatures: curvatures)
    
    let pclPointCloud = PCLInputPointNormalCloud(numPoints: Int32(pclNormalPoints.count),
                                                 points: pclNormalPoints)
    
    let pclPlaneCoeffInliers = pclPlaneFitting(pclPointCloud)
    
    defer {
        // points pointers were allocated in C++ so need to be freed here
        free(pclPlaneCoeffInliers.inlierIndices)
        free(pclPlaneCoeffInliers.coefficients)
    }
    
    let inlierBufferPointer = UnsafeBufferPointer(start: pclPlaneCoeffInliers.inlierIndices,
                                                 count: Int(pclPlaneCoeffInliers.numInliers))
    
    let coeffBufferPointer = UnsafeBufferPointer(start: pclPlaneCoeffInliers.coefficients,
                                                 count: Int(4))
    
    var inlierIndices: [Int] = []
    var coefficients: [Float] = []
    
    for (index, value) in inlierBufferPointer.enumerated() {
        inlierIndices.append(Int(value))
    }
    
    for (index, value) in coeffBufferPointer.enumerated() {
        coefficients.append(value)
    }
    
    return (inlierIndices, coefficients)
    
    
}

private func getPointNeighborhoodIndices(searchPoint: PointCloud, pointCloud: [PointCloud], radius: Double) -> [Int] {
    
    let points = [searchPoint.position]
    let colors = [searchPoint.color]
    let normals = [searchPoint.normal]
    let curvatures = [searchPoint.curvature]
    
    let pclNormalPoints = convertPointsToPCLPointNormal3D(points: points, colors: colors, normals: normals, curvatures: curvatures)
    
    let pclRegionNearestNeighbors = findRegionNearestNeighbors(pclNormalPoints[0], radius)
    
    defer {
        // points pointers were allocated in C++ so need to be freed here
        free(pclRegionNearestNeighbors.indices)
        free(pclRegionNearestNeighbors.distances)
    }
    
    let pointBufferPointer = UnsafeBufferPointer(start: pclRegionNearestNeighbors.indices,
                                                 count: Int(pclRegionNearestNeighbors.numPoints))
    
    var pointNeighborhoodIdx: [Int] = []
    for (_, value) in pointBufferPointer.enumerated() {
        pointNeighborhoodIdx.append(Int(value))
    }
    
    return pointNeighborhoodIdx
}

private func getTerrainEvaluationUnit(searchPoint: PointCloud, pointCloud: [PointCloud], radius: Double) -> [PointCloud] {
    
    let points = [searchPoint.position]
    let colors = [searchPoint.color]
    let normals = [searchPoint.normal]
    let curvatures = [searchPoint.curvature]
    
    let pclNormalPoints = convertPointsToPCLPointNormal3D(points: points, colors: colors, normals: normals, curvatures: curvatures)
    
    let pclRegionNearestNeighbors = findRegionNearestNeighbors(pclNormalPoints[0], radius)
    
    defer {
        // points pointers were allocated in C++ so need to be freed here
        free(pclRegionNearestNeighbors.indices)
        free(pclRegionNearestNeighbors.distances)
    }
    
    let pointBufferPointer = UnsafeBufferPointer(start: pclRegionNearestNeighbors.indices,
                                                 count: Int(pclRegionNearestNeighbors.numPoints))
    
    var terrainEvaluationUnit: [PointCloud] = []
    
    for (_, value) in pointBufferPointer.enumerated() {
        terrainEvaluationUnit.append(pointCloud[Int(value)])
    }
    
    return terrainEvaluationUnit
    
}

private func numberOfPointsInUnit(radius: Double, voxelSize: Double) -> Double {
    let diameter = radius + radius
    let maxNumOfPointsInUnit: Double = ceil(((.pi * pow(radius, 2))
                                              / pow(voxelSize, 2) - (.pi * diameter)/(sqrt(2*pow(voxelSize, 2)))))
    
    print("Maximum number of voxels is ", maxNumOfPointsInUnit, " within radius ", radius)
    return maxNumOfPointsInUnit
}

/// Return the slope between the fitted plane and ground plane in degrees.
private func planeSlopeAngle(fitPlaneNormal: SIMD3<Float>) -> Double {
    var slopeAngle: Double = 0.0;
    var groundPlane: SIMD3<Float> = SIMD3<Float>(0.0, 0.0, 1.0)
    var groundPlaneNormal = sqrt( pow(groundPlane.x,2) + pow(groundPlane.y,2) + pow(groundPlane.z,2) )
    var normPlaneNormal = sqrt( pow(fitPlaneNormal.x,2) + pow(fitPlaneNormal.y,2) + pow(fitPlaneNormal.z,2) )
    
    
    
    slopeAngle = Double(acos( abs( dot(fitPlaneNormal, groundPlane)) / (normPlaneNormal * groundPlaneNormal) ))
    
    //print(slopeAngle * 180.0 / .pi)
    
    return slopeAngle * 180.0 / .pi;
}


private func calculateRelief(pointCloud: [PointCloud]) -> Double {
    let maxZ = abs(Double(pointCloud.max { $0.position.y < $1.position.y }!.position.y))
    let minZ = abs(Double(pointCloud.min { abs($0.position.y) < abs($1.position.y) }!.position.y))
    
    let rel = maxZ - minZ
    return rel
}


private func calculateRoughness(pointCloud: [PointCloud], inlierIndices: [Int], planeEquation: [Float]) -> Double {
    
    let allIndices: Set = Set(0...pointCloud.count-1)
    let inlierIndiceSet: Set = Set(inlierIndices)
    
    let residualIndices: Set = allIndices.subtracting(inlierIndiceSet)
    
    if residualIndices.count == 0 {
        return 0.0
    }
    
    var sumDts: Float = 0.0
    
    for i in residualIndices {
        var pos = pointCloud[i].position
        var dt: Float = abs(planeEquation[0]*pos.x + planeEquation[1]*pos.z + planeEquation[2]*pos.y + planeEquation[3] ) / sqrt(pow(planeEquation[0],2) + pow(planeEquation[1],2) + pow(planeEquation[2],2))
        
        sumDts += pow(dt, 2)
        
    }
    
    var roughness: Double = sqrt(1.0 / Double(pointCloud.count) * Double(sumDts) )
    
    return roughness
}


func resetPointCloud() -> [PointCloud] {
//    var pc: [PointCloud] = pointCloud
//
//    for idx in 0..<pc.count {
//        pc[idx].classIdx = -1
//        pc[idx].terrainComplexity = 99999.0
//        pc[idx].region = -1
//        pc[idx].regionColor = SIMD3<Float>(0.0, 0.0, 0.0)
//        pc[idx].classColor = SIMD3<Float>(0.0, 0.0, 0.0)
//    }
    
    return arProvider.lastArData!.originalPointCloud
    
}


func updateOptimalLandingArea(optimalPoint: PointCloud, pointCloud: inout [PointCloud], radius: Double) {
    
    if(arProvider.useRos && arProvider.websocket!.connected) {
        print("Publish target point with ROS")
//        let d2 = Date()
//        let elapsed = Float(d2.timeIntervalSince(arProvider.d1))
//        let floorInt = floor(elapsed)
//        //print(elapsed)
//        let nanosec = Int((elapsed-floorInt)*1e+6)
        //["header": ["stamp": ["sec": floorInt, "nanosec": nanosec], "frame_id": "iphone"],

        arProvider.websocket?.publish(json: ["op": "publish",
                                             "topic": "/iphone/target",
                                             "msg": ["x": optimalPoint.position.x,
                                                     "y": optimalPoint.position.y,
                                                     "z": optimalPoint.position.z]
                                            ])
    }
    
    let neighborhoodIdx = getPointNeighborhoodIndices(searchPoint: optimalPoint, pointCloud: pointCloud, radius: radius)
    
    //var updatedPointCloud = pointCloud
    
    for i in neighborhoodIdx {
        pointCloud[i].classColor = SIMD3<Float>(0.0, 255.0, 255.0)
    }
    
    //return pointCloud
}


/// Terrain complexity evaluation. Classify each point based on its neighborhood, terrain complexity values are estimated from the neighborhood.
func safeLandingAreaDetermination(pointCloud: inout [PointCloud],
                                  landingRadius: Double,
                                  voxelSize: Double,
                                  slopeThreshold: Double,
                                  roughThreshold: Double,
                                  relThreshold: Double) {
    
    
    queue.maxConcurrentOperationCount = 4
    print("Evaluating terrain based on the complexity parameters.")
    
    let slopeMin = -1 * slopeThreshold
    let slopeMax = slopeThreshold
    
    let roughMin = 0.0
    let roughMax = roughThreshold

    let relMin = -1 * relThreshold
    let relMax = relThreshold
    
    //var classfiedPointCloud: [PointCloud] = pointCloud
    
    let pointsInUnitThreshold: Double = numberOfPointsInUnit(radius: landingRadius, voxelSize: voxelSize)
    
    var riskyRough = 0
    var riskySlope = 0
    var riskyRel = 0
    var riskyNonFlat = 0
    
    
    let r_mid_x = vDSP.mean(pointCloud.flatMap({ $0.position.x }))
    let r_mid_y = vDSP.mean(pointCloud.flatMap({ $0.position.z }))
    
    for regionIdx in regionIndices {
        // Skip the regionless points
        if regionIdx == -1 {
            continue
        }
        
        let pointsIndices = pointCloud.enumerated().filter({ $0.element.region == regionIdx }).map({ $0.offset})
        let points = pointCloud.enumerated().filter({ $0.element.region == regionIdx }).map({$0.element})
        
        
        var planeParams = planeFitting(pointCloud: points)
        var inlierIndices = planeParams.0
        var coefficients = planeParams.1
        
        
        let slopeAngle = planeSlopeAngle(fitPlaneNormal: SIMD3<Float>(coefficients[0], coefficients[1], coefficients[2]))
        
        if slopeAngle > 20 {
            print("Exclude vertical planes. move on.")
            
            for index in pointsIndices {
                pointCloud[index].classIdx = 3
                pointCloud[index].classColor = SIMD3<Float>(255.0, 0.0, 0.0)
                arProvider.modifyRenderVertexBufferAtIndex(idx: index, point: pointCloud[index])
            }
            
            continue
        }
        
        
        print("Processing region with index ", regionIdx, ", number of points: ", pointsIndices.count)
        
        
        
        for index in pointsIndices {
        
            var p = pointCloud[index]
            let terrainEvaluationUnit = getTerrainEvaluationUnit(searchPoint: p, pointCloud: pointCloud, radius: landingRadius)
            
            // If the terrainevaluation unit contains regionless points, then mark as risky
            // Regionless points does not belong into flat regions => shouldnt be flat area
            let containsRegionless = terrainEvaluationUnit.filter { $0.region != regionIdx }
            if Double(containsRegionless.count) > Double(terrainEvaluationUnit.count) * 0.05 {
                pointCloud[index].classIdx = 3
                pointCloud[index].classColor = SIMD3<Float>(255.0, 0.0, 0.0)
                
                var renderPoint = pointCloud[index]
                queue.addOperation {
                    arProvider.modifyRenderVertexBufferAtIndex(idx: index, point: renderPoint)
                }
                
                continue
            }
            
            //print(index, ". Terrain evaluation unit contains ", terrainEvaluationUnit.count, " points.")
            
            // The point is either on the edge or somewhere in the area where not enough information is found
            // => unsuitable
            if terrainEvaluationUnit.count < Int(pointsInUnitThreshold) {
                pointCloud[index].classIdx = 4
                pointCloud[index].classColor = SIMD3<Float>(0.0, 0.0, 255.0)
                
                var renderPoint = pointCloud[index]
                queue.addOperation {
                    arProvider.modifyRenderVertexBufferAtIndex(idx: index, point: renderPoint)
                }
                
                continue
            }
            
            let maxCurvature = terrainEvaluationUnit.max { $0.curvature < $1.curvature }!.curvature
            
            // Non flat point => classify as unsuitable
            if maxCurvature > 0.1 {
                //print(" - Contains non flat point => unsuitable.")
                pointCloud[index].classIdx = 2
                pointCloud[index].classColor = SIMD3<Float>(255.0, 128.0, 0.0)
                riskyNonFlat += 1
                
                var renderPoint = pointCloud[index]
                queue.addOperation {
                    arProvider.modifyRenderVertexBufferAtIndex(idx: index, point: renderPoint)
                }
                continue
            }
            
//                let maxX = terrainEvaluationUnit.max { $0.position.x < $1.position.x }!.position.x
//                let minX = terrainEvaluationUnit.min { $0.position.x < $1.position.x }!.position.x
//                
//                let maxY = terrainEvaluationUnit.max { $0.position.z < $1.position.z }!.position.z
//                let minY = terrainEvaluationUnit.min { $0.position.z < $1.position.z }!.position.z
//                
//                let margin = Float(landingRadius) - 0.05
//                
//                if maxX <= p.position.x + margin || minX >= p.position.x - margin
//                    || maxY <= p.position.z + margin || minY >= p.position.z - margin {
//                    //print(" - Unit is on the edge of the pointcloud => unsuitable.")
//                    classfiedPointCloud[index].classIdx = 4
//                    classfiedPointCloud[index].classColor = SIMD3<Float>(0.0, 0.0, 255.0)
//                    continue
//                }
            
            var planeParams = planeFitting(pointCloud: terrainEvaluationUnit)
            var inlierIndices = planeParams.0
            var coefficients = planeParams.1
            
            
            let slopeAngle = planeSlopeAngle(fitPlaneNormal: SIMD3<Float>(coefficients[0], coefficients[1], coefficients[2]))
            
            if slopeAngle > slopeThreshold {
                pointCloud[index].classIdx = 3
                pointCloud[index].classColor = SIMD3<Float>(255.0, 0.0, 0.0)
                riskySlope += 1
                
                var renderPoint = pointCloud[index]
                queue.addOperation {
                    arProvider.modifyRenderVertexBufferAtIndex(idx: index, point: renderPoint)
                }
                continue
            }
            
            let rel = calculateRelief(pointCloud: terrainEvaluationUnit)
            
            if rel > relThreshold {
                pointCloud[index].classIdx = 3
                pointCloud[index].classColor = SIMD3<Float>(255.0, 0.0, 0.0)
                riskyRel += 1
                
                var renderPoint = pointCloud[index]
                queue.addOperation {
                    arProvider.modifyRenderVertexBufferAtIndex(idx: index, point: renderPoint)
                }
                continue
            }
            
            let roughness = calculateRoughness(pointCloud: terrainEvaluationUnit, inlierIndices: inlierIndices, planeEquation: coefficients)
            
            if roughness > roughThreshold {
                pointCloud[index].classIdx = 3
                pointCloud[index].classColor = SIMD3<Float>(255.0, 0.0, 0.0)
                riskyRough += 1
                
                var renderPoint = pointCloud[index]
                queue.addOperation {
                    arProvider.modifyRenderVertexBufferAtIndex(idx: index, point: renderPoint)
                }
                continue
            }
            
            
            let pointsInSameRegion = Double(terrainEvaluationUnit.filter { $0.region == regionIdx }.count)
            
            
            let slopeNorm = Float(0.333 * ( (slopeAngle - slopeMin) / (slopeMax - slopeMin)))
            let roughnessNorm = Float(0.333 * ( (roughness - roughMin) / (roughMax - roughMin)))
            let relNorm = Float(0.333 * ( (rel - relMin) / (relMax - relMin)))
            //let numberOfPointsVal = Float(abs(1.0 - (Double(terrainEvaluationUnit.count) / pointsInUnitThreshold)))
            let locFromCenter = Float(sqrt( pow(abs(r_mid_y - p.position.z),2) + pow(abs(r_mid_x - p.position.x),2) ))
            let normPointsInSameRegion = 1 - Float((pointsInSameRegion - 0.0) / (pointsInUnitThreshold - 0.0))
            
            let terrainComplexity = slopeNorm + roughnessNorm + relNorm + locFromCenter + normPointsInSameRegion
            
            pointCloud[index].classIdx = 1
            pointCloud[index].terrainComplexity = terrainComplexity
            pointCloud[index].classColor = SIMD3<Float>(0.0, 125.0, 0.0)
            
            var renderPoint = pointCloud[index]
            queue.addOperation {
                arProvider.modifyRenderVertexBufferAtIndex(idx: index, point: renderPoint)
            }
            
            
            //print(index, ". Terrain complexity value:", terrainComplexity)
        
        } // End of pointindices loop
            
        
    } // End of region loop
    
    print("This is done")
    
    let optimalLandingSpot = pointCloud.min { $0.terrainComplexity < $1.terrainComplexity }!
    
    if optimalLandingSpot.terrainComplexity < 10.0 {
        let optLanSpotIdx = pointCloud.firstIndex(of: optimalLandingSpot)
        pointCloud[optLanSpotIdx!].classIdx = 1
        pointCloud[optLanSpotIdx!].classColor = SIMD3<Float>(0.0, 255.0, 255.0)
        
        updateOptimalLandingArea(optimalPoint: pointCloud[optLanSpotIdx!],
                                 pointCloud: &pointCloud,
                                 radius: landingRadius)
    }
    
    print("\n\nrel_t : \(relThreshold), slope_t : \(slopeThreshold), rough_t : \(roughThreshold)")
    print("Risky slope : \(riskySlope) \nRisky rel : \(riskyRel)\nRisky roughness : \(riskyRough)\nRisky non flat : \(riskyNonFlat)")
    print("This is done 2")

    //return classfiedPointCloud
}
