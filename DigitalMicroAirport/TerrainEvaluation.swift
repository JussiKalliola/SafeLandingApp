//
//  TerrainEvaluation.swift
//  DigitalMicroAirport
//
//  Created by Jussi Kalliola (TAU) on 31.10.2022.
//

import Foundation
import simd
import Vision

class TerrainEvaluation {
    var regionGrowingSlope: Double
    var regionGrowingCurvature: Double
    
    var terrainEvalSlope: Double
    var terrainEvalRough: Double
    var terrainEvalRel: Double
    
    let queue = OperationQueue()
    
    init(regionGrowingSlope: Double,
         regionGrowingCurvature: Double,
         terrainEvalSlope: Double,
         terrainEvalRough: Double,
         terrainEvalRel: Double) {
        self.regionGrowingSlope = regionGrowingSlope
        self.regionGrowingCurvature = regionGrowingCurvature
        self.terrainEvalSlope = terrainEvalSlope
        self.terrainEvalRough = terrainEvalRough
        self.terrainEvalRel = terrainEvalRel
        

        queue.qualityOfService = .userInitiated
        queue.maxConcurrentOperationCount = 4
    }
    
//    func pclRegGrowTest(pointCloud: [PointCloud]) {
////        typedef struct PCLPointNormalCloud { // Used for outputting to text files for testing
////            int numPoints;
////            PCLPointNormal3D *points;
////            //PCLPointNormal3D *normals;
////            int numFrames;
////            const int *pointFrameLengths;
////            const PCLViewPoint3D *viewpoints;
////        } PCLPointNormalCloud;
//
//        let pclPoints = pointCloud.compactMap({ PCLPointNormal3D(x:    Double($0.position.x),
//                                                                y:    Double($0.position.y),
//                                                                z:    Double($0.position.z),
//                                                                r:    Double($0.color.x),
//                                                                g:    Double($0.color.y),
//                                                                b:    Double($0.color.z),
//                                                                n_x:  Double($0.normal.x),
//                                                                n_y:  Double($0.normal.y),
//                                                                n_z:  Double($0.normal.z),
//                                                                c:    $0.curvature)})
//
//        let pclPointCloud = PCLInputPointNormalCloud(
//            numPoints: Int32(pclPoints.count),
//            points: pclPoints,
//            numFrames: Int32(0),
//            pointFrameLengths: [0],
//            viewpoints: [PCLViewPoint3D(x: 0.0, y: 0.0, z: 0.0)])
//
//        pclRegionGrowingSegmentation(pclPointCloud)
//
//
//
//    }
    
    func regionGrowingSegmentation(pointCloud: [PointCloud]) -> [PCRegion] {
        
        let curvatures = pointCloud.flatMap({ Double($0.curvature) })
        var c_threshold = Sigma.percentile(curvatures, percentile: regionGrowingCurvature / 100)
        print(regionGrowingCurvature / 100, c_threshold)
        
        var regions: [PCRegion] = []
        
        // 1.1 Sort by height
        let sortedByHeightPointloud = pointCloud.sorted(by: { $0.position.z < $1.position.z })
        
        // 1.2 Sort by curvature
        var sortedPointloud = sortedByHeightPointloud.sorted(by: { $0.curvature < $1.curvature })
        
        
        while(sortedPointloud.count > 0) {
            var SQ: [PointCloud] = []
            var currentRegion: PCRegion = PCRegion(points: [], regionColor: SIMD3<Float>())
            
            let randomColor = Float(drand48())
            let regionColor = SIMD3<Float>(randomColor*255.0, randomColor*255.0, randomColor*255.0)
            
            currentRegion.regionColor = regionColor
            
            // 1.3 Determine seed point A with low height and the smallest curvature
            let A = sortedPointloud[0]
            
            // Add the seed point into current region and queue
            currentRegion.points.append(A)
            SQ.append(A)
            
            sortedPointloud.remove(at: 0)
            
            while(SQ.count > 0) {
                let seedPoint: PointCloud = SQ[0]
                
                // 4.1 Dequeue A from SQ.
                SQ.remove(at: 0)
                
                // 2.1 Search knn of the seed A
                let searchPoint: PCLPointNormal3D = PCLPointNormal3D(x: Double(seedPoint.position.x),
                                                                     y: Double(seedPoint.position.y),
                                                                     z: Double(seedPoint.position.z),
                                                                     r: Double(seedPoint.color.x),
                                                                     g: Double(seedPoint.color.y),
                                                                     b: Double(seedPoint.color.z),
                                                                     n_x: Double(seedPoint.normal.x),
                                                                     n_y: Double(seedPoint.normal.y),
                                                                     n_z: Double(seedPoint.normal.z),
                                                                     c: Double(seedPoint.curvature))
                let PCLknn = findKNearestNeighbors(searchPoint) // Returns indices and distances
                defer {
                    // The mesh points and polygons pointers were allocated in C++ so need to be freed here
                    free(PCLknn.indices)
                    free(PCLknn.distances)
                }
                
                let idxBufferPointer = UnsafeBufferPointer(start: PCLknn.indices, count: Int(PCLknn.numPoints))
                
                var knnPoints: [PointCloud] = []
                for (_, value) in idxBufferPointer.enumerated() {
                    knnPoints.append(pointCloud[Int(value)])
//                    if sortedPointloud.contains(obj: pointCloud[Int(value)]) {
//                        knnPoints.append(pointCloud[Int(value)])
//                    }
                }
                
                for point in knnPoints {
                    
                    guard let idx = sortedPointloud.firstIndex(of: point) else { continue }
                    
                    // 2.2 Calculate angle S_i between normal vectors of each neighbor point B_i and A
                    let angleDeg = angleBetweenPoints(p1: seedPoint.normal, p2: point.normal)
                    
                    // 3.1 If angle Si satisfies S_i < S_threshold => add B_i to region where A is located
                    // 3.2 If curvature c_i of B_i satisfies c_i < c_threshold => add B_i to seed queue SQ as new candidate seed point
                    
                    if (Double(angleDeg) < regionGrowingSlope) {
                        currentRegion.points.append(point)
                        
                        sortedPointloud.remove(at: idx)
                        
                        if Double(point.curvature) < c_threshold! {
                            SQ.append(point)
                        }
                        
                    }

                }
                
                print(SQ.count, currentRegion.points.count, sortedPointloud.count, knnPoints.count)
            }
            
            if currentRegion.points.count > 100 {
                regions.append(currentRegion)
            }
            
        }
        
        return regions
    }
    
    func angleBetweenPoints(p1: SIMD3<Float>, p2: SIMD3<Float>) -> Float {

        var rad = acos(dot(normalize(p1), normalize(p2)).clamped(to: 0...1))
        
        // Return in degrees
        return rad * 180.0 / .pi
    }
}
