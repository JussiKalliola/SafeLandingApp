//
//  PCLWrapper.hpp
//  DigitalMicroAirport
//
//  Created by Jussi Kalliola (TAU) on 25.10.2022.
//

#ifndef PCLWrapper_hpp
#define PCLWrapper_hpp

#ifdef __cplusplus
extern "C" {
#endif
    
    //------------------ Input ------------------
    typedef struct PCLPoint3D {
        double x, y, z, r, g, b;
    } PCLPoint3D;

    typedef struct PCLPointNormal3D {
        double x, y, z, r, g, b, n_x, n_y, n_z, c;
    } PCLPointNormal3D;

    typedef struct PCLViewPoint3D {
        double x, y, z;
    } PCLViewPoint3D;
    
    typedef struct PCLPointCloud {
        int numPoints;
        const PCLPoint3D *points;
        int numFrames;
        const int *pointFrameLengths;
        const PCLViewPoint3D *viewpoints;
    } PCLPointCloud;

    typedef struct PCLInputPointNormalCloud {
        int numPoints;
        const PCLPointNormal3D *points;
    } PCLInputPointNormalCloud;
    
    //------------------ Output ------------------
    typedef struct PCLRegionPoint {
        int region;
    } PCLRegionPoint;

    typedef struct PCLPointNormalCloud {
        int numPoints;
        PCLPointNormal3D *points;
    } PCLPointNormalCloud;

    typedef struct PCLRegionPointIndices {
        int numPoints;
        PCLRegionPoint *points;
    } PCLRegionPointIndices;

    typedef struct PCLNearestNeighborPointCloud {
        int numPoints;
        int *indices;
        float *distances;
    } PCLNearestNeighborPointCloud;

    typedef struct PCLModelCoefficients {
        float *coefficients;
        int *inlierIndices;
        int numInliers;
    } PCLModelCoefficients;

    typedef struct PCLGlobalVariables {
        int K;
        int minClusterSize;
        int maxClusterSize;
        double curvatureThreshold;
        double smoothnessThreshold;
        float voxelLeafSize;
    } PCLGlobalVariables;
    
    
    //------------------ Header declarations ------------------
    // Compute normals and curvatures for each point
    PCLPointNormalCloud computeNormals(PCLPointCloud inputPCLPointCloud, int useIcp);
    // Filter pointcloud using voxelgridfiltering
    PCLPointNormalCloud voxelGridFilterPointCloud(PCLInputPointNormalCloud inputPCLPointCloud);

    // KNN with number of points
    PCLNearestNeighborPointCloud findKNearestNeighbors(PCLPointNormal3D searchPoint);
    // KNN with radius
    PCLNearestNeighborPointCloud findRegionNearestNeighbors(PCLPointNormal3D searchPoint, double radius);
    
    // Region segmentation
    PCLRegionPointIndices pclRegionGrowingSegmentation(PCLInputPointNormalCloud inputPointCloud, double curvatureThreshold);
    // Find plane equation using RANSAC
    PCLModelCoefficients pclPlaneFitting(PCLInputPointNormalCloud inputPointCloud);

    // Define global variables
    void setGlobalVariables(PCLGlobalVariables globalVars);

    // Initialize KDTree
    void initializeKDTree();
    
#ifdef __cplusplus
}
#endif

#endif /* PCLWrapper_hpp */
