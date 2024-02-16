//
//  PCLWrapper.cpp
//  DigitalMicroAirport
//
//  Created by Jussi Kalliola (TAU) on 25.10.2022.
//

#include "PCLWrapper.hpp"
#include <iostream>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/registration/icp.h>

using namespace pcl;
using namespace std;


/*
 Global variables
 */
PointCloud<PointXYZRGBNormal>::Ptr cloud_filtered (new PointCloud<PointXYZRGBNormal> ());
KdTreeFLANN<PointXYZRGBNormal> mainTree;
int K = 10;
int minClusterSize = 50;
int maxClusterSize = 1000000;
double curvatureThreshold = 1.0;
double smoothnessThreshold = 3.0;
float voxelLeafSize = 0.01f;


/*
 Helper method to compute normals.
 */

void setGlobalVariables(PCLGlobalVariables globalVars) {
    K = globalVars.K;
    minClusterSize = globalVars.minClusterSize;
    maxClusterSize = globalVars.maxClusterSize;
    curvatureThreshold = globalVars.curvatureThreshold;
    smoothnessThreshold = globalVars.smoothnessThreshold;
    voxelLeafSize = globalVars.voxelLeafSize;
    
    cout << "Global variables set. K=" << K << ", Minimum cluster size=" << minClusterSize
    << ", Maximum cluster size=" << maxClusterSize << ", Curvature threshold=" << curvatureThreshold
    << ", Smoothness threshold=" << smoothnessThreshold << ", Voxel leaf size=" << voxelLeafSize << endl;
}


void initializeKDTree() {
    // Set kdtree after filtering is done.
    mainTree.setInputCloud (cloud_filtered);
}


PointCloud<Normal>::Ptr computeNormals(PointCloud<PointXYZRGB>::Ptr pointCloudPtr, PCLViewPoint3D viewpoint)
{
    search::KdTree<PointXYZRGB>::Ptr tree(new search::KdTree<PointXYZRGB>());
    NormalEstimationOMP<PointXYZRGB, Normal> ne;
    ne.setSearchMethod(tree);
    ne.setNumberOfThreads(8);
    ne.setInputCloud(pointCloudPtr);
    ne.setKSearch(K);
    ne.setViewPoint(viewpoint.x, viewpoint.y, viewpoint.z);
    
    // Compute normals
    PointCloud<Normal>::Ptr cloudNormalsPtr(new PointCloud<Normal>());
    ne.compute(*cloudNormalsPtr);
    return cloudNormalsPtr;
}

/// Calculate normals and convert the PCLPointCloud to pcl PointXYZRGBNormal cloud
PointCloud<PointXYZRGBNormal>::Ptr constructPointNormalCloud(PCLPointCloud inputPCLPointCloud) {
     //cout << "Constructing Point Cloud with normals" << endl;
    
    // Initalize Empty Point Cloud
    PointCloud<PointXYZRGBNormal>::Ptr pointCloudPtr(new PointCloud<PointXYZRGBNormal>);
    pointCloudPtr->width    = 0; // Always size of cloud
    pointCloudPtr->height   = 1; // Always 1
    pointCloudPtr->is_dense = true;
    pointCloudPtr->points.resize (pointCloudPtr->width * pointCloudPtr->height); // Need this line
    
    int currentPointsIdx = 0;
    for (size_t frameIdx = 0; frameIdx < inputPCLPointCloud.numFrames; frameIdx++) {
        
        int framePointCloudSize = inputPCLPointCloud.pointFrameLengths[frameIdx];
        
        PointCloud<PointXYZRGB>::Ptr tempPointCloudPtr(new PointCloud<PointXYZRGB>);
        tempPointCloudPtr->width    = framePointCloudSize; // Always size of cloud
        tempPointCloudPtr->height   = 1; // Always 1
        tempPointCloudPtr->is_dense = true;
        tempPointCloudPtr->points.resize (tempPointCloudPtr->width * tempPointCloudPtr->height);

        for (size_t i = 0; i < framePointCloudSize; i++, currentPointsIdx++)
        {
            tempPointCloudPtr->points[i].x = inputPCLPointCloud.points[currentPointsIdx].x;
            tempPointCloudPtr->points[i].y = inputPCLPointCloud.points[currentPointsIdx].y;
            tempPointCloudPtr->points[i].z = inputPCLPointCloud.points[currentPointsIdx].z;
            
            tempPointCloudPtr->points[i].r = inputPCLPointCloud.points[currentPointsIdx].r;
            tempPointCloudPtr->points[i].g = inputPCLPointCloud.points[currentPointsIdx].g;
            tempPointCloudPtr->points[i].b = inputPCLPointCloud.points[currentPointsIdx].b;
        }
        
        PointCloud<Normal>::Ptr tempPointCloudNormalsPtr = computeNormals(tempPointCloudPtr, inputPCLPointCloud.viewpoints[frameIdx]);
        
        // Combine Points and Normals
        PointCloud<PointXYZRGBNormal>::Ptr tempCloudSmoothedNormalsPtr(new PointCloud<PointXYZRGBNormal>());
        concatenateFields(*tempPointCloudPtr, *tempPointCloudNormalsPtr, *tempCloudSmoothedNormalsPtr);
        
        // Append temp cloud to full cloud
        *pointCloudPtr += *tempCloudSmoothedNormalsPtr;
    }
    
    // Sanity Check
    //cout << "Num points = " << inputPCLPointCloud.numPoints << ", Last Current Points Index = " << currentPointsIdx << endl;
    
    return pointCloudPtr;
}


/// Convert the PCLInputPointNormalCloud to pcl PointXYZRGBNormal cloud
PointCloud<PointXYZRGBNormal>::Ptr constructPointNormalCloud(PCLInputPointNormalCloud inputPCLPointCloud) {
     //cout << "Constructing Point Cloud with normals" << endl;
    
    // Initalize Empty Point Cloud
    PointCloud<PointXYZRGBNormal>::Ptr pointCloudPtr(new PointCloud<PointXYZRGBNormal>);
    pointCloudPtr->width    = inputPCLPointCloud.numPoints; // Always size of cloud
    pointCloudPtr->height   = 1; // Always 1
    pointCloudPtr->is_dense = true;
    pointCloudPtr->points.resize (pointCloudPtr->width * pointCloudPtr->height); // Need this line
    
    int currentPointsIdx = 0;
    for (size_t i = 0; i < inputPCLPointCloud.numPoints; i++, currentPointsIdx++) {
        pointCloudPtr->points[i].x = inputPCLPointCloud.points[i].x;
        pointCloudPtr->points[i].y = inputPCLPointCloud.points[i].y;
        pointCloudPtr->points[i].z = inputPCLPointCloud.points[i].z;
        
        pointCloudPtr->points[i].r = inputPCLPointCloud.points[i].r;
        pointCloudPtr->points[i].g = inputPCLPointCloud.points[i].g;
        pointCloudPtr->points[i].b = inputPCLPointCloud.points[i].b;
        
        pointCloudPtr->points[i].normal_x = inputPCLPointCloud.points[i].n_x;
        pointCloudPtr->points[i].normal_y = inputPCLPointCloud.points[i].n_y;
        pointCloudPtr->points[i].normal_z = inputPCLPointCloud.points[i].n_z;
        
        pointCloudPtr->points[i].curvature = (float) inputPCLPointCloud.points[i].c;
    }
    
    // Sanity Check
    //cout << "Num points = " << inputPCLPointCloud.numPoints << ", Last Current Points Index = " << currentPointsIdx << endl;
    
    return pointCloudPtr;
}

/// Convert the PCLPointCloud to pcl PointXYZ cloud
PointCloud<PointXYZ>::Ptr constructPointcloud(PCLPointCloud inputPCLPointCloud) {
     cout << "Constructing Point Cloud" << endl;
    
    // Initalize Empty Point Cloud
    PointCloud<PointXYZ>::Ptr pointCloudPtr(new PointCloud<PointXYZ>);
    pointCloudPtr->width    = 0; // Always size of cloud
    pointCloudPtr->height   = 1; // Always 1
    pointCloudPtr->is_dense = true;
    pointCloudPtr->points.resize (pointCloudPtr->width * pointCloudPtr->height); // Need this line
    
    int currentPointsIdx = 0;
    for (size_t frameIdx = 0; frameIdx < inputPCLPointCloud.numFrames; frameIdx++) {
        
        int framePointCloudSize = inputPCLPointCloud.pointFrameLengths[frameIdx];
        
        PointCloud<PointXYZ>::Ptr tempPointCloudPtr(new PointCloud<PointXYZ>);
        tempPointCloudPtr->width    = framePointCloudSize; // Always size of cloud
        tempPointCloudPtr->height   = 1; // Always 1
        tempPointCloudPtr->is_dense = true;
        tempPointCloudPtr->points.resize (tempPointCloudPtr->width * tempPointCloudPtr->height);

        for (size_t i = 0; i < framePointCloudSize; i++, currentPointsIdx++)
        {
            tempPointCloudPtr->points[i].x = inputPCLPointCloud.points[currentPointsIdx].x;
            tempPointCloudPtr->points[i].y = inputPCLPointCloud.points[currentPointsIdx].y;
            tempPointCloudPtr->points[i].z = inputPCLPointCloud.points[currentPointsIdx].z;
        }
        
        // Append temp cloud to full cloud
        *pointCloudPtr += *tempPointCloudPtr;
    }
    
    // Sanity Check
    cout << "Num points = " << inputPCLPointCloud.numPoints << ", Last Current Points Index = " << currentPointsIdx << endl;
    
    return pointCloudPtr;
}


PCLNearestNeighborPointCloud findKNearestNeighbors(PCLPointNormal3D searchPoint) {
    
    PointXYZRGBNormal pclSearchPoint;
    
    pclSearchPoint.x = searchPoint.x;
    pclSearchPoint.y = searchPoint.y;
    pclSearchPoint.z = searchPoint.z;
    
    pclSearchPoint.r = searchPoint.r;
    pclSearchPoint.g = searchPoint.g;
    pclSearchPoint.b = searchPoint.b;
    
    pclSearchPoint.normal_x = searchPoint.n_x;
    pclSearchPoint.normal_y = searchPoint.n_y;
    pclSearchPoint.normal_z = searchPoint.n_z;
    
    pclSearchPoint.curvature = (float) searchPoint.c;
    
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    
    mainTree.nearestKSearch (pclSearchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
    
    PCLNearestNeighborPointCloud pclNNPoints;
    
    // Convert to output format
    long int numPoints = pointIdxNKNSearch.size();
    
    int *indices;
    indices = (int *) calloc(numPoints, sizeof(*indices));
    
    float *distances;
    distances = (float *) calloc(numPoints, sizeof(*distances));
    
    if ( numPoints > 0 )
    {
        for (std::size_t i = 0; i < pointIdxNKNSearch.size (); ++i) {
            if (pointNKNSquaredDistance[i] > 0.0) {
                indices[i] = pointIdxNKNSearch[i];
                distances[i] = pointNKNSquaredDistance[i];
            }
        }
    }
    
    pclNNPoints.numPoints = (int) numPoints;
    pclNNPoints.indices = indices;          // Free in swift
    pclNNPoints.distances = distances;      // Free in swift
    
    return pclNNPoints;
}


PCLModelCoefficients pclPlaneFitting(PCLInputPointNormalCloud inputPointCloud) {
    PointCloud<PointXYZRGBNormal>::Ptr inputCloud = constructPointNormalCloud(inputPointCloud);
    
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGBNormal> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
    
    seg.setInputCloud (inputCloud);
    seg.segment (*inliers, *coefficients);
    
    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
    }
    
//    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
//                                        << coefficients->values[1] << " "
//                                        << coefficients->values[2] << " "
//                                        << coefficients->values[3] << std::endl;
//
//    std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
    
    PCLModelCoefficients pclModelCoefficients;
    
    int *inlierIndicesPtr;
    inlierIndicesPtr = (int *) calloc(inliers->indices.size(), sizeof(*inlierIndicesPtr)); // Must be freed in Swift after method call
    
    for (size_t i = 0; i < inliers->indices.size(); ++i)
    {
        inlierIndicesPtr[i] = inliers->indices[i];
    }
    
    float *coefficientsPtr;
    coefficientsPtr = (float *) calloc(coefficients->values.size(), sizeof(*coefficientsPtr)); // Must be freed in Swift after method call
    
    for(size_t i = 0; i < coefficients->values.size(); ++i)
    {
        coefficientsPtr[i] = coefficients->values[i];
    }
    
    pclModelCoefficients.inlierIndices = inlierIndicesPtr;
    pclModelCoefficients.coefficients = coefficientsPtr;
    pclModelCoefficients.numInliers = inliers->indices.size();
    
    return pclModelCoefficients;
}



PCLNearestNeighborPointCloud findRegionNearestNeighbors(PCLPointNormal3D searchPoint, double radius) {
    
    PointXYZRGBNormal pclSearchPoint;
    
    pclSearchPoint.x = searchPoint.x;
    pclSearchPoint.y = searchPoint.y;
    pclSearchPoint.z = searchPoint.z;
    
    pclSearchPoint.r = searchPoint.r;
    pclSearchPoint.g = searchPoint.g;
    pclSearchPoint.b = searchPoint.b;
    
    pclSearchPoint.normal_x = searchPoint.n_x;
    pclSearchPoint.normal_y = searchPoint.n_y;
    pclSearchPoint.normal_z = searchPoint.n_z;
    
    pclSearchPoint.curvature = (float) searchPoint.c;
    
    std::vector<int> pointIdxNKNSearch;
    std::vector<float> pointNKNSquaredDistance;
    
    mainTree.radiusSearch (pclSearchPoint, (float) radius, pointIdxNKNSearch, pointNKNSquaredDistance);
    
    PCLNearestNeighborPointCloud pclNNPoints;
    
    // Convert to output format
    long int numPoints = pointIdxNKNSearch.size();
    
    int *indices;
    indices = (int *) calloc(numPoints, sizeof(*indices));
    
    float *distances;
    distances = (float *) calloc(numPoints, sizeof(*distances));
    
    if ( numPoints > 0 )
    {
        for (std::size_t i = 0; i < pointIdxNKNSearch.size (); ++i) {
            if (pointNKNSquaredDistance[i] > 0.0) {
                indices[i] = pointIdxNKNSearch[i];
                distances[i] = pointNKNSquaredDistance[i];
            }
        }
    }
    
    pclNNPoints.numPoints = (int) numPoints;
    pclNNPoints.indices = indices;          // Free in swift
    pclNNPoints.distances = distances;      // Free in swift
    
    return pclNNPoints;
}

PCLRegionPointIndices pclRegionGrowingSegmentation(PCLInputPointNormalCloud inputPointCloud, double curvatureThreshold) {
    curvatureThreshold = curvatureThreshold;
    PointCloud<PointXYZRGBNormal>::Ptr inputCloud = constructPointNormalCloud(inputPointCloud);
    
    // Initalize Empty Point Cloud
    PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
    cloud->width    = inputPointCloud.numPoints; // Always size of cloud
    cloud->height   = 1; // Always 1
    cloud->is_dense = true;
    cloud->points.resize (cloud->width * cloud->height); // Need this line
    
    PointCloud<Normal>::Ptr normals (new PointCloud<Normal>);
    normals->width    = inputPointCloud.numPoints; // Always size of cloud
    normals->height   = 1; // Always 1
    normals->is_dense = true;
    normals->points.resize (normals->width * normals->height); // Need this line
    
    long int numPoints = inputPointCloud.numPoints;
    
    std::cout << "Number of points is equal to " << numPoints << std::endl;
    
    for (size_t i = 0; i < numPoints; i++)
    {
        normals->points[i].normal_x = inputPointCloud.points[i].n_x;
        normals->points[i].normal_y = inputPointCloud.points[i].n_y;
        normals->points[i].normal_z = inputPointCloud.points[i].n_z;
        normals->points[i].curvature = (float) inputPointCloud.points[i].c;
        
        cloud->points[i].x = inputPointCloud.points[i].x;
        cloud->points[i].y = inputPointCloud.points[i].y;
        cloud->points[i].z = inputPointCloud.points[i].z;
    }
    
    IndicesPtr indices (new vector <int>);
    PassThrough<PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1.0);
    pass.filter (*indices);
    search::Search<PointXYZ>::Ptr tree (new search::KdTree<PointXYZ>);

    RegionGrowing<PointXYZ, Normal> reg;
    reg.setMinClusterSize (minClusterSize);
    reg.setMaxClusterSize (maxClusterSize);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (K);
    reg.setInputCloud (cloud);
    //reg.setIndices (indices);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (smoothnessThreshold / 180.0 * M_PI);
    reg.setCurvatureThreshold (curvatureThreshold);

    vector <PointIndices> clusters;
    reg.extract (clusters);

    std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
    std::cout << "First cluster has " << clusters[0].indices.size () << " points." << std::endl;
    std::cout << "These are the indices of the points of the initial" <<
    std::endl << "cloud that belong to the first cluster:" << std::endl;
    
    PCLRegionPointIndices pclRegionIndices;
    
    PCLRegionPoint *pointsPtr;
    pointsPtr = (PCLRegionPoint *) calloc(numPoints, sizeof(*pointsPtr)); // Must be freed in Swift after method call
    
    // Init the pointer with "no region" value
    for (size_t i = 0; i < numPoints; i++)
    {
        pointsPtr[i].region = -1;
    }
    
    for (size_t clusterIdx = 0; clusterIdx < clusters.size(); clusterIdx++) {
        std::cout << "Cluster " << clusterIdx << " of " << clusters.size() << std::endl;
        for (size_t i = 0; i < clusters[clusterIdx].indices.size(); i++) {
            pointsPtr[clusters[clusterIdx].indices[i]].region = (int) clusterIdx;
        }
    }
    
    std::cout << "Clusters processed. Returning data." << std::endl;
    
    pclRegionIndices.points = pointsPtr; // Must be freed in Swift after method call
    pclRegionIndices.numPoints = (int) numPoints;
    
    return pclRegionIndices;
}


PCLPointNormalCloud voxelGridFilterPointCloud(PCLInputPointNormalCloud inputPCLPointCloud) {
    
    // Convert the swift pointcloud to c++ pointcloud
    PointCloud<PointXYZRGBNormal>::Ptr cloud = constructPointNormalCloud(inputPCLPointCloud);
    
    PointCloud<PointXYZRGBNormal>::Ptr local_cloud_filtered (new PointCloud<PointXYZRGBNormal>);
    
    // Create the filtering object
    pcl::StatisticalOutlierRemoval<PointXYZRGBNormal> statSor;
    statSor.setInputCloud (cloud);
    statSor.setMeanK (K);
    statSor.setStddevMulThresh (1.0);
    statSor.filter (*local_cloud_filtered);
    
    *cloud = *local_cloud_filtered;

    cout << "PointCloud before filtering: " << cloud->width * cloud->height << " data points (" << getFieldsList (*cloud) << ")." << endl;

    // Create the filtering object
    VoxelGrid<PointXYZRGBNormal> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (voxelLeafSize, voxelLeafSize, voxelLeafSize);
    sor.filter (*cloud_filtered);

    cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points (" << getFieldsList (*cloud_filtered) << ")." << endl;
    
    
    // Set kdtree after filtering is done.
    initializeKDTree();

    PCLPointNormalCloud pclPointCloudFiltered;

    // Convert to output format
    long int numPoints = cloud_filtered->size();

    PCLPointNormal3D *pointsPtr;
    pointsPtr = (PCLPointNormal3D *) calloc(numPoints, sizeof(*pointsPtr)); // Must be freed in Swift after method call
    
//    PCLPointNormal3D *normalsPtr;
//    normalsPtr = (PCLPointNormal3D *) calloc(numPoints, sizeof(*normalsPtr)); // Must be freed in Swift after method call
    
    for (size_t i = 0; i < numPoints; i++)
    {
        pointsPtr[i].x = cloud_filtered->points[i].x;
        pointsPtr[i].y = cloud_filtered->points[i].y;
        pointsPtr[i].z = cloud_filtered->points[i].z;
        
        pointsPtr[i].r = cloud_filtered->points[i].r;
        pointsPtr[i].g = cloud_filtered->points[i].g;
        pointsPtr[i].b = cloud_filtered->points[i].b;
        
        pointsPtr[i].n_x = cloud_filtered->points[i].normal_x;
        pointsPtr[i].n_y = cloud_filtered->points[i].normal_y;
        pointsPtr[i].n_z = cloud_filtered->points[i].normal_z;
        pointsPtr[i].c = (double) cloud_filtered->points[i].curvature;
        //cerr << "curvature" << pointsPtr[i].c << endl;
        
    }

    
    //pclRegionGrowingSegmentation();

    pclPointCloudFiltered.numPoints = (int) numPoints;
    pclPointCloudFiltered.points = pointsPtr;

    return pclPointCloudFiltered;
}

PCLPointNormalCloud computeNormals(PCLPointCloud inputPCLPointCloud, int useIcp) {
    PointCloud<PointXYZRGBNormal>::Ptr cloud = constructPointNormalCloud(inputPCLPointCloud);
    
    long int numPointsGlobalPc = cloud_filtered->size();
    
    if (numPointsGlobalPc > 0 && useIcp == 1) {
        IterativeClosestPoint<PointXYZRGBNormal, PointXYZRGBNormal> icp;
        icp.setInputSource(cloud);
        icp.setInputTarget(cloud_filtered);
        icp.setMaximumIterations(10);
        icp.setTransformationEpsilon(1e-5);
        icp.setEuclideanFitnessEpsilon(1e-3);
        icp.setMaxCorrespondenceDistance(0.02); // 50cm
        icp.setRANSACOutlierRejectionThreshold(0.0002);
        
        PointCloud<PointXYZRGBNormal> Final;
        icp.align(Final);
        
        std::cout << Final.size() << std::endl;
        *cloud = Final;
        
        std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
        
    }
    
    *cloud_filtered += *cloud;
    

    
    long int numPoints = cloud->size();
    
    PCLPointNormal3D *pointsPtr;
    pointsPtr = (PCLPointNormal3D *) calloc(numPoints, sizeof(*pointsPtr)); // Must be freed in Swift after method call
    
    for (size_t i = 0; i < numPoints; i++)
    {
        pointsPtr[i].x      = cloud->points[i].x;
        pointsPtr[i].y      = cloud->points[i].y;
        pointsPtr[i].z      = cloud->points[i].z;
        
        pointsPtr[i].r      = cloud->points[i].r;
        pointsPtr[i].g      = cloud->points[i].g;
        pointsPtr[i].b      = cloud->points[i].b;
        
        pointsPtr[i].n_x    = cloud->points[i].normal_x;
        pointsPtr[i].n_y    = cloud->points[i].normal_y;
        pointsPtr[i].n_z    = cloud->points[i].normal_z;
        pointsPtr[i].c      = (double) cloud->points[i].curvature;
        //cerr << "curvature" << pointsPtr[i].c << endl;
        
    }
    
    PCLPointNormalCloud pclPointNormalCloud;
    
    pclPointNormalCloud.numPoints = (int) numPoints;
    pclPointNormalCloud.points = pointsPtr;
    
    
    return pclPointNormalCloud;
}
