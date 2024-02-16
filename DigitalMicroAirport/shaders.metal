//
//  shaders.metal
//  DigitalMicroAirport
//
//  Created by Jussi Kalliola (TAU) on 29.9.2022.
//

#include <metal_stdlib>
using namespace metal;

/// ####################################################
///             IMAGE SHADING FUNCTIONS AND STRUCTURES
/// ####################################################

struct ImageVertex {
    float2 position [[attribute(0)]];
    float2 texCoord [[attribute(1)]];
};

struct TexturedQuadVertexOut {
    float4 position [[position]];
    float2 texCoord;
};

// 2D image contains 4 vertices, so it just maps it to the right place on the screen.
vertex TexturedQuadVertexOut cameraVertexTransform(ImageVertex in [[stage_in]]) {
    TexturedQuadVertexOut out;
    out.position = float4(in.position, 0.0, 1.0);
    out.texCoord = in.texCoord;
    return out;
}


// Takes pixel, converts it to RGB and returns it.
fragment float4 cameraFragmentShader(TexturedQuadVertexOut in [[stage_in]],
                                     constant int &confFilterMode [[ buffer(1) ]],
                                     texture2d<float, access::sample> CameraTextureRGB [[ texture(1) ]],
                                     texture2d<float, access::sample> confidenceTexture [[ texture(2) ]],
                                     texture2d<float, access::sample> depthTexture [[ texture(3) ]])
{
    
    constexpr sampler colorSampler(filter::linear);
    
    const float4x4 ycbcrToRGBTransform = float4x4(
        float4( 1.0000f,  1.0000f,  1.0000f, 0.0000f),
        float4( 0.0000f, -0.3441f,  1.7720f, 0.0000f),
        float4( 1.4020f, -0.7141f,  0.0000f, 0.0000f),
        float4(-0.7010f,  0.5291f, -0.8860f, 1.0000f)
    );


    float4 outColor = float4(CameraTextureRGB.sample(colorSampler, in.texCoord).rgb, 1.0);
    
    float4 depthIn = depthTexture.sample(colorSampler, in.texCoord);
    float depthValue = (depthIn.r + depthIn.b + depthIn.g) / 3;

    int depthInt = int(round(255.0*depthValue));
    //if (depthInt >= 250)
    //    outColor = float4(0.0, 0.0, 0.0, 0.0);

    float4 inConf = confidenceTexture.sample(colorSampler, in.texCoord);
    int confInt = int(round(255.0*(inConf.r)));

    const auto visibility = confInt >= confFilterMode;
    if(visibility==false)
        outColor=float4(0.0, 0.0, 0.0, 0.0);
    
    
    
    return outColor;
}

// for depth. Takes pixel, converts it to RGB and returns it.
fragment float4 depthFragmentShader(TexturedQuadVertexOut in [[stage_in]],
                                    constant int &confFilterMode [[ buffer(1) ]],
                                    texture2d<float, access::sample> depthTexture [[ texture(1) ]],
                                    texture2d<float, access::sample> confidenceTexture [[ texture(2) ]])
{
    
    constexpr sampler colorSampler(filter::linear);
    float4 inColor = depthTexture.sample(colorSampler, in.texCoord);
    float4 inConf = confidenceTexture.sample(colorSampler, in.texCoord);
    
    float value = (inColor.r + inColor.b + inColor.g) / 3;
    
    float4 outColor = float4(value, value, value, 1.0);
    
    int confInt = int(round(255.0*(inConf.r)));
    
    const auto visibility = confInt >= confFilterMode;
    if(visibility==false)
        outColor=float4(1.0, 1.0, 1.0, 1.0);
        
    return outColor;
}


/// ############################
///             CONVERT YUV TO RGB
/// ############################

kernel void YUVColorConversion(texture2d<float, access::read> cameraTextureY [[texture(0)]],
                               texture2d<float, access::sample> cameraTextureCbCr [[ texture(1) ]],
                               texture2d<float, access::write> outTexture [[texture(2)]],
                               uint2 gid [[thread_position_in_grid]])
{
    
    const float4x4 ycbcrToRGBTransform = float4x4(
        float4( 1.0000f,  1.0000f,  1.0000f, 0.0000f),
        float4( 0.0000f, -0.3441f,  1.7720f, 0.0000f),
        float4( 1.4020f, -0.7141f,  0.0000f, 0.0000f),
        float4(-0.7010f,  0.5291f, -0.8860f, 1.0000f)
    );

    uint2 uvCoords = uint2(gid.x / 2, gid.y / 2);
    
    float4 ycbcr = float4(cameraTextureY.read(gid).r,
                          cameraTextureCbCr.read(uvCoords).rg, 1.0);

    float4 outColor = ycbcrToRGBTransform * ycbcr;

    outTexture.write(outColor, gid);
}


/// ##########################################
///             CONVERT 1 CHANNEL TO 3 CHANNELS
/// ##########################################


kernel void depthRGBAConversion(texture2d<float, access::read> cameraDepthTexture [[texture(0)]],
                               texture2d<float, access::write> outTexture [[texture(1)]],
                               uint2 gid [[thread_position_in_grid]])
{
    float4 inColor = cameraDepthTexture.read(gid).r;
    
    float value = (inColor.r + inColor.b + inColor.g) / 3;
    
    float4 outColor = float4(value, value, value, 1.0);

    outTexture.write(outColor, gid);
}


kernel void confRGBAConversion(texture2d<float, access::read> cameraConfTexture [[texture(0)]],
                               texture2d<float, access::write> outTexture [[texture(1)]],
                               uint2 gid [[thread_position_in_grid]])
{
    float4 inConf = cameraConfTexture.read(gid).r;
    
    float4 outColor = float4(255.0*inConf.r, 125.0*inConf.r, 50.0*inConf.r, 1.0);

    outTexture.write(outColor, gid);
}





/// ####################################################
///             ANCHOR SHADING FUNCTIONS AND STRUCTURES
/// ####################################################

/// UNIFORMS
struct FrameUniforms {
    float4x4 projectionMatrix;
    float4x4 viewMatrix;
};

struct InstanceUniforms {
    float4x4 modelMatrix;
};

// VERTICES
struct Vertex {
    float3 position [[attribute(0)]];
    float3 normal   [[attribute(1)]];
    float2 texCoords [[attribute(2)]];
};

struct VertexOut {
    float4 position [[position]];
    float4 color;
    float4 color2;
    float3 eyePosition;
    float3 normal;
    float2 texCoords;
};

// Takes vertex, converts it from the object coordinates into world coordinates,
// and then returns the new position.
vertex VertexOut anchorGeometryVertexTransform(Vertex in [[stage_in]],
                                                constant FrameUniforms &uniforms [[ buffer(3) ]],
                                                constant InstanceUniforms *instanceUniforms [[ buffer(2) ]],
                                                constant float &offsetY [[ buffer(4) ]],
                                                uint vid [[vertex_id]],
                                                uint iid [[instance_id]]) {
    
    VertexOut out;

    float4 position = float4(in.position, 1.0);
    
    float4x4 modelMatrix = instanceUniforms[iid].modelMatrix;
    float4x4 modelViewMatrix = uniforms.viewMatrix * modelMatrix;
    
    // Project to world coordinates
    out.position = uniforms.projectionMatrix * modelViewMatrix * position;
    
    out.position.y = out.position.y + offsetY;
    
    out.eyePosition = (modelViewMatrix * position).xyz;

    float4 normal = modelMatrix * float4(in.normal.xyz, 0.0f);
    out.normal = normalize(normal.xyz);
    
    out.color = float4(0.099, 0.202, 0.125, 0.5f);
    out.color2 = float4(0.0, 0.0, 0.0, 1.0);
    out.texCoords = in.texCoords;
    
    return out;
}


fragment float4 anchorGeometryFragmentLighting(VertexOut in [[stage_in]]) {
    return in.color;
}

fragment float4 geometryOutlineFragment(VertexOut in [[stage_in]])
{
    return in.color2;
}




/// ####################################################
///          POINTCLOUD SHADING FUNCTIONS AND STRUCTURES
/// ####################################################


// VERTICES
struct PointCloudVertex {
    float3 position         [[attribute(0)]];
    float3 color            [[attribute(1)]];
    float3 classColor       [[attribute(2)]];
    int classIdx            [[attribute(3)]];
    float terrainComplexity [[attribute(4)]];
    float3 regionColor      [[attribute(5)]];
    int region              [[attribute(6)]];
    float2 texCoords        [[attribute(7)]];
    float3 normal           [[attribute(8)]];
    float curvature         [[attribute(9)]];
    float depth             [[attribute(10)]];
};

struct PointCloudVertexOut {
    float4 position [[position]];
    float4 color;
    float4 classColor;
    int classIdx;
    float terrainComplexity;
    float4 regionColor;
    int region;
    float2 texCoords;
    float3 normal;
    float curvature;
    float depth;
};

struct PointCloudVertexPc {
    float3 position     [[attribute(0)]];
    float3 color        [[attribute(1)]];
    float3 classColor   [[attribute(2)]];
    float3 regionColor  [[attribute(3)]];
    int region          [[attribute(4)]];
    float curvature     [[attribute(5)]];
};

struct PointCloudVertexOutPc {
    float4 position [[position]];
    float4 color;
    float4 classColor;
    float4 regionColor;
    int region;
    float curvature;
    float pSize [[point_size]];
};

struct CameraIntrinsics {
    float3x3 cameraIntrinsics;
    float4x4 worldPose;
    float3x3 cameraTransformMat;
};

// Position vertices for the point cloud view. Filters out points with
// confidence below the selected confidence value and calculates the color of a
// particle using the color Y and CbCr per vertex. Use `viewMatrix` and
// `cameraIntrinsics` to calculate the world point location of each vertex in
// the depth map.
//- Tag: pointCloudVertexShader
kernel void pointCloudVertexShader(
    //uint vertexID [[ vertex_id ]],
    texture2d<float, access::read> depthTexture [[ texture(0) ]],
    texture2d<float, access::read> confTexture [[ texture(1) ]],
    texture2d<float, access::sample> colorRGBTexture [[ texture(2) ]],
    constant CameraIntrinsics &uniforms [[ buffer(0) ]],
    constant int &confFilterMode [[ buffer(1) ]],
    device PointCloudVertexOut *outBuffer [[buffer(2)]],
    device PointCloudVertexOut *cameraLocalOutBuffer [[buffer(3)]],
    uint2 gid [[thread_position_in_grid]]
    )
{ // ...
    uint2 pos;
    uint texPos = gid.x + gid.y * depthTexture.get_width();
    // Count the rows that are depth-texture-width wide to determine the y-value.
    pos.y = texPos / depthTexture.get_width();
    
    // The x-position is the remainder of the y-value division.
    pos.x = texPos % depthTexture.get_width();
    
    //get depth in meters
    float depth = -depthTexture.read(gid).r;
    
    // Convert confidence from normalized `float` to `int`.
    float4 conf = confTexture.read(gid);
    int confInt = int(round( 255.0f*(conf.r) )) ;
    
    // Filter points by confidence level.
    const auto visibility = confInt >= confFilterMode;
    if(visibility == false)
        depth = 0.0f;
    
    // Filter by depth; if over 4 meters, then filter out
    if (abs(depth) > 2.5f)
        depth = 0.0f;
    

    // Calculate the vertex's world coordinates.
    float3 localPoint = (uniforms.cameraIntrinsics * float3(pos.x, pos.y, 1.0f)) * -depth;
    float4 worldPoint = uniforms.worldPose * float4(localPoint, 1.0f);

    // Color the vertex.
    constexpr sampler textureSampler (mag_filter::linear,
                                      min_filter::linear);
    //out.position = xyzw;
    float2 texCoord = { pos.x / (depthTexture.get_width() - 1.0f), pos.y / (depthTexture.get_height() - 1.0f) };
    float4 outColor = float4(colorRGBTexture.sample(textureSampler, texCoord).rgb, 1.0);
    outColor = outColor * 255.0;

    // Convert YUV to RGB inline.
    outBuffer[texPos].position = float4(worldPoint.x, worldPoint.y, worldPoint.z, worldPoint.w);
    outBuffer[texPos].color = outColor;
    outBuffer[texPos].classColor = float4(1.0, 0.0, 0.0, 1.0);
    outBuffer[texPos].classIdx = -1;
    outBuffer[texPos].terrainComplexity = 99999.0;
    outBuffer[texPos].regionColor = float4(0.0, 0.0, 0.0, 1.0);
    outBuffer[texPos].region = -1;
    outBuffer[texPos].texCoords = texCoord;
    outBuffer[texPos].normal = float3(0.0, 0.0, 0.0);
    outBuffer[texPos].curvature = 0.0f;
    outBuffer[texPos].depth = depth;
    
    cameraLocalOutBuffer[texPos].position = float4(localPoint.x, localPoint.y, localPoint.z, 1.0f);
    cameraLocalOutBuffer[texPos].color = outColor;
    cameraLocalOutBuffer[texPos].classColor = float4(1.0, 0.0, 0.0, 1.0);
    cameraLocalOutBuffer[texPos].classIdx = -1;
    cameraLocalOutBuffer[texPos].terrainComplexity = 99999.0;
    cameraLocalOutBuffer[texPos].regionColor = float4(0.0, 0.0, 0.0, 1.0);
    cameraLocalOutBuffer[texPos].region = -1;
    cameraLocalOutBuffer[texPos].texCoords = texCoord;
    cameraLocalOutBuffer[texPos].normal = float3(0.0, 0.0, 0.0);
    cameraLocalOutBuffer[texPos].curvature = 0.0f;
    cameraLocalOutBuffer[texPos].depth = depth;
}

// Takes each point in the point cloud, converts it from the world coordinates to view space
vertex PointCloudVertexOutPc pointcloudVertexGeometryTransfrom(PointCloudVertexPc in [[stage_in]],
                                                   constant FrameUniforms &uniforms [[ buffer(1) ]],
                                                   constant float &offsetY [[ buffer(2) ]],
                                                   constant float &pointSize [[ buffer(3) ]],
                                                   uint vid [[vertex_id]]) {
    
    PointCloudVertexOutPc out;

    float4 position = float4(in.position.x, in.position.y, in.position.z, 1.0);
    
    // Project to image coordinates
    float4 outPosition = uniforms.viewMatrix * position;
    out.position = uniforms.projectionMatrix * outPosition;
    out.position.y = out.position.y + offsetY;
    out.color = float4(in.color/255.0, 0.5);
    out.classColor = float4(in.classColor/255.0, 0.5);
    out.regionColor = float4(in.regionColor/255.0, 1.0);
    out.region = in.region;
    out.curvature = in.curvature;
    out.pSize = pointSize;
    if (in.classColor.y == 255.0 && in.classColor.z == 255.0){
        out.classColor = float4(in.classColor/255.0, 1.0);
        out.pSize = 20;
    }
    
    return out;
}

struct CurvatureNormalization {
    float minCurvature;
    float maxCurvature;
};

// Draw the point
fragment float4 pointcloudGeometryFragment(PointCloudVertexOutPc in [[stage_in]],
                                           constant int &colorSelection [[buffer(1)]],
                                           constant CurvatureNormalization &curvatureNorm [[buffer(2)]]) {
    
    float4 outColor = float4();
    
    if (colorSelection == 0) {
        outColor = in.color;
    } else if (colorSelection == 1) {
        if (in.region == -1)
            discard_fragment();
        outColor = in.classColor;
    } else if (colorSelection == 2) {
        float normC = ((in.curvature - curvatureNorm.minCurvature) / (curvatureNorm.maxCurvature - curvatureNorm.minCurvature));
        outColor = float4(normC, 0.0, 0.0, 0.3);
    } else if (colorSelection == 3) {
//        if (in.region == -1)
//            discard_fragment();
        outColor = in.regionColor;
    }
    
    return outColor;
}
