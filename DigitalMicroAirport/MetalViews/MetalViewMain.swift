//
//  MetalViewMain.swift
//  DigitalMicroAirport
//
//  Created by Jussi Kalliola (TAU) on 29.9.2022.
//

import Foundation
import SwiftUI
import MetalKit
import Metal
import ARKit

let kImagePlaneVertexData: [Float] = [
    -1.0, -1.0,  0.0, 1.0,
     1.0, -1.0,  1.0, 1.0,
    -1.0,  1.0,  0.0, 0.0,
     1.0,  1.0,  1.0, 0.0,
]

class MTKCoordinator: NSObject, MTKViewDelegate {
    var arData: ARProvider
    
    let view: MTKView
    var viewportSize: CGSize = CGSize()
    
    // Define all the required rendering variables in main view.
    // Context based variables can be defined in their views.
    let kMaxBuffersInFlight: Int = 3
    let kMaxAnchorInstanceCount: Int = 64
    let kAlignedFrameUniformsSize = 256
    let kAlignedFragmentUniformsSize = 256
    let kAlignedInstanceUniformsSize = 16_384
    
    // Buffers
    var commandQueue: MTLCommandQueue!
    var frameUniformBuffer: MTLBuffer!
    var anchorUniformBuffer: MTLBuffer!
    var fragmentUniformBuffer: MTLBuffer!
    var imagePlaneVertexBuffer: MTLBuffer!
    var pointUniformBuffer: MTLBuffer!
    
    // PipelineStates.
    var cameraPipelineState: MTLRenderPipelineState!
    var anchorPipelineState: MTLRenderPipelineState!
    var pointPipelineState: MTLRenderPipelineState!
    var outlinePipelineState: MTLRenderPipelineState!
    
    // Depthstates.
    var cameraDepthState: MTLDepthStencilState!
    var anchorDepthState: MTLDepthStencilState!
    var pointDepthState: MTLDepthStencilState!
    
    // Unifrom variables: index, offset in bytes and address.
    var uniformBufferIndex: Int = 0
    var frameUniformBufferOffset: Int = 0
    var anchorUniformBufferOffset: Int = 0
    var pointUniformBufferOffset: Int = 0
    var frameUniformBufferAddress: UnsafeMutableRawPointer!
    var anchorUniformBufferAddress: UnsafeMutableRawPointer!
    var pointUniformBufferAddress: UnsafeMutableRawPointer!
    
    // Descriptors
    var vertexDescriptor: MTLVertexDescriptor!
    
    init(view: MTKView, arData: ARProvider) {
        self.view = view
        self.arData = arData

        view.device = EnvironmentVariables.shared.metalDevice
        self.commandQueue = EnvironmentVariables.shared.metalCommandQueue
        
        super.init()
        
        
        self.prepareFunctions()
    }
    
    
    // Prepare functions required in printing the 3D mesh in the camera view.
    func prepareAnchorFunctions(metalDevice: MTLDevice) {
        
        let defaultLibrary = EnvironmentVariables.shared.metalLibrary
        
        let anchorGeometryVertexFunction = defaultLibrary.makeFunction(name: "anchorGeometryVertexTransform")!
        let anchorGeometryFragmentFunction = defaultLibrary.makeFunction(name: "anchorGeometryFragmentLighting")!
        
        vertexDescriptor = MTLVertexDescriptor()
        vertexDescriptor.attributes[0].format = .float3
        vertexDescriptor.attributes[0].offset = 0
        vertexDescriptor.attributes[0].bufferIndex = 0
        vertexDescriptor.attributes[1].format = .float3
        vertexDescriptor.attributes[1].offset = 0
        vertexDescriptor.attributes[1].bufferIndex = 1
        vertexDescriptor.attributes[2].format = .float2
        vertexDescriptor.attributes[2].offset = 0
        vertexDescriptor.attributes[2].bufferIndex = 2
        vertexDescriptor.layouts[0].stride = 12
        vertexDescriptor.layouts[1].stride = 12
        vertexDescriptor.layouts[2].stride = 8

        let anchorPipelineStateDescriptor = MTLRenderPipelineDescriptor()
        anchorPipelineStateDescriptor.sampleCount = view.sampleCount
        anchorPipelineStateDescriptor.vertexFunction = anchorGeometryVertexFunction
        anchorPipelineStateDescriptor.fragmentFunction = anchorGeometryFragmentFunction
        anchorPipelineStateDescriptor.vertexDescriptor = vertexDescriptor
        anchorPipelineStateDescriptor.colorAttachments[0].pixelFormat = view.colorPixelFormat
        anchorPipelineStateDescriptor.colorAttachments[0].isBlendingEnabled = true
        anchorPipelineStateDescriptor.colorAttachments[0].rgbBlendOperation = .add
        anchorPipelineStateDescriptor.colorAttachments[0].alphaBlendOperation = .add
        anchorPipelineStateDescriptor.colorAttachments[0].sourceRGBBlendFactor = .one
        anchorPipelineStateDescriptor.colorAttachments[0].sourceAlphaBlendFactor = .one
        anchorPipelineStateDescriptor.colorAttachments[0].destinationRGBBlendFactor = .oneMinusSourceAlpha
        anchorPipelineStateDescriptor.colorAttachments[0].destinationAlphaBlendFactor = .oneMinusSourceAlpha
        anchorPipelineStateDescriptor.depthAttachmentPixelFormat = view.depthStencilPixelFormat
        anchorPipelineStateDescriptor.stencilAttachmentPixelFormat = view.depthStencilPixelFormat
        
        do {
            try anchorPipelineState = metalDevice.makeRenderPipelineState(descriptor: anchorPipelineStateDescriptor)
        } catch let error {
            fatalError("Failed to create anchor geometry pipeline state, error \(error)")
        }
        
        
        let geometryOutlineVertexFunction = defaultLibrary.makeFunction(name: "anchorGeometryVertexTransform")!
        let geometryOutlineFragmentFunction = defaultLibrary.makeFunction(name: "geometryOutlineFragment")!
        anchorPipelineStateDescriptor.vertexFunction = geometryOutlineVertexFunction
        anchorPipelineStateDescriptor.fragmentFunction = geometryOutlineFragmentFunction

        do {
            try outlinePipelineState = metalDevice.makeRenderPipelineState(descriptor: anchorPipelineStateDescriptor)
        } catch let error {
            print("Failed to create outline geometry pipeline state, error \(error)")
        }
        
        let anchorDepthStateDescriptor = MTLDepthStencilDescriptor()
        anchorDepthStateDescriptor.depthCompareFunction = .lessEqual
        anchorDepthStateDescriptor.isDepthWriteEnabled = true
        anchorDepthState = metalDevice.makeDepthStencilState(descriptor: anchorDepthStateDescriptor)
    }
    
    
    func preparePointcloudFunctions(metalDevice: MTLDevice) {
        let defaultLibrary = EnvironmentVariables.shared.metalLibrary
        
        let pointGeometryVertexFunction = defaultLibrary.makeFunction(name: "pointcloudVertexGeometryTransfrom")!
        let pointGeometryFragmentFunction = defaultLibrary.makeFunction(name: "pointcloudGeometryFragment")!
        
        vertexDescriptor = MTLVertexDescriptor()

        // Position
        vertexDescriptor.attributes[0].format = .float3
        vertexDescriptor.attributes[0].offset = 0
        vertexDescriptor.attributes[0].bufferIndex = 0
        
        // color
        vertexDescriptor.attributes[1].format = .float3
        vertexDescriptor.attributes[1].offset = 0 + MemoryLayout<SIMD3<Float>>.stride
        vertexDescriptor.attributes[1].bufferIndex = 0
        
        // classColor
        vertexDescriptor.attributes[2].format = .float3
        vertexDescriptor.attributes[2].offset = 0 + MemoryLayout<SIMD3<Float>>.stride * 2
        vertexDescriptor.attributes[2].bufferIndex = 0
        
        // regionColor
        vertexDescriptor.attributes[3].format = .float3
        vertexDescriptor.attributes[3].offset = 0 + MemoryLayout<SIMD3<Float>>.stride * 3
        vertexDescriptor.attributes[3].bufferIndex = 0
        
        // region
        vertexDescriptor.attributes[4].format = .int
        vertexDescriptor.attributes[4].offset = 0 + MemoryLayout<SIMD3<Float>>.stride * 4
        vertexDescriptor.attributes[4].bufferIndex = 0
        
        // curvature
        vertexDescriptor.attributes[5].format = .float
        vertexDescriptor.attributes[5].offset = 0 + MemoryLayout<SIMD3<Float>>.stride * 4 + MemoryLayout<Int32>.stride
        vertexDescriptor.attributes[5].bufferIndex = 0

        
        
        
        vertexDescriptor.layouts[0].stride = MemoryLayout<RenderedPointCloud>.stride
        
        
        let pointPipelineStateDescriptor = MTLRenderPipelineDescriptor()
        pointPipelineStateDescriptor.sampleCount = view.sampleCount
        pointPipelineStateDescriptor.vertexFunction = pointGeometryVertexFunction
        pointPipelineStateDescriptor.fragmentFunction = pointGeometryFragmentFunction
        pointPipelineStateDescriptor.vertexDescriptor = vertexDescriptor
        pointPipelineStateDescriptor.colorAttachments[0].pixelFormat = view.colorPixelFormat
        pointPipelineStateDescriptor.colorAttachments[0].isBlendingEnabled = true
        pointPipelineStateDescriptor.colorAttachments[0].rgbBlendOperation = .add
        pointPipelineStateDescriptor.colorAttachments[0].alphaBlendOperation = .add
        pointPipelineStateDescriptor.colorAttachments[0].sourceRGBBlendFactor = .one
        pointPipelineStateDescriptor.colorAttachments[0].sourceAlphaBlendFactor = .one
        pointPipelineStateDescriptor.colorAttachments[0].destinationRGBBlendFactor = .oneMinusSourceAlpha
        pointPipelineStateDescriptor.colorAttachments[0].destinationAlphaBlendFactor = .oneMinusSourceAlpha
        pointPipelineStateDescriptor.depthAttachmentPixelFormat = view.depthStencilPixelFormat
        pointPipelineStateDescriptor.stencilAttachmentPixelFormat = view.depthStencilPixelFormat
        
        do {
            try pointPipelineState = metalDevice.makeRenderPipelineState(descriptor: pointPipelineStateDescriptor)
        } catch let error {
            fatalError("Failed to create anchor geometry pipeline state, error \(error)")
        }
        
        let pointDepthStateDescriptor = MTLDepthStencilDescriptor()
        pointDepthStateDescriptor.depthCompareFunction = .always
        pointDepthStateDescriptor.isDepthWriteEnabled = true
        pointDepthState = metalDevice.makeDepthStencilState(descriptor: pointDepthStateDescriptor)
    }
    
    
    // Prepare common rendering functions.
    func prepareFunctions() {
        guard let metalDevice = view.device else { fatalError("Expected a Metal device.") }
        
        do {
            view.depthStencilPixelFormat = .depth32Float_stencil8
            view.colorPixelFormat = .bgra8Unorm
            view.sampleCount = 1

            let frameUniformBufferSize = kAlignedFrameUniformsSize * kMaxBuffersInFlight

            frameUniformBuffer = metalDevice.makeBuffer(length: frameUniformBufferSize, options: .storageModeShared)
            fragmentUniformBuffer = metalDevice.makeBuffer(length: kAlignedFragmentUniformsSize, options: .storageModeShared)
            
            let imagePlaneVertexDataCount = kImagePlaneVertexData.count * MemoryLayout<Float>.size
            imagePlaneVertexBuffer = metalDevice.makeBuffer(bytes: kImagePlaneVertexData, length: imagePlaneVertexDataCount, options: [])
            
        } catch {
            print("Unexpected error: \(error).")
        }
    }
    
    func updateImagePlane(frame: ARFrame) {
        let displayToCameraTransform = frame.displayTransform(for: .portrait, viewportSize: viewportSize).inverted()

        let vertexData = imagePlaneVertexBuffer.contents().assumingMemoryBound(to: Float.self)
        for index in 0...3 {
            let textureCoordIndex = 4 * index + 2
            let textureCoord = CGPoint(x: CGFloat(kImagePlaneVertexData[textureCoordIndex]),
                                       y: CGFloat(kImagePlaneVertexData[textureCoordIndex + 1]))
            let transformedCoord = textureCoord.applying(displayToCameraTransform)
            vertexData[textureCoordIndex] = Float(transformedCoord.x)
            vertexData[textureCoordIndex + 1] = Float(transformedCoord.y)
        }
    }
    
    func updateFrameUniforms(frame: ARFrame) {
        let uniforms = frameUniformBufferAddress.assumingMemoryBound(to: FrameUniforms.self)
        
        uniforms.pointee.viewMatrix = frame.camera.viewMatrix(for: .portrait)
        uniforms.pointee.projectionMatrix = frame.camera.projectionMatrix(for: .portrait,
                                                                          viewportSize: viewportSize,
                                                                          zNear: 0.05,
                                                                          zFar: 5)
    }
    
    // Update anchor buffers.
    func updateAnchors() {
        guard self.arData.lastArData?.worldMeshes != nil  else { return }
        
        for (index, mesh) in self.arData.lastArData!.worldMeshes.enumerated() {
            let instanceIndex = min(index, kMaxAnchorInstanceCount - 1)
            let modelMatrix = mesh.transform
            let anchorUniforms = anchorUniformBufferAddress.assumingMemoryBound(to: InstanceUniforms.self).advanced(by: instanceIndex)
            anchorUniforms.pointee.modelMatrix = modelMatrix
        }
    }
    
    
    // Update plane buffer state.
    // Call this inside update if overridden.
    // Override if some other buffers need to be updated, e.g., anchor
    func updateBufferStates() {
        uniformBufferIndex = (uniformBufferIndex + 1) % kMaxBuffersInFlight
        
        frameUniformBufferOffset = kAlignedFrameUniformsSize * uniformBufferIndex
        
        frameUniformBufferAddress = frameUniformBuffer.contents().advanced(by: frameUniformBufferOffset)
    }
    
    
    // Here we update stuff which is required to update if something is drawn on the screen.
    // Use this inside update if overriden.
    func updateFrameState() {
        guard let currentFrame = arData.lastArData?.frame else {
            print("No frame available; nothing to display.")
            return
        }
        viewportSize = view.drawableSize
        
        updateImagePlane(frame: currentFrame)
        updateFrameUniforms(frame: currentFrame)
    }
    
    
    // Draw pointCloud in the scene.
    // Remember to run preparePointCloudFunctions() if you are planning to render pointcloud.
    func drawPointcloudGeometry(renderEncoder: MTLRenderCommandEncoder, pointcloudOffsetY: Float, pointSize: Double, colorSelection: Int) {
        
        guard self.arData.lastArData?.renderVertexBuffer != nil, self.arData.lastArData?.renderVertexBuffer.length ?? 0 > 0, self.arData.lastArData?.finalPointCloud.count != 0 else { return }
        
        let pointCount = self.arData.lastArData!.finalPointCloud.count
        
        // Offset buffer
        let offsetYBuffer = EnvironmentVariables.shared.metalDevice.makeBuffer(length: MemoryLayout<Float>.stride, options: .storageModeShared)
        var pointer = offsetYBuffer!.contents().assumingMemoryBound(to: Float.self).advanced(by: 0)
        pointer.pointee.self = pointcloudOffsetY
        
        // Point size buffer
        let pointSizeBuffer = EnvironmentVariables.shared.metalDevice.makeBuffer(length: MemoryLayout<Float>.stride, options: .storageModeShared)
        var sizePointer = pointSizeBuffer!.contents().assumingMemoryBound(to: Float.self).advanced(by: 0)
        sizePointer.pointee.self = Float(pointSize)
        
        // Color selection buffer
        let colorSelBuffer = EnvironmentVariables.shared.metalDevice.makeBuffer(length: MemoryLayout<Int>.stride, options: .storageModeShared)
        var colPointer = colorSelBuffer!.contents().assumingMemoryBound(to: Int.self).advanced(by: 0)
        colPointer.pointee.self = colorSelection
        
        renderEncoder.setVertexBuffer(arData.lastArData!.renderVertexBuffer, offset: 0, index: 0)
        renderEncoder.setVertexBuffer(frameUniformBuffer, offset: 0, index: 1)
        renderEncoder.setVertexBuffer(offsetYBuffer, offset: 0, index: 2)
        renderEncoder.setVertexBuffer(pointSizeBuffer, offset: 0, index: 3)
        
        renderEncoder.setFragmentBuffer(colorSelBuffer, offset: 0, index: 1)
        renderEncoder.setFragmentBuffer(arProvider.curvatureBuffer, offset: 0, index: 2)
        
        renderEncoder.setRenderPipelineState(pointPipelineState)
        renderEncoder.setDepthStencilState(pointDepthState)
        renderEncoder.drawPrimitives(type: .point, vertexStart: 0, vertexCount: pointCount)

    }
    
    // Draw anchor geometry in the scene.
    // Remember to run prepareAnchorFunctions() if you are planning to render anchors.
    func drawAnchorGeometry(renderEncoder: MTLRenderCommandEncoder, pointcloudOffsetY: Float) {
        
        guard self.arData.lastArData?.worldMeshes != nil else { return }
        
        let samplerDescriptor = MTLSamplerDescriptor()
        samplerDescriptor.minFilter = .nearest
        samplerDescriptor.magFilter = .linear
        
        let offsetYBuffer = EnvironmentVariables.shared.metalDevice.makeBuffer(length: MemoryLayout<Float>.stride, options: .storageModeShared)
        
        var pointer = offsetYBuffer!.contents().assumingMemoryBound(to: Float.self).advanced(by: 0)
        pointer.pointee.self = pointcloudOffsetY
        
        let samplerState = EnvironmentVariables.shared.metalDevice.makeSamplerState(descriptor: samplerDescriptor)

        for (index, mesh) in self.arData.lastArData!.worldMeshes.enumerated() {
            renderEncoder.setVertexBuffer(mesh.vertices.buffer, offset: 0, index: 0)
            renderEncoder.setVertexBuffer(mesh.normals.buffer, offset: 0, index: 1)
            renderEncoder.setVertexBuffer(anchorUniformBuffer,
                                          offset: anchorUniformBufferOffset + MemoryLayout<InstanceUniforms>.size * index,
                                          index: 2)
            renderEncoder.setVertexBuffer(frameUniformBuffer, offset: frameUniformBufferOffset, index: 3)
            renderEncoder.setVertexBuffer(offsetYBuffer, offset: 0, index: 4)
            renderEncoder.setFragmentTexture(arData.lastArData?.colorYTexture.texture, index: 1)
            renderEncoder.setFragmentTexture(arData.lastArData?.colorCbCrTexture.texture, index: 2)
            //renderEncoder.setFragmentSamplerState(samplerState, index: 0)

            renderEncoder.setRenderPipelineState(anchorPipelineState)
            renderEncoder.setDepthStencilState(anchorDepthState)
            renderEncoder.setTriangleFillMode(.fill)
            renderEncoder.drawIndexedPrimitives(type: .triangle,
                                                indexCount: mesh.submesh.count * mesh.submesh.indexCountPerPrimitive,
                                                indexType: .uint32,
                                                indexBuffer: mesh.submesh.buffer,
                                                indexBufferOffset: 0)

            renderEncoder.setRenderPipelineState(outlinePipelineState)
            renderEncoder.setDepthStencilState(anchorDepthState)
            renderEncoder.setTriangleFillMode(.lines)
            renderEncoder.drawIndexedPrimitives(type: .triangle,
                                                indexCount: mesh.submesh.count * mesh.submesh.indexCountPerPrimitive,
                                                indexType: .uint32,
                                                indexBuffer: mesh.submesh.buffer,
                                                indexBufferOffset: 0)
        }


    }
    
    // Override if something else needs to be rendered.
    func update() {
        
        if let commandBuffer = commandQueue.makeCommandBuffer() {
            updateBufferStates()
            updateFrameState()
            
            // Here we dont actually render anything, but just setup "template".
            // Override whole function when something needs to be rendered.
            if let renderPassDescriptor = view.currentRenderPassDescriptor, let currentDrawable = view.currentDrawable, let renderEncoder = commandBuffer.makeRenderCommandEncoder(descriptor: renderPassDescriptor) {

                renderEncoder.endEncoding()
                commandBuffer.present(currentDrawable)
                
            }

            commandBuffer.commit()
        }
        
    }
    
    // this is ran every frame
    func draw(in view: MTKView) {
        update()
    }
    
    func mtkView(_ view: MTKView, drawableSizeWillChange size: CGSize) {}
    
}


//- Tag: MetalViewMain
struct MetalViewMain: UIViewRepresentable {
    var mtkView: MTKView
    var arData: ARProvider
    
    func makeCoordinator() -> MTKCoordinator {
        MTKCoordinator(view: mtkView, arData: arData)
    }
    func makeUIView(context: UIViewRepresentableContext<MetalViewMain>) -> MTKView {
        mtkView.delegate = context.coordinator
        mtkView.preferredFramesPerSecond = 60
        mtkView.backgroundColor = context.environment.colorScheme == .dark ? .black : .white
        mtkView.isOpaque = true
        //mtkView.framebufferOnly = false
        mtkView.clearColor = MTLClearColor(red: 0, green: 0, blue: 0, alpha: 0)
        mtkView.drawableSize = mtkView.frame.size
        mtkView.enableSetNeedsDisplay = false
        mtkView.colorPixelFormat = .bgra8Unorm
        return mtkView
    }
    
    // `UIViewRepresentable` requires this implementation; however, the sample
    // app doesn't use it. Instead, `MTKView.delegate` handles display updates.
    func updateUIView(_ uiView: MTKView, context: UIViewRepresentableContext<MetalViewMain>) {
        
    }
}
