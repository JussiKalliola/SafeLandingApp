//
//  MetalViewDepth.swift
//  DigitalMicroAirport
//
//  Created by Jussi Kalliola (TAU) on 29.9.2022.
//

import Foundation
import SwiftUI
import MetalKit
import Metal
import ARKit

// Inherit the main MTKCoordinator from MetalViewMain.
// Here prepare new pipelinestates, descriptors etc. required for printing camera texture.
final class CoordinatorDepth: MTKCoordinator {
    @Binding var confSelection: Int
    
    init(view: MTKView, arData: ARProvider, confSelection: Binding<Int>) {
        self._confSelection = confSelection
        super.init(view: view, arData: arData)
    }

    override func prepareFunctions() {
        
        guard let metalDevice = view.device else { fatalError("Expected a Metal device.") }
        
        do {
            view.depthStencilPixelFormat = .depth32Float_stencil8
            view.colorPixelFormat = .bgra8Unorm
            view.sampleCount = 1

            let frameUniformBufferSize = kAlignedFrameUniformsSize * kMaxBuffersInFlight
            let anchorUniformBufferSize = kAlignedInstanceUniformsSize * kMaxBuffersInFlight

            frameUniformBuffer = metalDevice.makeBuffer(length: frameUniformBufferSize, options: .storageModeShared)
            fragmentUniformBuffer = metalDevice.makeBuffer(length: kAlignedFragmentUniformsSize, options: .storageModeShared)
            
            let imagePlaneVertexDataCount = kImagePlaneVertexData.count * MemoryLayout<Float>.size
            imagePlaneVertexBuffer = metalDevice.makeBuffer(bytes: kImagePlaneVertexData, length: imagePlaneVertexDataCount, options: [])

            let defaultLibrary = EnvironmentVariables.shared.metalLibrary
            //print(defaultLibrary)
            
            let cameraVertexFunction = defaultLibrary.makeFunction(name: "cameraVertexTransform")!
            let cameraFragmentFunction = defaultLibrary.makeFunction(name: "depthFragmentShader")!

            let imagePlaneVertexDescriptor = MTLVertexDescriptor()
            imagePlaneVertexDescriptor.attributes[0].format = .float2
            imagePlaneVertexDescriptor.attributes[0].offset = 0
            imagePlaneVertexDescriptor.attributes[0].bufferIndex = 0
            imagePlaneVertexDescriptor.attributes[1].format = .float2
            imagePlaneVertexDescriptor.attributes[1].offset = 8
            imagePlaneVertexDescriptor.attributes[1].bufferIndex = 0
            imagePlaneVertexDescriptor.layouts[0].stride = 16

            let cameraPipelineStateDescriptor = MTLRenderPipelineDescriptor()
            cameraPipelineStateDescriptor.sampleCount = view.sampleCount
            cameraPipelineStateDescriptor.vertexFunction = cameraVertexFunction
            cameraPipelineStateDescriptor.fragmentFunction = cameraFragmentFunction
            cameraPipelineStateDescriptor.vertexDescriptor = imagePlaneVertexDescriptor
            cameraPipelineStateDescriptor.colorAttachments[0].pixelFormat = view.colorPixelFormat
            cameraPipelineStateDescriptor.depthAttachmentPixelFormat = view.depthStencilPixelFormat
            cameraPipelineStateDescriptor.stencilAttachmentPixelFormat = view.depthStencilPixelFormat
            
            do {
                try cameraPipelineState = metalDevice.makeRenderPipelineState(descriptor: cameraPipelineStateDescriptor)
            } catch let error {
                fatalError("Failed to created captured image pipeline state, error \(error)")
            }

            let cameraDepthStateDescriptor = MTLDepthStencilDescriptor()
            cameraDepthStateDescriptor.depthCompareFunction = .always
            cameraDepthStateDescriptor.isDepthWriteEnabled = false
            cameraDepthState = metalDevice.makeDepthStencilState(descriptor: cameraDepthStateDescriptor)

        } catch {
            print("Unexpected error: \(error).")
        }
    }
    
    override func update() {
        guard arData.lastArData?.depthImageTexture != nil else {
            print("Nothing to display; skipping a draw.")
            return
        }
        
        
        if let commandBuffer = commandQueue.makeCommandBuffer() {
            updateBufferStates()
            updateFrameState()
            
            // Render rgb image
            if let renderPassDescriptor = view.currentRenderPassDescriptor, let currentDrawable = view.currentDrawable, let renderEncoder = commandBuffer.makeRenderCommandEncoder(descriptor: renderPassDescriptor) {
                
                renderEncoder.setCullMode(.none)
                renderEncoder.setRenderPipelineState(cameraPipelineState)
                renderEncoder.setDepthStencilState(cameraDepthState)
                renderEncoder.setVertexBuffer(imagePlaneVertexBuffer, offset: 0, index: 0)
                renderEncoder.setFragmentTexture(arData.lastArData?.depthImageTexture.texture, index: 1)
                renderEncoder.setFragmentTexture(arData.lastArData?.confidenceImageTexture.texture, index: 2)
                renderEncoder.setFragmentBytes(&confSelection, length: MemoryLayout<Int>.stride, index: 1)
                renderEncoder.drawPrimitives(type: .triangleStrip, vertexStart: 0, vertexCount: 4)

                renderEncoder.endEncoding()
                commandBuffer.present(currentDrawable)
                
            }

            commandBuffer.commit()
        }
    }
}


//- Tag: MetalViewMain
struct MetalViewDepth: UIViewRepresentable {
    
    var mtkView: MTKView
    var arData: ARProvider
    @Binding var confSelection: Int
    
    func makeCoordinator() -> CoordinatorDepth {
        CoordinatorDepth(view: mtkView, arData: arData, confSelection: $confSelection)
    }
    
    func makeUIView(context: UIViewRepresentableContext<MetalViewDepth>) -> MTKView {
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
    func updateUIView(_ uiView: MTKView, context: UIViewRepresentableContext<MetalViewDepth>) {
        
    }
}
