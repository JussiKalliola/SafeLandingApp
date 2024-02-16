//
//  MetalViewCamera.swift
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
final class CoordinatorCamera: MTKCoordinator {
    @Binding var confSelection: Int
    @Binding var visualization: MeshVisualization
    @Binding var cameraOn: Bool
    @Binding var pointcloudOffsetY: Float
    @Binding var pointSize: Double
    @Binding var colorSelection: Int
    @Binding var showConfInCam: Bool
    
    init(view: MTKView, arData: ARProvider, confSelection: Binding<Int>, visualization: Binding<MeshVisualization>, cameraOn: Binding<Bool>, pointcloudOffsetY: Binding<Float>, pointSize: Binding<Double>, colorSelection: Binding<Int>, showConfInCam: Binding<Bool>) {
        self._confSelection = confSelection
        self._visualization = visualization
        self._cameraOn = cameraOn
        self._pointcloudOffsetY = pointcloudOffsetY
        self._pointSize = pointSize
        self._colorSelection = colorSelection
        self._showConfInCam = showConfInCam
        
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
            let pointUniformBufferSize = kAlignedInstanceUniformsSize * kMaxBuffersInFlight

            frameUniformBuffer = metalDevice.makeBuffer(length: frameUniformBufferSize, options: .storageModeShared)
            fragmentUniformBuffer = metalDevice.makeBuffer(length: kAlignedFragmentUniformsSize, options: .storageModeShared)
            anchorUniformBuffer = metalDevice.makeBuffer(length: anchorUniformBufferSize, options: .storageModeShared)
            pointUniformBuffer = metalDevice.makeBuffer(length: pointUniformBufferSize, options: .storageModeShared)
            
            let imagePlaneVertexDataCount = kImagePlaneVertexData.count * MemoryLayout<Float>.size
            imagePlaneVertexBuffer = metalDevice.makeBuffer(bytes: kImagePlaneVertexData, length: imagePlaneVertexDataCount, options: [])

            let defaultLibrary = EnvironmentVariables.shared.metalLibrary
            //print(defaultLibrary)
            
            let cameraVertexFunction = defaultLibrary.makeFunction(name: "cameraVertexTransform")!
            let cameraFragmentFunction = defaultLibrary.makeFunction(name: "cameraFragmentShader")!

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
            
            prepareAnchorFunctions(metalDevice: metalDevice)
            preparePointcloudFunctions(metalDevice: metalDevice)

        } catch {
            print("Unexpected error: \(error).")
        }
    }
    
    // Setup anchor buffers also, since we want to render the mesh
    override func updateBufferStates() {
        uniformBufferIndex = (uniformBufferIndex + 1) % kMaxBuffersInFlight
        
        frameUniformBufferOffset = kAlignedFrameUniformsSize * uniformBufferIndex
        anchorUniformBufferOffset = kAlignedInstanceUniformsSize * uniformBufferIndex
        pointUniformBufferOffset = kAlignedInstanceUniformsSize * uniformBufferIndex
        
        frameUniformBufferAddress = frameUniformBuffer.contents().advanced(by: frameUniformBufferOffset)
        anchorUniformBufferAddress = anchorUniformBuffer.contents().advanced(by: anchorUniformBufferOffset)
        pointUniformBufferAddress = pointUniformBuffer.contents().advanced(by: pointUniformBufferOffset)
    }
    
    override func update() {
        guard arData.lastArData?.colorRGBTexture.texture != nil,
              arData.lastArData?.depthImageTexture != nil else {
            print("Nothing to display; skipping a draw.")
            return
        }
        
        
        if let commandBuffer = commandQueue.makeCommandBuffer() {
            updateBufferStates()
            updateFrameState()
            
            if visualization == .mesh {
                updateAnchors()
            }
            
            
            // Render rgb image
            if let renderPassDescriptor = view.currentRenderPassDescriptor, let currentDrawable = view.currentDrawable, let renderEncoder = commandBuffer.makeRenderCommandEncoder(descriptor: renderPassDescriptor) {
                if cameraOn {
                    
                    var confSelFilteres = (showConfInCam) ? confSelection : 0
                    renderEncoder.setCullMode(.none)
                    renderEncoder.setRenderPipelineState(cameraPipelineState)
                    renderEncoder.setDepthStencilState(cameraDepthState)
                    renderEncoder.setVertexBuffer(imagePlaneVertexBuffer, offset: 0, index: 0)
                    renderEncoder.setFragmentTexture(arData.lastArData?.colorRGBTexture.texture, index: 1)
                    renderEncoder.setFragmentTexture(arData.lastArData?.confidenceImageTexture.texture, index: 2)
                    renderEncoder.setFragmentTexture(arData.lastArData?.depthImageTexture.texture, index: 3)
                    renderEncoder.setFragmentBytes(&confSelFilteres, length: MemoryLayout<Int>.stride, index: 1)
                    //renderEncoder.set
                    renderEncoder.drawPrimitives(type: .triangleStrip, vertexStart: 0, vertexCount: 4)
                }
                
                if visualization == .pc {
                    drawPointcloudGeometry(renderEncoder: renderEncoder,
                                           pointcloudOffsetY: pointcloudOffsetY,
                                           pointSize: pointSize,
                                           colorSelection: $colorSelection.wrappedValue)
                } else if visualization == .mesh {
                    drawAnchorGeometry(renderEncoder: renderEncoder, pointcloudOffsetY: pointcloudOffsetY)
                }

                renderEncoder.endEncoding()
                commandBuffer.present(currentDrawable)
                
            }

            commandBuffer.commit()
        }
    }
}


//- Tag: MetalViewMain
struct MetalViewCamera: UIViewRepresentable {
    
    var mtkView: MTKView
    var arData: ARProvider
    @Binding var confSelection: Int
    @Binding var visualization: MeshVisualization
    @Binding var cameraOn: Bool
    @Binding var pointcloudOffsetY: Float
    @Binding var pointSize: Double
    @Binding var colorSelection: Int
    @Binding var showConfInCam: Bool

    
    func makeCoordinator() -> CoordinatorCamera {
        CoordinatorCamera(view: mtkView,
                          arData: arData,
                          confSelection: $confSelection,
                          visualization: $visualization,
                          cameraOn: $cameraOn,
                          pointcloudOffsetY: $pointcloudOffsetY,
                          pointSize: $pointSize,
                          colorSelection: $colorSelection,
                          showConfInCam: $showConfInCam)
    }
    
    func makeUIView(context: UIViewRepresentableContext<MetalViewCamera>) -> MTKView {
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
    func updateUIView(_ uiView: MTKView, context: UIViewRepresentableContext<MetalViewCamera>) {
        
    }
}
