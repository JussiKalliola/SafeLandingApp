//
//  Extensions.swift
//  DigitalMicroAirport
//
//  Created by Jussi Kalliola (TAU) on 29.9.2022.
//

import Foundation
import SwiftUI
import Combine
import ARKit
import Accelerate
import MetalPerformanceShaders
import MetalKit

let arProvider = ARProvider.shared


// Enable `CVPixelBuffer` to output an `MTLTexture`.
extension CVPixelBuffer {
    
    func texture(withFormat pixelFormat: MTLPixelFormat, planeIndex: Int, addToCache cache: CVMetalTextureCache) -> MTLTexture? {
        let width =  CVPixelBufferGetWidthOfPlane(self, planeIndex)
        let height = CVPixelBufferGetHeightOfPlane(self, planeIndex)
        
        var cvtexture: CVMetalTexture?
        _ = CVMetalTextureCacheCreateTextureFromImage(nil, cache, self, nil, pixelFormat, width, height, planeIndex, &cvtexture)
        let texture = CVMetalTextureGetTexture(cvtexture!)
        
        return texture
        
    }
    
}

// Add next() function to CaseIterable
extension CaseIterable where Self: Equatable {
    func next() -> Self {
        let all = Self.allCases
        let idx = all.firstIndex(of: self)!
        let next = all.index(after: idx)
        return all[next == all.endIndex ? all.startIndex : next]
    }
}


extension UIApplication {
    
    var keyWindow: UIWindow? {
        // Get connected scenes
        return UIApplication.shared.connectedScenes
            // Keep only active scenes, onscreen and visible to the user
            .filter { $0.activationState == .foregroundActive }
            // Keep only the first `UIWindowScene`
            .first(where: { $0 is UIWindowScene })
            // Get its associated windows
            .flatMap({ $0 as? UIWindowScene })?.windows
            // Finally, keep only the key window
            .first(where: \.isKeyWindow)
    }
    
}


extension UIActivityViewController {
    override open func viewDidDisappear(_ animated: Bool) {
        arProvider.fileManager?.clearTempFolder()
        arProvider.fileCounter = 0
    }
}

/// MESH HELPERS

extension ARMeshGeometry {
    func vertex(at index: UInt32) -> SIMD3<Float> {
        assert(vertices.format == MTLVertexFormat.float3, "Expected three floats (twelve bytes) per vertex.")
        let vertexPointer = vertices.buffer.contents().advanced(by: vertices.offset + (vertices.stride * Int(index)))
        let vertex = vertexPointer.assumingMemoryBound(to: SIMD3<Float>.self).pointee
        return vertex
    }
}

extension matrix_float4x4 {
    func position() -> SCNVector3 {
        return SCNVector3(columns.3.x, columns.3.y, columns.3.z)
    }
}

// Convert TimeInterval into different types.
extension TimeInterval {
    var hourMinuteSecondMS: String {
        String(format:"%d:%02d:%02d.%03d", hour, minute, second, millisecond)
    }
    var minuteSecondMS: String {
        String(format:"%d:%02d.%03d", minute, second, millisecond)
    }
    var hour: Int {
        Int((self/3600).truncatingRemainder(dividingBy: 3600))
    }
    var minute: Int {
        Int((self/60).truncatingRemainder(dividingBy: 60))
    }
    var second: Int {
        Int(truncatingRemainder(dividingBy: 60))
    }
    var millisecond: Int {
        Int((self*1000).truncatingRemainder(dividingBy: 1000))
    }
}

extension Int {
    var msToSeconds: Double { Double(self) / 1000 }
}


extension UISegmentedControl {
  override open func didMoveToSuperview() {
     super.didMoveToSuperview()
     self.setContentHuggingPriority(.defaultLow, for: .vertical)
   }
}


extension Array {
    func contains<T>(obj: T) -> Bool where T: Equatable {
        return !self.filter({$0 as? T == obj}).isEmpty
    }
}


extension Comparable {
    func clamped(to limits: ClosedRange<Self>) -> Self {
        return min(max(self, limits.lowerBound), limits.upperBound)
    }
}


extension UIImage {

  func imageRotated(on degrees: CGFloat) -> UIImage {
    // Following code can only rotate images on 90, 180, 270.. degrees.
    let degrees = round(degrees / 90) * 90
    let sameOrientationType = Int(degrees) % 180 == 0
    let radians = CGFloat.pi * degrees / CGFloat(180)
    let newSize = sameOrientationType ? size : CGSize(width: size.height, height: size.width)

    UIGraphicsBeginImageContext(newSize)
    defer {
      UIGraphicsEndImageContext()
    }

    guard let ctx = UIGraphicsGetCurrentContext(), let cgImage = cgImage else {
      return self
    }

    ctx.translateBy(x: newSize.width / 2, y: newSize.height / 2)
    ctx.rotate(by: radians)
    ctx.scaleBy(x: 1, y: -1)
    let origin = CGPoint(x: -(size.width / 2), y: -(size.height / 2))
    let rect = CGRect(origin: origin, size: size)
    ctx.draw(cgImage, in: rect)
    let image = UIGraphicsGetImageFromCurrentImageContext()
    return image ?? self
  }

}



extension Float {
    /**
     Converts the float to an array of UInt8.
     With this method, it is possible to encode a float as bytes and later
     unpack the bytes to a float again. Note though that some of the precision
     is lost in the conversion.
     For instance, a conversion of 0.75 with the maxRange 1.0 results in the
     array `[233, 255, 255, 0]`. To convert the array back to a float, do the
     following calculation:
         (223 / 256 + 255 / 256 / 256 + 255 / 256 / 256 / 256) * (1.0 * 2.0) - 1.0 ≈
         0.8749999 * 2.0 - 1.0 ≈
         0.7499999
     A conversion of 23.1337 with the maxRange 100.0 results in the array
     `[157, 156, 114, 0]`. Converting it back:
         (157 / 256 + 156 / 256 / 256 + 114 / 256 / 256 / 256) * (100.0 * 2.0) - 100.0 ≈
         23.133683
     */
    func toUint8Array(maxRange: Float) -> [UInt8] {
        let max = (UInt32(UInt16.max) + 1) * UInt32(UInt32(UInt8.max) + 1) - 1
        let int = UInt32(((self / maxRange + 1.0) / 2.0 * Float(max)).rounded())
        let a = int.quotientAndRemainder(dividingBy: UInt32(UInt16.max) + 1)
        let b = a.remainder.quotientAndRemainder(dividingBy: UInt32(UInt8.max) + 1)
        return [UInt8(a.quotient), UInt8(b.quotient), UInt8(b.remainder), 0]
    }
}
