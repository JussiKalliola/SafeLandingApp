//
//  ContentView.swift
//  DigitalMicroAirport
//
//  Created by Jussi Kalliola (TAU) on 29.9.2022.
//

import SwiftUI
import RealityKit
import MetalKit
import SceneKit
import ARKit

enum ScanState: Int, CaseIterable {
    case rgb
    case depth
}

enum MeshVisualization: Int, CaseIterable {
    case mesh
    case pc
    case none
}

enum ProcessingState: String, CaseIterable {
    case VoxelFiltering = "Voxel Filtering"
    case RegionSegmentation = "Region segmentation"
    case SLAD = "Safe Landing Area Determination"
    case Saving = "Saving data..."
    case none
}

struct ContentView : View {
    @State var showMenu: Bool = false
    @State var navigationTitle: String = "Scan"
    
    @State var selectedConfidence = 2
    @State var colorSelection: Int = 0
    @State var pointSize: Double = 9.0
    
    @State var voxelLeafSize: Double = 0.02
    @State var showConfInCam: Bool = false
    @State var filteringPhase: Int = 1
    
    @State var useICP: Int = 0
    
    @State var K: Double = 30
    
    @State var regGrowSlope: Double = 3.0
    @State var regGrowCurPerc: Double = 0.98
    
    @State var minClusterSize: Double = 100
    @State var maxClusterSize: Double = 1000000
    
    @State var landingRadius: Double = 0.3
    @State var terCompSlope: Double = 3.0
    @State var terCompRough: Double = 0.05
    @State var terCompRel: Double = 0.03
    
    init() {
        setGlobalVariables(PCLGlobalVariables(K: Int32(K),
                                              minClusterSize: Int32(minClusterSize),
                                              maxClusterSize: Int32(maxClusterSize),
                                              curvatureThreshold: regGrowCurPerc,
                                              smoothnessThreshold: regGrowSlope,
                                              voxelLeafSize: Float(voxelLeafSize)))
    }
    
    
    var body: some View {
         
         return NavigationView {
             GeometryReader { geometry in
                 ZStack(alignment: .leading) {
                     MainView(colorSelection: $colorSelection,
                              selectedConfidence: $selectedConfidence,
                              pointSize: $pointSize,
                              voxelLeafSize: $voxelLeafSize,
                              showConfInCam: $showConfInCam,
                              filteringPhase: $filteringPhase,
                              useICP: $useICP,
                              K: $K,
                              regGrowSlope: $regGrowSlope,
                              regGrowCurPerc: $regGrowCurPerc,
                              minClusterSize: $minClusterSize,
                              maxClustersize: $maxClusterSize,
                              landingRadius: $landingRadius,
                              terCompSlope: $terCompSlope,
                              terCompRough: $terCompRough,
                              terCompRel: $terCompRel)
                         .frame(width: geometry.size.width, height: geometry.size.height)
                         .offset(x: self.showMenu ? geometry.size.width : 0)
                         .disabled(self.showMenu ? true : false)
                     if self.showMenu {
                         SideMenuView(colorSelection: $colorSelection,
                                      selectedConfidence: $selectedConfidence,
                                      pointSize: $pointSize,
                                      voxelLeafSize: $voxelLeafSize,
                                      showConfInCam: $showConfInCam,
                                      filteringPhase: $filteringPhase,
                                      useICP: $useICP,
                                      K: $K,
                                      regGrowSlope: $regGrowSlope,
                                      regGrowCurPerc: $regGrowCurPerc,
                                      minClusterSize: $minClusterSize,
                                      maxClustersize: $maxClusterSize,
                                      landingRadius: $landingRadius,
                                      terCompSlope: $terCompSlope,
                                      terCompRough: $terCompRough,
                                      terCompRel: $terCompRel)
                             .frame(width: geometry.size.width, height: geometry.size.height)
                             .transition(.move(edge: .leading))
                     }
                 }
             }
                 .navigationBarTitle(navigationTitle, displayMode: .inline)
                 .navigationBarItems(leading: (
                 Button(action: {
                     withAnimation {
                         self.showMenu.toggle()
                         if self.showMenu {
                             self.navigationTitle = "Settings"
                         } else {
                             self.navigationTitle = "Scan"
                             setGlobalVariables(PCLGlobalVariables(K: Int32(K),
                                                                   minClusterSize: Int32(minClusterSize),
                                                                   maxClusterSize: Int32(maxClusterSize),
                                                                   curvatureThreshold: regGrowCurPerc,
                                                                   smoothnessThreshold: regGrowSlope,
                                                                   voxelLeafSize: Float(voxelLeafSize)))
                         }
                     }
                 })
                 {
                     Image(systemName: "line.horizontal.3")
                         .frame(width: 35, height: 35)
                         .foregroundColor(.white)
                 }
             ))
         }
    }
}



#if DEBUG
struct ContentView_Previews : PreviewProvider {
    static var previews: some View {
        ContentView()
    }
}
#endif
