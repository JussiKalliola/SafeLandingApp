//
//  MainView.swift
//  DigitalMicroAirport
//
//  Created by Jussi Kalliola (TAU) on 29.10.2022.
//

import SwiftUI
import RealityKit
import MetalKit
import SceneKit
import ARKit

struct MainView: View {
    // Manage the AR session and AR data processing.
    //- Tag: ARProvider
    @ObservedObject var arProvider: ARProvider = ARProvider.shared
    
    @State var state: ScanState = .rgb
    @State var meshVisualization: MeshVisualization = .pc
    
    @State var processingState: ProcessingState = .none
    
    // Confidence map variables
    @Binding var colorSelection: Int
    @Binding var selectedConfidence: Int
    @Binding var pointSize: Double
    
    
    @Binding var voxelLeafSize: Double
    @Binding var showConfInCam: Bool
    @Binding var filteringPhase: Int
    
    @Binding var useICP: Int
    
    @Binding var K: Double
    
    @Binding var regGrowSlope: Double
    @Binding var regGrowCurPerc: Double
    
    @Binding var minClusterSize: Double
    @Binding var maxClustersize: Double
    
    @Binding var landingRadius: Double
    @Binding var terCompSlope: Double
    @Binding var terCompRough: Double
    @Binding var terCompRel: Double
    
    @Binding var useRos: Bool
    @Binding var rosIpAddress: String
    
    
    var confLevels = [0, 1, 2]
    
    var btnSize: CGFloat = 30.0
    
    @State var pointCloudProcessState: Int = 0
    
    @State private var showingAlert = false
    
    @State var showPointcloud: Bool = false
    @State var cameraOn: Bool = true
    
    @State var offsetY: Float = 0.0
    @State var tempOffsetY: Float = 0.0
    
    @State var prevDragY: Float = 0.0
    @State var dragIdx: Int = 0
    @State var dragging: Bool = false
    
    
    let queue = DispatchQueue(label: "PCProcessing", qos: .userInitiated)
    
    var body: some View {
        VStack {
            let dragGesture = DragGesture(minimumDistance: 0)
                .onChanged({gesture in
                    
                    var curDragY = (Float) (-1 * gesture.translation.height)
                    
                    if !dragging {
                        prevDragY = curDragY
                    }
                    
                    if curDragY < prevDragY {
                        offsetY -= abs(curDragY) / 3000
                    } else {
                        offsetY += abs(curDragY) / 3000
                    }
                    
                    dragging = true

                })
                .onEnded({_ in
                    offsetY = 0.0
                    dragging = false
                    prevDragY = 0.0
                    
                })

            let tapGesture = LongPressGesture(minimumDuration: 0.2).onEnded({_ in
                //state=state.next()
                if arProvider.scanState {
                    arProvider.useICP = useICP
                    arProvider.confSelection = selectedConfidence
                    arProvider.captureFrameData(filteringPhase: filteringPhase, projectDepthMap: true)
                } else {
                    state=state.next()
                }
            })
            
            let combined = dragGesture.sequenced(before: tapGesture)
            
            let simultaneously = dragGesture.simultaneously(with: tapGesture)
            
            ZStack(alignment: .bottom) {
                switch state {
                case .rgb:
                    
                    MetalViewCamera(mtkView: MTKView(),
                                    arData: arProvider,
                                    confSelection: $selectedConfidence,
                                    visualization: $meshVisualization,
                                    cameraOn: $cameraOn,
                                    pointcloudOffsetY: $offsetY,
                                    pointSize: $pointSize,
                                    colorSelection: $colorSelection,
                                    showConfInCam: $showConfInCam)
                    .gesture(tapGesture)
                    .zIndex(-1)

                case .depth:
                    MetalViewDepth(mtkView: MTKView(),
                                   arData: arProvider,
                                   confSelection: $selectedConfidence).onTapGesture {
                        state=state.next()
                    }

                default:
                    MetalViewCamera(mtkView: MTKView(),
                                    arData: arProvider,
                                    confSelection: $selectedConfidence,
                                    visualization: $meshVisualization,
                                    cameraOn: $cameraOn,
                                    pointcloudOffsetY: $offsetY,
                                    pointSize: $pointSize,
                                    colorSelection: $colorSelection,
                                    showConfInCam: $showConfInCam).onTapGesture {
                        state=state.next()
                    }
                }
                
                HStack {
                    if colorSelection == 1 {
                        PointClassView()
                    }
                    
                    Spacer()
                    
                    // Show statistics in the bottom right corner of the video feed.
                    VStack {
                        Spacer()
                        Text("\(arProvider.lastArData?.worldMeshes.count ?? 0) Meshes ").font(.system(size: 12))
                        Text("\((arProvider.lastArData?.finalPointCloud.count ?? 0) > 1000 ? ((arProvider.lastArData?.finalPointCloud.count ?? 0) / 1000) : (arProvider.lastArData?.finalPointCloud.count ?? 0)) \((arProvider.lastArData?.finalPointCloud.count ?? 0) > 1000 ? "K" : "") Points").font(.system(size: 12))
                        Text("\(arProvider.fileCounter) Images").font(.system(size: 12))
                        
                    }.zIndex(3)
                }.frame(height:100, alignment: .bottom).padding([.horizontal, .vertical], 10)
                if(arProvider.websocket != nil) {
                    HStack{

                        Circle()
                            .fill(arProvider.websocket!.connected ? .green : .red)
                            .frame(width: 10, height: 10)
                        Text(arProvider.websocket!.connectionStatus)
                    }.frame(height:20, alignment: .bottom).padding([.horizontal, .vertical], 10)
                }
                
            }
            
            VStack {
                HStack {
                    
                    //-------------- Test websocket send. --------------
//                    Button(action: {
//                    
//                        arProvider.websocket?.publish(json: ["op": "publish", "topic": "/test_topic", "msg": ["data": "test msg."]])
//
//                        
//                    }) {
//                        VStack {
//                            Image(systemName: "message")
//                                .resizable()
//                                .frame(width: btnSize, height: btnSize)
//                                .foregroundColor(.white)
//                            Text("Send").font(.system(size: 11)).foregroundColor(.white)
//                        }
//                    }.padding([.horizontal], 20)
//                     .disabled(processingState != .none)
                    
                    
                    
                    //-------------- Start capturing data. --------------
                    Button(action: {
                        
                        arProvider.scanState.toggle()
                        pointCloudProcessState = 0
                        
                        if arProvider.scanState {
                            arProvider.confSelection = selectedConfidence
                            if(self.useRos) {
                                arProvider.websocket = WebSocket(url: self.rosIpAddress)
                            }
                            //arProvider.startTimer()
                        } else {
                            //arProvider.stopTimer()
                        }
                        
                        
                    }) {
                        arProvider.scanState ?
                        VStack {
                            Image(systemName: "stop.circle")
                                .resizable()
                                .frame(width: btnSize, height: btnSize)
                                .foregroundColor(.red)
                            Text("Stop").font(.system(size: 11)).foregroundColor(.red)
                        } :
                        VStack {
                            Image(systemName: "play.circle")
                                .resizable()
                                .frame(width: btnSize, height: btnSize)
                                .foregroundColor((processingState == .none) ? .white : .gray)
                            Text("Scan").font(.system(size: 11)).foregroundColor((processingState == .none) ? .white : .gray)
                        }
                        
                    }.padding([.horizontal], 20)
                     .disabled(processingState != .none)
                    //-----------------------------------------------------
                    
                    
                    
                    //-------------- Turn on/off camera feed --------------
                    Button(action: {
                        cameraOn.toggle()
                    }) {
                        VStack {
                            Image(systemName: "camera.circle")
                                .resizable()
                                .frame(width: btnSize, height: btnSize)
                                .foregroundColor((cameraOn) ? .red : .white)
                            Text((cameraOn) ? "Off" : "On").font(.system(size: 11)).foregroundColor((cameraOn) ? .red : .white)
                        }
                        
                    }.padding([.horizontal], 20)
                    //-----------------------------------------------------
                    
                    
                    // -------------- Change the visualization to pointcloud --------------
                    Button(action: {
                        if meshVisualization == .pc {
                            meshVisualization = .mesh
                        } else {
                            meshVisualization = .pc
                        }
                    }) {
                        (meshVisualization == .pc) ?
                        VStack {
                            Image(systemName: "circle.grid.cross")
                                .resizable()
                                .frame(width: btnSize, height: btnSize)
                                .foregroundColor(.white)
                            Text("PointCloud").font(.system(size: 11)).foregroundColor(.white)
                        }
                        : VStack {
                            Image(systemName: "rotate.3d")
                                .resizable()
                                .frame(width: btnSize, height: btnSize)
                                .foregroundColor(.white)
                            Text("Mesh").font(.system(size: 11)).foregroundColor(.white)
                        }
                        
                    }.padding([.horizontal], 20)
                    //-----------------------------------------------------
                    
                    // Buttons which are visible if some scanning has been done.
                    if !arProvider.scanState && (arProvider.lastArData?.finalPointCloud.count ?? 0) > 0 {
                        
                        //-------------- Save data --------------
                        Button(action: {
                            //arProvider.stopTimer()
                            showingAlert = true
                        }) {
                            VStack {
                                Image(systemName: "square.and.arrow.down.fill")
                                    .resizable()
                                    .frame(width: btnSize, height: btnSize)
                                    .foregroundColor((processingState == .none) ? .white : .gray)
                                Text("Save").font(.system(size: 11)).foregroundColor((processingState == .none) ? .white : .gray)
                            }
                        }.padding([.horizontal], 20)
                        .disabled(processingState != .none)
//                                .foregroundColor(Color.white)
//                                .border(.gray)
                        .alert("Are you sure you want to save the data?", isPresented: $showingAlert) {
                            Button("Save", action: shareFiles)
                            Button("Cancel", role: .cancel) { processingState = .none }
                        } message: {
                            Text("Continuing will delete all your saved files, even if saving is cancelled.")
                        }
                        //-----------------------------------------------------
                    }
                }.disabled(processingState == .Saving)
                
                
                // Buttons which are visible if some scanning has been done.
                if !arProvider.scanState && (arProvider.lastArData?.finalPointCloud.count ?? 0) > 0 {
                    VStack {
                        Divider().padding([.vertical], 5)
                        HStack {
                            //-------------- Filter the pointcloud --------------
                            Button(action: {
                                
                                if !arProvider.scanState && arProvider.lastArData!.finalPointCloud.count > 0 {
                                    
                                    processingState = ProcessingState.VoxelFiltering
                                    
                                    // Perform in another thread so that the main arkit thread is not affected.
                                    queue.async {
                                        colorSelection = 0
                                        var filteredPointCloud = voxelGridSampling(pointCloud: arProvider.lastArData!.finalPointCloud)
                                        arProvider.initRenderVertexBuffer(pointCloud: filteredPointCloud)
                                        pointCloudProcessState = 1
                                        processingState = ProcessingState.none
                                    }
                                }
                                
                
                            }) {
                                VStack {
                                    Image(systemName: "camera.filters")
                                        .resizable()
                                        .frame(width: btnSize, height: btnSize)
                                        .foregroundColor((processingState != .none) ? .gray : .white)
                                    Text("Voxel filter").font(.system(size: 11)).foregroundColor((processingState != .none) ? .gray : .white)
                                }
                                
                            }.padding([.horizontal], 20)
                             .disabled(processingState != .none)
                            //-----------------------------------------------------
                            
                            
                            //-------------- Region clustering --------------
                            Button(action: {
                                
                                if !arProvider.scanState && arProvider.lastArData!.finalPointCloud.count > 0 {
                                    
                                    processingState = ProcessingState.RegionSegmentation
                                    
                                    // Perform in another thread so that the main arkit thread is not affected.
                                    queue.async {
                                        if pointCloudProcessState == 0 {
                                            initializeKDTree()
                                        }
                                        
                                        var clusteredPointCloud = regionGrowingSegmentation(pointCloud: arProvider.lastArData!.finalPointCloud, percentile: regGrowCurPerc)
                                        arProvider.initRenderVertexBuffer(pointCloud: clusteredPointCloud)
                                        colorSelection = 3
                                        pointCloudProcessState = 2
                                        processingState = ProcessingState.none
                                    }
                                }
                                
                
                            }) {
                                VStack {
                                    Image(systemName: "rectangle.3.group")
                                        .resizable()
                                        .frame(width: btnSize, height: btnSize)
                                        .foregroundColor((pointCloudProcessState > 0 && processingState == .none) ? .white : .gray)
                                    Text("Cluster").font(.system(size: 11)).foregroundColor((pointCloudProcessState > 0 && processingState == .none) ? .white : .gray)
                                }
                                
                            }.padding([.horizontal], 20)
                             .disabled(pointCloudProcessState < 1)
                            //-----------------------------------------------------
                            
                            
                            //-------------- Safe Landing area determination --------------
                            Button(action: {
                                
                                if !arProvider.scanState && arProvider.lastArData!.finalPointCloud.count > 0 {
                                    
                                    // Perform in another thread so that the main arkit thread is not affected.
                                    queue.async {
                                        colorSelection = 1
                                        processingState = ProcessingState.SLAD
                                        safeLandingAreaDetermination(pointCloud: &arProvider.lastArData!.finalPointCloud,
                                                                              landingRadius: landingRadius,
                                                                              voxelSize: voxelLeafSize,
                                                                              slopeThreshold: terCompSlope,
                                                                              roughThreshold: terCompRough,
                                                                              relThreshold: terCompRel)
                                        
                                        
                                        print("This is done 3")
                                        //DispatchQueue.main.async {
                                        arProvider.initRenderVertexBuffer(pointCloud: arProvider.lastArData!.finalPointCloud)
                                        processingState = ProcessingState.none
                                        print("This is done 4")
                                        //}
                                    }
                                }
                
                            }) {
                                VStack {
                                    Image(systemName: "circle.circle")
                                        .resizable()
                                        .frame(width: btnSize, height: btnSize)
                                        .foregroundColor((pointCloudProcessState > 1 && processingState == .none) ? .white : .gray)
                                    Text("SLAD").font(.system(size: 11)).foregroundColor((pointCloudProcessState > 1 && processingState == .none) ? .white : .gray)
                                }
                                
                            }.padding([.horizontal], 20)
                             .disabled(pointCloudProcessState < 2)
                            //-----------------------------------------------------
                            
                            //-------------- Reset the pointcloud --------------
                            Button(action: {
                                
                                if !arProvider.scanState && arProvider.lastArData!.finalPointCloud.count > 0 && pointCloudProcessState > 0 {
                                    
                                    queue.async {
                                        print("Resetting the pointcloud")
                                        colorSelection = 0
                                        pointCloudProcessState = 0
                                        let pc = resetPointCloud()
                                        
                                        arProvider.initRenderVertexBuffer(pointCloud: pc)
                                    }
                                }
                
                            }) {
                                VStack {
                                    Image(systemName: "arrow.uturn.backward.circle")
                                        .resizable()
                                        .frame(width: btnSize, height: btnSize)
                                        .foregroundColor((pointCloudProcessState > 0 && processingState == .none) ? .red : .gray)
                                    Text("Reset").font(.system(size: 11)).foregroundColor((pointCloudProcessState > 0 && processingState == .none) ? .red : .gray)
                                }
                                
                            }.padding([.horizontal], 20)
                             .disabled(pointCloudProcessState < 1)
                            //-----------------------------------------------------
                            
                        }.disabled(processingState != .none)
                        Text("Processing: \(processingState.rawValue)").font(.system(size: 11)).foregroundColor((processingState == .none) ? .white : .red).padding([.top], 10)
                    }.disabled(processingState == .Saving)
                }
                
            }.padding(.all)

        }.edgesIgnoringSafeArea(.top)
    }
    
    func shareFiles() {
        
        //arProvider.pause()
        queue.async {
            processingState = .Saving
            let operationQueue = OperationQueue()
            //operationQueue.maxConcurrentOperationCount = 8
    
            do {
                
                let savedFrames = arProvider.savedFrames
                var cameraPoses: [CameraPose] = []
                for frame in savedFrames {
                    let sourceImgPath = arProvider.fileManager!.path!.path + "/" + frame.rgbPath
                    let targetImgPath = arProvider.fileManager!.path!.path + "/mapping/" + frame.rgbPath.dropLast(4) + ".jpeg"
                    
                    //let sourceDepthPath = arProvider.fileManager!.path!.path + "/" + frame.depthPath
                    //let targetDepthPath = arProvider.fileManager!.path!.path + "/depth/" + frame.depthPath.dropLast(4) + ".jpeg"
                    
                    //let sourceConfPath = arProvider.fileManager!.path!.path + "/" + frame.confPath
                    //let targetConfPath = arProvider.fileManager!.path!.path + "/conf/" + frame.confPath.dropLast(4) + ".jpeg"
                    
                    cameraPoses.append(frame.pose)
                    
                    //operationQueue.addOperation {
                        //arProvider.fileManager?.convertBinToJpeg(sourceFilePath: sourceImgPath,
                        //                                         targetFilePath: targetImgPath,
                        //                                         width: frame.rgbResolution[1],
                        //                                         height: frame.rgbResolution[0],
                        //                                         orientation: frame.orientation)
                        
                        //arProvider.fileManager?.convertBinToJpeg(sourceFilePath: sourceDepthPath,
                        //                                         targetFilePath: targetDepthPath,
                        //                                         width: frame.depthResolution[1],
                        //                                         height: frame.depthResolution[0],
                        //                                         orientation: frame.orientation)
                        
                        arProvider.fileManager!.moveFileToFolder(sourcePath: arProvider.fileManager!.path!.path + "/" + frame.depthPathBin,
                                                                 targetPath: arProvider.fileManager!.path!.path + "/depth/" + frame.depthPathBin)
                        arProvider.fileManager!.moveFileToFolder(sourcePath: sourceImgPath,
                                                                 targetPath: arProvider.fileManager!.path!.path + "/mapping/" + frame.rgbPath)
                        
                        //arProvider.fileManager?.convertBinToJpeg(sourceFilePath: sourceConfPath,
                        //                                         targetFilePath: targetConfPath,
                        //                                         width: frame.confResolution[1],
                        //                                         height: frame.confResolution[0],
                        //                                         orientation: frame.orientation)
                        
                        arProvider.fileManager!.moveFileToFolder(sourcePath: arProvider.fileManager!.path!.path + "/" + frame.confPathBin,
                                                                 targetPath: arProvider.fileManager!.path!.path + "/confidence/" + frame.confPathBin)
                    }
                //}
                
//                let filePaths = try arProvider.fileManager!.fileManager.contentsOfDirectory(atPath: arProvider.fileManager!.path!.path)
//                print(filePaths)
                
//                for filePath in filePaths {
//                    let sourceFilePath = arProvider.fileManager!.path!.path + "/" + filePath
//                    let targetImageFilePath = arProvider.fileManager!.path!.path + "/rgb/" + filePath.dropLast(4) + ".jpeg"
//                    let targetDepthFilePath = arProvider.fileManager!.path!.path + "/depth/" + filePath.dropLast(4) + ".jpeg"
//                    let targetFramePLYFilePath = arProvider.fileManager!.path!.path + "/depth/"
//
//
//                    operationQueue.addOperation {
//                        if filePath.contains(".bin") && filePath.contains("rgb") {
//
//                            if filePath.contains("downscaled") {
//                                arProvider.fileManager?.convertBinToJpeg(sourceFilePath: sourceFilePath,
//                                                                         targetFilePath: targetImageFilePath,
//                                                                         width: 256,
//                                                                         height: 192)
//                            } else {
//                                arProvider.fileManager?.convertBinToJpeg(sourceFilePath: sourceFilePath,
//                                                                         targetFilePath: targetImageFilePath,
//                                                                         width: 1920,
//                                                                         height: 1440)
//                            }
//
//                        } else if filePath.contains(".bin") && (filePath.contains("depth") || filePath.contains("conf")) {
//                            arProvider.fileManager?.convertBinToJpeg(sourceFilePath: sourceFilePath,
//                                                                     targetFilePath: targetDepthFilePath,
//                                                                     width: 256,
//                                                                     height: 192)
//                        } else if filePath.contains(".bin") && (filePath.contains("pointcloud")) {
//                            arProvider.fileManager?.convertBinToPly(sourceFilePath: sourceFilePath,
//                                                                    targetFilePath: targetFramePLYFilePath,
//                                                                    targetFileName: filePath.dropLast(4) + ".ply")
//                        }
//                    }
//                }
                
                if arProvider.lastArData?.arAnchors != nil {
                    //operationQueue.addOperation {
                        arProvider.fileManager?.processMesh(arAnchors: arProvider.lastArData!.arAnchors)
                    //}
                    //operationQueue.addOperation {
                        arProvider.fileManager!.processPointCloud(pointCloud: arProvider.lastArData!.originalPointCloud, filteredPointCloud: arProvider.lastArData!.finalPointCloud)
                    //}
                }
                var params = Parameters(selectedConfidence: self.selectedConfidence,
                                        colorSelection: self.colorSelection,
                                        pointSize: self.pointSize,
                                        voxelLeafSize: self.voxelLeafSize,
                                        filteringPhase: self.filteringPhase,
                                        useICP: self.useICP,
                                        K: self.K,
                                        regGrowSlope: self.regGrowSlope,
                                        regGrowCurPerc: self.regGrowCurPerc,
                                        minClusterSize: self.minClusterSize,
                                        maxClusterSize: self.maxClustersize,
                                        landingRadius: self.landingRadius,
                                        terCompSlope: self.terCompSlope,
                                        terCompRough: self.terCompRough,
                                        terCompRel: self.terCompRel,
                                        originGps: arProvider.arReceiver.originGps!)
                
                arProvider.fileManager?.writeParameters(params: params)
                
                //operationQueue.addOperation {
                    arProvider.fileManager?.writeCameraPose(cameraPoses: cameraPoses,
                                                            sampleTimes: arProvider.lastArData!.sampleTimes )
                //}
            } catch {
                print("error")
            }
            
            
            //operationQueue.waitUntilAllOperationsAreFinished()
            
            //operationQueue.addOperation {
                if arProvider.rgbNames.count > 0 {
                    arProvider.fileManager?.writeColmapFiles(cameraPoses: arProvider.lastArData!.cameraPoses,
                                                             fileNames: arProvider.rgbNames)
                }
            //}
            
            operationQueue.waitUntilAllOperationsAreFinished()
            
            
            DispatchQueue.main.async {
                var filesToShare = [Any]()
                filesToShare.append(arProvider.fileManager!.path)
                let av = UIActivityViewController(activityItems: filesToShare, applicationActivities: nil)

                UIApplication.shared.keyWindow?.rootViewController?.present(av, animated: true, completion: nil)
                arProvider.resetArData()
                processingState = .none
            }
        }
    }
}

//struct MainView_Previews: PreviewProvider {
//    static var previews: some View {
//        MainView()
//    }
//}
