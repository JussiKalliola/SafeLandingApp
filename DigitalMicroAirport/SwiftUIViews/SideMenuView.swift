//
//  SideMenuView.swift
//  DigitalMicroAirport
//
//  Created by Jussi Kalliola (TAU) on 29.10.2022.
//

import SwiftUI

struct SideMenuView: View {
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
    
    @State var tempRosIpAddress: String = ""
    
    var body: some View {
        ScrollView(.vertical) {
            VStack {
                // ROS parameters.
                VStack(alignment: .leading) {
                    Text("ROS").font(.title2).padding([.vertical], 10)
                    Text("ROS Starts to send updates to the ROS network when scan is started. ROS Topics:").font(.footnote)
                    Text("/iphone/pose").padding(.leading, 5).padding(.top, 1).font(.system(size: 11))
                    Text("/iphone/rgb/image_raw").padding(.leading, 5).font(.system(size: 11))
                    Text("/iphone/depth/image_raw").padding(.leading, 5).font(.system(size: 11))
                    Text("/iphone/confidence/image_raw").padding(.leading, 5).padding(.bottom, 3).font(.system(size: 11))
                    Text("[More info in github.](https://github.com/JussiKalliola/SafeLandingApp)").font(.footnote)
                    Toggle("Enable ROS:", isOn: $useRos).disabled($rosIpAddress.wrappedValue != "")
                    
                    HStack {
                        TextField(
                            "URL",
                            text: $tempRosIpAddress,
                            prompt: Text("ws://0.0.0.0:9090").foregroundColor($useRos.wrappedValue ? .white : .gray)
                        ).textFieldStyle(.roundedBorder).disabled($useRos.wrappedValue && $rosIpAddress.wrappedValue != "")
                            .autocorrectionDisabled()
                            .border($useRos.wrappedValue ? .white : .gray)
                        
                        if $rosIpAddress.wrappedValue == "" {
                            Button(action: {
                                // Here we should do some checking if its an actual IP, TODO
                                $rosIpAddress.wrappedValue = $tempRosIpAddress.wrappedValue
                                
                            }) {
                                Text("Save").font(.system(size: 11)).foregroundColor($useRos.wrappedValue ? .white : .gray)
                                
                            }.padding([.horizontal], 20).padding([.vertical], 10).buttonBorderShape(.roundedRectangle(radius: 10)).border($useRos.wrappedValue ? .white : .gray)
                        } else {
                            Button(action: {
                                // Here we should do some checking if its an actual IP, TODO
                                $rosIpAddress.wrappedValue = ""
                                $tempRosIpAddress.wrappedValue = ""
                                
                            }) {
                                Text("Delete").font(.system(size: 11)).foregroundColor(.white)
                                
                            }.padding([.horizontal], 20).padding([.vertical], 10).buttonBorderShape(.roundedRectangle(radius: 10)).border(.red).background(.red)
                        }
                         //.disabled(pointCloudProcessState < 1)
                    }.disabled(!$useRos.wrappedValue)
                }.padding([.top], 20)
                
                Divider().foregroundColor(.gray).padding([.top, .bottom], 10)
                
                VStack(alignment: .leading) {
                    Text("Rendering").font(.title2).padding([.vertical], 10)
                    Text("Pointcloud Color")
                    Picker(selection: $colorSelection, label: Text("Pointcloud Color")) {
                        Text("Color").tag(0)
                        Text("Class").tag(1)
                        Text("Curvature").tag(2)
                        Text("Region").tag(3)

                    }.frame(height: 20).padding([.vertical], 5).pickerStyle(SegmentedPickerStyle())
                    
                    // Confidence selection
                    Text("Confidence select")
                    Picker(selection: $selectedConfidence, label: Text("Confidence select")) {
                        Text("0").tag(0)
                        Text("1").tag(1)
                        Text("2").tag(2)

                    }.frame(height: 20).padding([.vertical], 5).pickerStyle(SegmentedPickerStyle())
                    Toggle("Filter camera feed with confidence map", isOn: $showConfInCam)
                    Text("Point size")
                    HStack {
                        Slider(
                            value: $pointSize,
                            in: 0.0...20.0,
                            step: 1.0,
                            onEditingChanged: { _ in
                                print("")
                            }
                        ).padding([.vertical], 0)
                        Text("\( String(format: "%g", pointSize)) size")
                            .foregroundColor(.white).padding([.vertical], 0).frame(width: 120, alignment: .trailing)
                    }
                }
                
                Divider().foregroundColor(.gray).padding([.top, .bottom], 10)
                
                // Global settings for pointclouds.
                VStack(alignment: .leading) {
                    Text("Global PointCloud").font(.title2).padding([.vertical], 10)
                    
                    Text("K nearest neighbors")
                    HStack {
                        Slider(
                            value: $K,
                            in: 0.0...200.0,
                            step: 10.0,
                            onEditingChanged: { _ in
                                print("")
                            }
                        ).padding([.vertical], 0)
                        Text("\( String(format: "%g", K)) points")
                            .foregroundColor(.white).padding([.vertical], 0).frame(width: 120, alignment: .trailing)
                    }
                    
                }
                
                Divider().foregroundColor(.gray).padding([.top, .bottom], 10)
                
                // Pointcloud downsample slider.
                VStack(alignment: .leading) {
                    Text("Pointcloud filtering").font(.title2).padding([.vertical], 10)
                    Text("Use Iterative Closest Point")
                    Picker(selection: $useICP, label: Text("Use Iterative Closest Point")) {
                        Text("False").tag(0)
                        Text("True").tag(1)
                    }.frame(height: 20).padding([.vertical], 5).pickerStyle(SegmentedPickerStyle())
                    Text("Voxel size")
                    HStack {
                        Slider(
                            value: $voxelLeafSize,
                            in: 0.0...0.1,
                            step: 0.001,
                            onEditingChanged: { _ in
                                print("")
                            }
                        ).padding([.vertical], 0).disabled($filteringPhase.wrappedValue == 2).id($filteringPhase.wrappedValue != 2)
                        Text("\( String(format: "%g", voxelLeafSize*100)) cm")
                            .foregroundColor(.white).padding([.vertical], 0).frame(width: 70, alignment: .trailing)
                    }
                }
                
                Divider().foregroundColor(.gray).padding([.top, .bottom], 10)
                
                // Safe landing area determination parameters.
                VStack(alignment: .leading) {
                    Text("Safe Landing Area Determination").font(.title2).padding([.vertical], 10)
                    Text("Region growing").font(.title3).padding([.vertical], 5)
                    
                    
                    Text("Cluster size")
                    HStack {
                        VStack {
                            Text("Minimum").font(.system(size: 14))
                            Slider(
                                value: $minClusterSize,
                                in: 1.0...1000.0,
                                step: 50.0,
                                onEditingChanged: { _ in
                                    print("")
                                }
                            ).padding([.vertical], 0)
                            Text("\( String(format: "%g", minClusterSize)) points")
                                .foregroundColor(.white).padding([.vertical], 0).frame(alignment: .leading)
                        }.frame(alignment: .leading)
                        
                        VStack {
                            Text("Maximum").font(.system(size: 14))
                            Slider(
                                value: $maxClustersize,
                                in: 1000.0...1000000.0,
                                step: 10000.0,
                                onEditingChanged: { _ in
                                    print("")
                                }
                            ).padding([.vertical], 0)
                            Text("\( String(format: "%g", maxClustersize)) points")
                                .foregroundColor(.white).padding([.vertical], 0).frame(alignment: .leading)
                        }.frame(alignment: .leading)
                    }.padding([.vertical], 10)
                    
                    Text("Slope angle")
                    HStack {
                        Slider(
                            value: $regGrowSlope,
                            in: 0.5...30.0,
                            step: 0.5,
                            onEditingChanged: { _ in
                                print("")
                            }
                        ).padding([.vertical], 0)
                        Text("\( String(format: "%g", regGrowSlope)) deg")
                            .foregroundColor(.white).padding([.vertical], 0).frame(width: 70, alignment: .trailing)
                    }
                    
                    Text("Curvature percentile")
                    HStack {
                        Slider(
                            value: $regGrowCurPerc,
                            in: 0.05...1.0,
                            step: 0.05,
                            onEditingChanged: { _ in
                                print("")
                            }
                        ).padding([.vertical], 0)
                        Text("\( String(format: "%g", regGrowCurPerc*100)) %")
                            .foregroundColor(.white).padding([.vertical], 0).frame(width: 70, alignment: .trailing)
                    }
                }
                
                
                VStack(alignment: .leading) {
                    Text("Terrain complexity evaluation").font(.title3).padding([.top], 15).padding([.bottom], 5)
                    
                    
                    Text("Landing area radius")
                    HStack {
                        Slider(
                            value: $landingRadius,
                            in: 0.05...2.5,
                            step: 0.05,
                            onEditingChanged: { _ in
                                print("")
                            }
                        ).padding([.vertical], 0)

                        Text("\( String(format: "%g", landingRadius)) m")
                            .foregroundColor(.white).padding([.vertical], 0).frame(width: 70, alignment: .trailing)
                    }
                    
                    Text("Slope angle")
                    HStack {
                        Slider(
                            value: $terCompSlope,
                            in: 0.5...30.0,
                            step: 0.5,
                            onEditingChanged: { _ in
                                print("")
                            }
                        ).padding([.vertical], 0)

                        Text("\( String(format: "%g", terCompSlope)) deg")
                            .foregroundColor(.white).padding([.vertical], 0).frame(width: 70, alignment: .trailing)
                    }
                    
                    
                    Text("Roughness")
                    HStack {
                        Slider(
                            value: $terCompRough,
                            in: 0.005...0.3,
                            step: 0.005,
                            onEditingChanged: { _ in
                                print("")
                            }
                        ).padding([.vertical], 0)
                        Text("\( String(format: "%g", terCompRough*100)) cm")
                            .foregroundColor(.white).padding([.vertical], 0).frame(width: 70, alignment: .trailing)
                    }
                    
                    
                    Text("Relief")
                    HStack {
                        Slider(
                            value: $terCompRel,
                            in: 0.005...0.3,
                            step: 0.005,
                            onEditingChanged: { _ in
                                print("")
                            }
                        ).padding([.vertical], 0)
                        Text("\( String(format: "%g", terCompRel*100)) cm")
                            .foregroundColor(.white).padding([.vertical], 0).frame(width: 70, alignment: .trailing)
                    }
                }
                
                
                Spacer()
            }.padding([.vertical], 5)
             .padding([.horizontal], 10)
        }
    }
}

//struct SideMenuView_Previews: PreviewProvider {
//    @Binding var colorSelection: Int
//    static var previews: some View {
//        SideMenuView(colorSelection: $colorSelection)
//    }
//}
