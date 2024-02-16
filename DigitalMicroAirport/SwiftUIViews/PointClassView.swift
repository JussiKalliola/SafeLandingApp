//
//  PointClassView.swift
//  DigitalMicroAirport
//
//  Created by Jussi Kalliola (TAU) on 18.11.2022.
//

import SwiftUI

struct PointClassView: View {
    var body: some View {
        VStack {
            HStack {
                // 255.0, 128.0, 0.0 Non flat
                Text("Non-flat").font(.system(size: 12)).frame(alignment: .leading)
                Spacer()
                Rectangle().frame(width: 10, height: 10).foregroundColor(Color(UIColor(red: 255.0, green: 128.0, blue: 0.0, alpha: 1.0)))
            }.padding([.all], 0).frame(width:80, height:10, alignment: .leading)
            HStack {
                // 255.0, 0.0, 0.0 Risky
                Text("Risky").font(.system(size: 12)).frame(alignment: .leading)
                Spacer()
                Rectangle().frame(width: 10, height: 10).foregroundColor(Color(UIColor(red: 255.0, green: 0.0, blue: 0.0, alpha: 1.0)))
            }.padding([.all], 0).frame(width:80, height:10, alignment: .leading)
            HStack {
                // 0.0, 0.0, 255.0 Border
                Text("Border").font(.system(size: 12)).frame(alignment: .leading)
                Spacer()
                Rectangle().frame(width: 10, height: 10).foregroundColor(Color(UIColor(red: 0.0, green: 0.0, blue: 255.0, alpha: 1.0)))
            }.padding([.all], 0).frame(width:80, height:10, alignment: .leading)
            HStack {
                // Suitable 0.0, 125.0, 0.0
                Text("Suitable").font(.system(size: 12)).frame(alignment: .leading)
                Spacer()
                Rectangle().frame(width: 10, height: 10).foregroundColor(Color(UIColor(red: 0.0, green: 125.0, blue: 0.0, alpha: 1.0)))
            }.padding([.all], 0).frame(width:80, height:10, alignment: .leading)
            HStack {
                // Best 0.0, 255.0, 255.0
                Text("Optimal").font(.system(size: 12)).frame(alignment: .leading)
                Spacer()
                Rectangle().frame(width: 10, height: 10).foregroundColor(Color(UIColor(red: 0.0, green: 255.0, blue: 255.0, alpha: 1.0)))
            }.padding([.all], 0).frame(width:80, height:15, alignment: .leading)
        }.frame(alignment: .leading)
    }
}

struct PointClassView_Previews: PreviewProvider {
    static var previews: some View {
        PointClassView()
    }
}
