//
//  WebSocket.swift
//  DigitalMicroAirport
//
//  Created by Jussi Kalliola (TAU) on 28.8.2023.
//

import Foundation


class WebSocket: NSObject, URLSessionWebSocketDelegate {
    
    var session: URLSession?
    var url: URL?
    var webSocket : URLSessionWebSocketTask?

    
    init(url: String = "ws://192.168.177.6:9090") {
        super.init()
        self.session = URLSession(configuration: .default, delegate: self, delegateQueue: OperationQueue())
        self.url = URL(string:  url)
        
        self.start()
    }
    
    func start() {
        //Socket
        self.webSocket = self.session!.webSocketTask(with: self.url!)
        
        //Connect and hanles handshake
        self.webSocket?.resume()
    }
    
    //MARK: Receive
    func receive(){
        /// This Recurring will keep us connected to the server
        /*
         - Create a workItem
         - Add it to the Queue
         */
        
        let workItem = DispatchWorkItem{ [weak self] in
            
            self?.webSocket?.receive(completionHandler: { result in
                
                
                switch result {
                case .success(let message):
                    
                    switch message {
                    
                    case .data(let data):
                        print("Data received \(data)")
                        
                    case .string(let strMessgae):
                    print("String received \(strMessgae)")
                        
                    default:
                        break
                    }
                
                case .failure(let error):
                    print("Error Receiving \(error)")
                }
                // Creates the Recurrsion
                self?.receive()
            })
        }
        DispatchQueue.global().asyncAfter(deadline: .now() + 1 , execute: workItem)
    
    }
    
    func jsonToString(json: [String: Any]) -> String{
        do {
            let data1 =  try JSONSerialization.data(withJSONObject: json, options: JSONSerialization.WritingOptions.prettyPrinted)
            let convertedString = String(data: data1, encoding: String.Encoding.utf8)
            return convertedString!
        } catch let myJSONError {
            print(myJSONError)
            return ""
        }
    }
    
    func subscribe(json: [String: Any]) {
        
        let msg = self.jsonToString(json: json)
        //self.jsonToString(json: ["op": "subscribe", "topic": "/test_topic"])
                
        //print(msg)
        
        print("subscribed to topic /test_topic.")
        self.send(msg: msg)
        
    }
    
    func publish(json: [String: Any]) {
        
        let msg = self.jsonToString(json: json)
        //self.jsonToString(json: ["op": "publish", "topic": "/test_topic", "msg": ["data": "test msg."]])
                
        //print(msg)
        
        //print("topic: test_topic message published.")
        self.send(msg: msg)
        
    }
    
    func advertise(json: [String: Any]) {
        let msg = self.jsonToString(json: json)
        //self.jsonToString(json: ["op": "advertise", "topic": "/test_topic", "type": "std_msgs/String"])
        //print(msg)
        
        //print("topic: test_topic advertised.")
        self.send(msg: msg)
        
    }
    
    
    //MARK: Send
    func send(msg: String){
        /*
         - Create a workItem
         - Add it to the Queue
         */
        
        //print(msg)
        
        let workItem = DispatchWorkItem{
            
            self.webSocket?.send(URLSessionWebSocketTask.Message.string(msg), completionHandler: { error in
                print("message send succesfully.")
                
                if error != nil {
                    print(error)
                }
            })
        }
        
        DispatchQueue.global().asyncAfter(deadline: .now() + 3, execute: workItem)
    }
    
    //MARK: Close Session
    @objc func closeSession(){
        webSocket?.cancel(with: .goingAway, reason: "You've Closed The Connection".data(using: .utf8))
        
    }
    
    
    //MARK: URLSESSION Protocols
    func urlSession(_ session: URLSession, webSocketTask: URLSessionWebSocketTask, didOpenWithProtocol protocol: String?) {
        print("Connected to server")
        self.receive()
        //self.send(msg: "Test connection")
        
        // Subscribe to test topic
        self.advertise(json: ["op": "advertise", "topic": "/test_topic", "type": "std_msgs/String"])
        self.advertise(json: ["op": "advertise", "topic": "/iphone/rgb/image_raw", "type": "sensor_msgs/msg/Image"])
        self.advertise(json: ["op": "advertise", "topic": "/iphone/depth/image_raw", "type": "sensor_msgs/msg/Image"])
        self.advertise(json: ["op": "advertise", "topic": "/iphone/confidence/image_raw", "type": "sensor_msgs/msg/Image"])
        self.advertise(json: ["op": "advertise", "topic": "/iphone/pose", "type": "geometry_msgs/msg/PoseStamped"])
        self.advertise(json: ["op": "advertise", "topic": "/iphone/lidar", "type": "sensor_msgs/msg/PointCloud2"])
    }
    
    
    func urlSession(_ session: URLSession, webSocketTask: URLSessionWebSocketTask, didCloseWith closeCode: URLSessionWebSocketTask.CloseCode, reason: Data?) {
        print("Disconnect from Server \(reason)")
    }
    
}