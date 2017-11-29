/*
* Copyright 2010-2016 Amazon.com, Inc. or its affiliates. All Rights Reserved.
*
* Licensed under the Apache License, Version 2.0 (the "License").
* You may not use this file except in compliance with the License.
* A copy of the License is located at
*
*  http://aws.amazon.com/apache2.0
*
* or in the "license" file accompanying this file. This file is distributed
* on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
* express or implied. See the License for the specific language governing
* permissions and limitations under the License.
*/
import Foundation
import UIKit
import AWSIoT

struct ImageChunk: Codable {
    var data: String
    var pic_id: String
    var pos: Int
    var size: Int
}



struct StringImage: Codable {
    var data = [ImageChunk?]()
    var pic_id: String
    var size: Int
    var count: Int = 0
    
    mutating func add_chunk(new_chunk: ImageChunk) {
        let pos = Int(new_chunk.pos)
        self.data.remove(at: pos)
        self.data.insert(new_chunk, at: pos)
        count += 1
    }
    func check_image() -> Bool {
        if self.count == self.size {
            return true
        } else {
            return false
        }
    }
}

extension ImageChunk {
    static func from(json: String, using encoding: String.Encoding = .utf8) -> ImageChunk? {
        guard let data = json.data(using: encoding) else { return nil }
        return from(data: data)
    }
    
    static func from(data: Data) -> ImageChunk? {
        let decoder = JSONDecoder()
        return try? decoder.decode(ImageChunk.self, from: data)
    }
    
    static func from(url urlString: String) -> ImageChunk? {
        guard let url = URL(string: urlString) else { return nil }
        guard let data = try? Data(contentsOf: url) else { return nil }
        return from(data: data)
    }
    
    static func toString(object: ImageChunk) -> String {
        let encoder = JSONEncoder()
        return try! NSString(data: encoder.encode(object), encoding: String.Encoding.utf8.rawValue)! as String
        
        
    }
    
    var jsonData: Data? {
        let encoder = JSONEncoder()
        return try? encoder.encode(self)
    }
    
    var jsonString: String? {
        guard let data = self.jsonData else { return nil }
        return String(data: data, encoding: .utf8)
    }
}

extension ImageChunk {
    enum ChunkKeys: String {
        case data = "data"
        case pic_id = "pic_id"
        case pos = "pos"
        case size = "size"
    }
}
class SubscribeViewController: UIViewController {

    
    @IBOutlet weak var subscribeButtonRight: UIButton!
    @IBOutlet weak var subscribeButtonForward: UIButton!
    @IBOutlet weak var subscribeButtonLeft: UIButton!
    @IBOutlet weak var subscribeImageView: UIImageView!
    @IBOutlet weak var subscribeText: UITextView!
    @IBOutlet weak var subscribeNextButton: UIButton!
    @IBOutlet weak var subscribeStopButton: UIButton!
    
    var image_current: StringImage!
    var image_count: Int = 0
    var stop: Bool = false
    
    private func handle_chunks(newChunk: ImageChunk){
        //find string image in images array
        if image_current != nil {
            if image_current.pic_id == newChunk.pic_id {
                image_current.add_chunk(new_chunk: newChunk)
                if image_current.check_image() {
                    subscribeText.text.append("Ready set_image")
                    self.set_image(stringImage: image_current)
                }
            } else {
                image_current = nil 
            }
            
        } else {
            var temp_list = [ImageChunk?]()
            var i = 0
            while(i < newChunk.size){
                temp_list.append(nil)
                i += 1
            }
            image_current = StringImage(data: temp_list, pic_id: newChunk.pic_id, size: newChunk.size, count: 0)
            image_current.add_chunk(new_chunk: newChunk)
            
        }
        
        
        
    }
    
    private func set_image(stringImage: StringImage){
        var encodedImageData: String = ""
        var i = 0
        while i < image_current.size {
            if image_current.data[i]?.data != nil {
                let chunk = image_current.data[i]?.data
                encodedImageData.append(chunk!)
                i += 1
            }
            
        }
        let imageData: Data = Data(base64Encoded: encodedImageData, options: .ignoreUnknownCharacters)!
        let image = UIImage(data: imageData)
        subscribeImageView.image = image
        image_current = nil
        request_image()
        
        
            
    }
    
    
    override func viewDidLoad() {
        super.viewDidLoad()

        // Do any additional setup after loading the view, typically from a nib.
        //subscribeSlider.isEnabled = false
        subscribeText.text = ""
    }

    override func viewWillAppear(_ animated: Bool) {
        let iotDataManager = AWSIoTDataManager.default()
        let tabBarViewController = tabBarController as! AlfredIoTTabBarController

        iotDataManager.subscribe(toTopic: "robot_to_awsiot", qoS: .messageDeliveryAttemptedAtMostOnce, messageCallback: {
            (payload) ->Void in
            let stringValue = NSString(data: payload, encoding: String.Encoding.utf8.rawValue)!
            let curr_chunk = ImageChunk.from(json: stringValue as String)
            self.handle_chunks(newChunk: curr_chunk!)
            print("received: \(stringValue)")
            //self.subscribeText.insertText("received: \(stringValue)")
            //self.subscribeText.text?.append("\r\n received: \(stringValue)")
            let curr_pic_id = curr_chunk?.pic_id
            self.subscribeText.text = ("\r\n received: \(curr_pic_id ?? "Error here")")
            DispatchQueue.main.async {
                //self.subscribeSlider.value = stringValue.floatValue
            }
        } )
    }

    override func viewWillDisappear(_ animated: Bool) {
        let iotDataManager = AWSIoTDataManager.default()
        let tabBarViewController = tabBarController as! AlfredIoTTabBarController
        iotDataManager.unsubscribeTopic(tabBarViewController.topic)
    }
    
    func request_image() {
        if self.stop != true {
            let stringValue = "{\"data\":\"new\",\"pic_id\":\"\",\"pos\":0,\"size\":0}"
            let iotDataManager = AWSIoTDataManager.default()
            iotDataManager.publishString(stringValue, onTopic:"both_directions", qoS:.messageDeliveryAttemptedAtMostOnce)
        }
        
    }

    @IBAction func nextButtonPress(_ sender: UIButton) {
        self.stop = false 
        request_image()
    }
    
    @IBAction func stopButtonPress(_ sender: UIButton) {
        let stringValue = "{\"data\":\"stop\",\"pic_id\":\"\",\"pos\":0,\"size\":0}"
        let iotDataManager = AWSIoTDataManager.default()
        iotDataManager.publishString(stringValue, onTopic:"both_directions", qoS:.messageDeliveryAttemptedAtMostOnce)
        self.stop = true
    }
    @IBAction func buttonLeftPress(_ sender: UIButton) {
        let iotDataManager = AWSIoTDataManager.default()
        let tabBarViewController = tabBarController as! AlfredIoTTabBarController
        let dataString = "{\"linear\":{\"x\":0.0,\"y\":0.0,\"z\":0.0},\"angular\":{\"x\":0.0,\"y\":0.0,\"z\":2}}"
        iotDataManager.publishString(dataString, onTopic:tabBarViewController.topic, qoS: .messageDeliveryAttemptedAtMostOnce)
    }
    
    @IBAction func buttonForwardPress(_ sender: UIButton) {
        let iotDataManager = AWSIoTDataManager.default()
        let tabBarViewController = tabBarController as! AlfredIoTTabBarController
        let dataString = "{\"linear\":{\"x\":2,\"y\":0.0,\"z\":0.0},\"angular\":{\"x\":0.0,\"y\":0.0,\"z\":0.0}}"
        iotDataManager.publishString(dataString, onTopic:tabBarViewController.topic, qoS: .messageDeliveryAttemptedAtMostOnce)
    }
    
    @IBAction func buttonRightPress(_ sender: UIButton) {
        let iotDataManager = AWSIoTDataManager.default()
        let tabBarViewController = tabBarController as! AlfredIoTTabBarController
        let dataString = "{\"linear\":{\"x\":0.0,\"y\":0.0,\"z\":0.0},\"angular\":{\"x\":0.0,\"y\":0.0,\"z\":-2}}"
        iotDataManager.publishString(dataString, onTopic:tabBarViewController.topic, qoS: .messageDeliveryAttemptedAtMostOnce)
    }
    //    @IBAction func stopButtonPress(_ sender: Any) {
//        let stringValue = "{\"data\":\"new\",\"pic_id\":\"\",\"pos\":\"\",\"size\":\"\"}"
//        let iotDataManager = AWSIoTDataManager.default()
//        iotDataManager.publishString(stringValue, onTopic:"both_directions", qoS:.messageDeliveryAttemptedAtMostOnce)
//    }
//    @IBAction func sliderValueChanged(_ sender: UISlider) {
//        print("\(sender.value)")
//        let stringValue = "{\"data\":\"stop\",\"pic_id\":\"\",\"pos\":\"\",\"size\":\"\"}"
//
//        let iotDataManager = AWSIoTDataManager.default()
//        //let tabBarViewController = tabBarController as! AlfredIoTTabBarController
//
//        iotDataManager.publishString(stringValue, onTopic:"both_directions", qoS:.messageDeliveryAttemptedAtMostOnce)
//    }
    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // Dispose of any resources that can be recreated.
    }
}

