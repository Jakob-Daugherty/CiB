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

import UIKit
import AWSIoT
import Foundation

class PublishViewController: UIViewController {

    @IBOutlet weak var publishSlider: UISlider!
    @IBOutlet weak var publishButtonLeft: UIButton!
    @IBOutlet weak var publishButtonRight: UIButton!
    @IBOutlet weak var publishButtonForward: UIButton!
    @IBOutlet weak var publishWebView: UIWebView!
    @IBOutlet weak var publishTextField: UITextView!

    override func viewDidLoad() {
        super.viewDidLoad()
        //call rendering function
        publishSlider.isEnabled = false
        //let url = URL.init(string: "http://192.168.0.20:8081")
        let url = URL.init(string: "http://172.20.10.7:8081")
        publishWebView.loadRequest(URLRequest(url: url!))
        publishTextField.text = ""
        //request_image()
    }
    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // Dispose of any resources that can be recreated.
    }

    override func viewWillAppear(_ animated: Bool) {
        let iotDataManager = AWSIoTDataManager.default()
        //let iotDataManager = AWSIoTDataManager.init(forKey: "certificateId")
        _ = tabBarController as! AlfredIoTTabBarController
        //request_image()
        
        
//        iotDataManager.subscribe(toTopic: "both_directions", qoS: .messageDeliveryAttemptedAtMostOnce, messageCallback: {
//            (payload) ->Void in
//            let stringValue = NSString(data: payload, encoding: String.Encoding.utf8.rawValue)!
//
//            let newChunk = ImageChunk.from(json: stringValue as String)
//
//
//            //Convert jsonString to jsonArray
//
//
//            if (newChunk != nil && newChunk?.pic_id != "new"){
//                //print("Parsed JSON: \(newChunk!)")
//                self.handle_chunks(newChunk: newChunk!)
//                self.publishTextField.text = "Parsed JSON pic_id: " + String(describing: newChunk!.pic_id)
//            }
//
        
            //print("json[2]: \(json![2])")
            
            
            
            //print("received: \(stringValue)")
            //self.subscribeText.insertText("received: \(stringValue)")
            //self.subscribeText.text?.append("\r\n received: \(stringValue)")
            //self.publishImageField.text = "\r\n received: \(stringValue)"
            //DispatchQueue.main.async {
            //    self.publishSlider.value = stringValue.floatValue
            //}
        //} )
    }
    
    @IBAction func sliderValueChanged(_ sender: UISlider) {
        print("\(sender.value)")
        let iotDataManager = AWSIoTDataManager.default()
        let tabBarViewController = tabBarController as! AlfredIoTTabBarController
        iotDataManager.publishString("\(sender.value)", onTopic:tabBarViewController.topic, qoS:.messageDeliveryAttemptedAtMostOnce)
    }
    @IBAction func buttonForwardPress(_ sender: Any) {
        let iotDataManager = AWSIoTDataManager.default()
        let tabBarViewController = tabBarController as! AlfredIoTTabBarController
        let dataString = "{\"linear\":{\"x\":2,\"y\":0.0,\"z\":0.0},\"angular\":{\"x\":0.0,\"y\":0.0,\"z\":0.0}}"
        iotDataManager.publishString(dataString, onTopic:tabBarViewController.topic, qoS: .messageDeliveryAttemptedAtMostOnce)
        
    }
    @IBAction func buttonLeftPress(_ sender: Any) {
        let iotDataManager = AWSIoTDataManager.default()
        let tabBarViewController = tabBarController as! AlfredIoTTabBarController
        let dataString = "{\"linear\":{\"x\":0.0,\"y\":0.0,\"z\":0.0},\"angular\":{\"x\":0.0,\"y\":0.0,\"z\":2}}"
        iotDataManager.publishString(dataString, onTopic:tabBarViewController.topic, qoS: .messageDeliveryAttemptedAtMostOnce)
    }
    @IBAction func buttonRightPress(_ sender: Any) {
        let iotDataManager = AWSIoTDataManager.default()
        let tabBarViewController = tabBarController as! AlfredIoTTabBarController
        let dataString = "{\"linear\":{\"x\":0.0,\"y\":0.0,\"z\":0.0},\"angular\":{\"x\":0.0,\"y\":0.0,\"z\":-2}}"
        iotDataManager.publishString(dataString, onTopic:tabBarViewController.topic, qoS: .messageDeliveryAttemptedAtMostOnce)
    }
//    func request_image() {
//        if requested == nil {
//            let iotDataManager = AWSIoTDataManager.default()
//            let data = ImageChunk(data: "", pic_id: "new", pos: "", size: "")
//            let dataString = ImageChunk.toString(object: data)
//            iotDataManager.publishString(dataString, onTopic: "both_directions", qoS: .messageDeliveryAttemptedAtMostOnce)
//            requested = 1
//        }
//
//    }
}
