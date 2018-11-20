import UIKit
import SwiftGRPC
import SwiftProtobuf
import Foundation
import CoreLocation

public class RobotRequestServer {
    
    var robotRequestClient: RosServiceServiceClient
    
    var delegate: RobotRequestServerDelegate?
    
    init() {
        robotRequestClient = RosServiceServiceClient(
            address: PinitConstants.robotServerAddress,
            secure: false,
            arguments: [])
    }
    
    public func requestRobotToLocation(gpsCoordinates: CLLocation) {
        
        var robotNavigationRequest = Robo()
        if let userProfileObject = UserDefaults.standard.object(forKey: PinitConstants.savedProfileKey) {
            let userProfileDecoded = userProfileObject as! Data
            let userProfile = NSKeyedUnarchiver.unarchiveObject(with: userProfileDecoded) as! Profile
            
            robotNavigationRequest.token = userProfile.token
            
            var sentGpsCoordinates = GpsCoordinates()
            sentGpsCoordinates.longitude = Float(gpsCoordinates.coordinate.longitude)
            sentGpsCoordinates.latitude = Float(gpsCoordinates.coordinate.latitude)
            robotNavigationRequest.coordinates = sentGpsCoordinates
            
            delegate?.didRequestSuccessfully()
            
//            do {
//                let locationsList = try robotRequestClient.requestRobotToLocation(robotNavigationRequest)
//                delegate?.didRequestSuccessfully()
//            } catch {
//                delegate?.didFailToRequestRobot(erroMessage: "Error Occured")(
//            }
//
        }
    }
}
