import UIKit
import SwiftProtobuf

public class AddLocationServer {
    
    var delegate: AddLocationServerDelegate?
    
    func addLocation(locationName: String, locationDescription: String) {
        let accountClient = RosServiceServiceClient(
            address: PinitConstants.tempRobotServerAddress,
            secure: false,
            arguments: [])
        
        var addLocationRequest = AddWaypointRequest()
        addLocationRequest.waypointName = locationName
        addLocationRequest.description_p = locationDescription
        
        let userDefaults = UserDefaults.standard
        addLocationRequest.token = userDefaults.string(forKey: "AccountToken")!
                
        var timestamp = Google_Protobuf_Timestamp()
        timestamp.seconds = Int64(Date().timeIntervalSince1970)
        addLocationRequest.timestamp = timestamp
        
        do {
            let addLocationResponse = try accountClient.addWaypoint(addLocationRequest)
            delegate?.didAddLocationSuccessfully()
        } catch {
            delegate?.didAddLocationErrorOccur(errorMessage: "Error adding the location")
        }
    }
}

