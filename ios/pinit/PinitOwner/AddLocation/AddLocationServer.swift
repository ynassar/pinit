import UIKit
import SwiftProtobuf

public class AddLocationServer {
    
    var delegate: AddLocationServerDelegate?
    
    func addLocation(locationName: String, locationDescription: String) {
        let accountClient = RosServiceServiceClient(
            address: PinitConstants.robotServerAddress,
            secure: false,
            arguments: [])
        
        var addLocationRequest = AddWaypointRequest()
        addLocationRequest.waypointName = locationName
        addLocationRequest.description_p = locationDescription
        
        if let userProfile = UserDefaultsHelper.getUserProfile() {
            addLocationRequest.token = userProfile.token
        }
                
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

