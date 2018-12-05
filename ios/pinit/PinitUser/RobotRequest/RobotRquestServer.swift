import UIKit
import SwiftProtobuf

class RequestRobotServer {
    
    public var delegate: RobotRequestServerDelegate!
    
    private var requestClient: RosServiceServiceClient
    
    init() {
        requestClient = RosServiceServiceClient(
            address: PinitConstants.robotServerAddress,
            secure: false,
            arguments: [])
    }
    
    public func sendRequest(robotRequest: RobotRequest) {
        var createTripRequest =  CreateTripRequest()
        createTripRequest.startWaypoint = robotRequest.pickUpName
        createTripRequest.endWaypoint = robotRequest.destinationName
        
        if let userProfile = UserDefaultsHelper.getUserProfile() {
            createTripRequest.token = userProfile.token
        }
        
        do {
            let response = try requestClient.createTrip(createTripRequest)
            delegate.robotRequested()
        } catch {
            print("Error")
        }
    }
}
