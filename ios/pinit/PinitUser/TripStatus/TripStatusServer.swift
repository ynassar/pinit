import UIKit
import SwiftProtobuf

class TripStatusServer {
    
    public var delegate: TripStatusServerDelegate?
    
    private var tripStatusClient: RosServiceServiceClient!
    
    init() {
        tripStatusClient = RosServiceServiceClient(
            address: PinitConstants.robotServerAddress,
            secure: false,
            arguments: [])
    }
    
    public func getTripStatus() {
        
        var getTripStatusRequest = GetTripStatusRequest()
        if let userProfile = UserDefaultsHelper.getUserProfile() {
            getTripStatusRequest.token = userProfile.token
        }
        
        let _ = Timer.scheduledTimer(withTimeInterval: 0.5, repeats: true, block: {
            _ in
            do {
                let tripStatus = try self.tripStatusClient.getTripStatus(getTripStatusRequest)
                let status = PinitTripStatus(grpcTripStatus: tripStatus.status)
                self.delegate?.updateTripStatus(status: status)
            } catch {
                print("Error in Trip Status Server")
            }
        })
    }
    
    public func confrimTrip() {
        
        var confrimTripRequest = ConfirmTripRequest()
        if let userProfile = UserDefaultsHelper.getUserProfile() {
            confrimTripRequest.token = userProfile.token
        }
        
        do {
            let response = try tripStatusClient.confirmTrip(confrimTripRequest)
        } catch {
            print("Error in confirming request")
        }
    }
    
}
