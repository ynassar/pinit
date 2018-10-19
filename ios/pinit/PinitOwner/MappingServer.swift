import UIKit

public class MappingServer {
    
    var mappingClient: RosServiceServiceClient
    
    init() {
        mappingClient = RosServiceServiceClient(
            address: PinitConstants.tempRobotServerAddress,
            secure: false,
            arguments: [])
    }
    
    func sendMovementRequest(mappingRequest: MappingRequest) {
        do {
            let _ = try mappingClient.sendMovement(mappingRequest)
            print("in mapping Server")
        } catch {
            print("Error in Moving Request")
        }
    }
}
