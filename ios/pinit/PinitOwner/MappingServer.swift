import UIKit

/// The server responsible for sending requests to the gRPC server related to the
/// mapping features.
public class MappingServer {
    
    /// The client that has the interface of methods used to make different requests.
    var mappingClient: RosServiceServiceClient
    
    /// The delegate to notify when needed.
    var delegate: MappingServerDelegate?
    
    /// Initializer of the `MappingServer`.
    init() {
        mappingClient = RosServiceServiceClient(
            address: PinitConstants.tempRobotServerAddress,
            secure: false,
            arguments: [])
    }
    
    /// Send a `MappingRequest` related to the movemenet of the robot in a specific direction.
    func sendMovementRequest(mappingRequest: MappingRequest) {
        do {
            let movementResponse = try mappingClient.sendMovement(mappingRequest)
        } catch {
            delegate?.didMappingErrorOccur("Can't connect to the server")
        }
    }
    
    /// Send a `MappingRequest` related to the functionality of mapping, like staring and saving.
    func startMappingRequest(mappingRequest: MappingRequest) {
        do {
            let movementResponse = try mappingClient.sendMovement(mappingRequest)
        } catch {
            delegate?.didMappingErrorOccur("Can't start mapping now")
        }
    }
}
