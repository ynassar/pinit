import UIKit
import SwiftGRPC
import SwiftProtobuf
import Foundation

/// The server responsible for sending requests to the gRPC server related to the
/// mapping features.
public class MappingServer {
    
    /// The client that has the interface of methods used to make different requests.
    var mappingClient: RosServiceServiceClient
    
    /// The delegate to notify when needed.
    var delegate: MappingServerDelegate?
    
    var mappingRequest: MappingRequest!
    
    /// Initializer of the `MappingServer`.
    init() {
        mappingClient = RosServiceServiceClient(
            address: PinitConstants.tempRobotServerAddress,
            secure: false,
            arguments: [])
        
        mappingRequest = MappingRequest()
        mappingRequest.robotName = "nemo"
        var serverToRosMappingRequest = ServerToRosMappingRequest()
        mappingRequest.mappingRequest = serverToRosMappingRequest
    }
    
    private func sendMovement(direction: ServerToRosMappingRequest.Direction) {
        mappingRequest.mappingRequest.requestType = .direction
        mappingRequest.mappingRequest.direction = direction
        do {
            let movementResponse = try mappingClient.sendMovement(mappingRequest)
        } catch {
            delegate?.didMappingErrorOccur("Can't connect to the server")
        }
    }
    
    public func moveRobotForward() {
        sendMovement(direction: .forward)
    }
    
    public func moveRobotRight() {
        sendMovement(direction: .right)
    }
    
    public func moveRobotLeft() {
        sendMovement(direction: .left)
    }
    
    public func moveRobotBackward() {
        sendMovement(direction: .backward)
    }
    
    public func moveRobotStop() {
        sendMovement(direction: .stop)
    }
    
    public func startMappingRequest() {
        mappingRequest.mappingRequest.requestType = .startMapping
        do {
            let movementResponse = try mappingClient.sendMovement(mappingRequest)
            delegate?.mapStartConfirmation()
        } catch  {
            delegate?.didMappingErrorOccur("Can't start mapping now")
        } 
    }
    
    public func saveMappingRequest() {
        mappingRequest.mappingRequest.requestType = .stopMapping
        do {
            let movementResponse = try mappingClient.sendMovement(mappingRequest)
            delegate?.mapSaveConfirmation()
        } catch {
            let message = error.localizedDescription.description
            delegate?.didMappingErrorOccur("Couldn't Save the map")
        }
    }
    
    public func mapImageRequestAsynchronous() {
        
        var getMapRequest = GetMapRequest()
        getMapRequest.robotName = "nemo"
        
        let _ = Timer.scheduledTimer(withTimeInterval: 3, repeats: true, block: {
            _ in
            do {
                let mapResponse = try self.mappingClient.getMapImage(getMapRequest)
                let dataDecoded : Data = Data(base64Encoded: mapResponse.encodedImage, options: .ignoreUnknownCharacters)!
                let decodedImage = UIImage(data: dataDecoded)
                if let mapImage = decodedImage {
                    self.delegate?.mapImageUpdate(newImage: mapImage)
                }
            } catch {
            }
        })
        
    }

}
