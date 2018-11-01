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
    
    /// Private function responsible foe sending the direction mapping request to the GRPC server.
    private func sendMovement(direction: ServerToRosMappingRequest.Direction) {
        mappingRequest.mappingRequest.requestType = .direction
        mappingRequest.mappingRequest.direction = direction
        do {
            let movementResponse = try mappingClient.sendMovement(mappingRequest)
        } catch {
            delegate?.didMappingErrorOccur("Can't connect to the server")
        }
    }
    
    /// Function to move the robot in the direction `forward`.
    public func moveRobotForward() {
        sendMovement(direction: .forward)
    }
    
    /// Function to move the robot in the direction `right`.
    public func moveRobotRight() {
        sendMovement(direction: .right)
    }
    
    /// Function to move the robot in the direction `left`.
    public func moveRobotLeft() {
        sendMovement(direction: .left)
    }
    
    /// Function to move the robot in the direction `backward`.
    public func moveRobotBackward() {
        sendMovement(direction: .backward)
    }
    
    /// Function to move the robot in the direction `stop`.
    public func moveRobotStop() {
        sendMovement(direction: .stop)
    }
    
    /// Function to send the `MappingRequest` to start mapping.
    public func startMappingRequest() {
        mappingRequest.mappingRequest.requestType = .startMapping
        do {
            let movementResponse = try mappingClient.sendMovement(mappingRequest)
            delegate?.mapStartConfirmation()
        } catch  {
            delegate?.didMappingErrorOccur("Can't start mapping now")
        } 
    }
    
    /// Function to send the `MappingRequest` to stop mapping and saving the map on the robot
    /// or on the GRPC server.
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
    
    /// Function to send a `MappingRequest` every 3 seconds requesting the lastest version
    /// of the map saved in the GRPC server database and when fetched, it's sent back
    /// to the delegate view controller to do the appropriate changes.
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
