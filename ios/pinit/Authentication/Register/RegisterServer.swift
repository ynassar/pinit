import UIKit
import SwiftGRPC
import SwiftProtobuf

/// The `RegisterServer` is responsible for sending requests to gRPC server related to the
/// register feature.
public class RegisterServer {
    
    /// The delegate to notify when the `RegisterServer` methods return any meaningful results.
    var delegate: RegisterServerDelegate?
    
    /// Function responsible for sending a `RegisterRequest` with the apporpriate info and
    /// calling the appropriate delegate function when a response is receieved.
    func registerWithCredentials(
        username: String,
        password: String,
        email: String
    ) {
        let accountClient = AccountManagementServiceServiceClient(
            address: PinitConstants.authenticationServerAddress,
            secure: false,
            arguments: [])
        
        var registerRequest = RegisterRequest()
        registerRequest.username = username
        registerRequest.password = password
        registerRequest.email = email

        do {
            let registerResponse = try accountClient.register(registerRequest)
            delegate?.didRegisterSuccessfully()
        } catch {
            if let grpcError = error as? RPCError {
                delegate?.didRegisterErrorOccur(errorMessage:
                    grpcError.callResult?.statusMessage ?? " Unknown Error Occured")
            } else {
                delegate?.didRegisterErrorOccur(errorMessage: "Unknown Error Occured")
            }
        }
        
    }
}
