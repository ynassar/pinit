import UIKit
import SwiftGRPC
import SwiftProtobuf

/// The `LoginServer` is responsible for sending requests to the gRPC server related to the
/// login feature.
public class LoginServer {
    
    /// The delegate to notify when the `LoginServer` methods return any meaningful results.
    var delegate: LoginServerDelegate?
    
    /// Function responsible for sending a `LoginRequest` with the apporpriate credentials and
    /// calling the appropriate delegate function when a response is receieved.
    func loginWithCredentials(username: String, password: String) {
        let accountClient = AuthenticationServiceServiceClient(
            address: PinitConstants.authenticationServerAddress,
            secure: false,
            arguments: [])
        
        var loginRequest = LoginRequest()
        loginRequest.username = username
        loginRequest.password = password
        
        do {
            let loginResponse = try accountClient.login(loginRequest)
            delegate?.didLoginSuccessfully(loginResponse: loginResponse)
        } catch  {
            if let grpcError = error as? RPCError {
                delegate?.didLoginErrorOccur(errorMessage:
                    grpcError.callResult?.statusMessage ?? " Unknown Error Occured")
            } else {
                delegate?.didLoginErrorOccur(errorMessage: "Unknown Error Occured")
            }
        }
    }
}
