import UIKit

public class RegisterServer {
    
    var delegate: RegisterServerDelegate?
    
    func registerWithCredentials(
        username: String,
        password: String,
        email: String
    ) {
        let accountClient = AccountManagementServiceServiceClient(
            address: PinitConstants.tempAuthenticationServerAddress,
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
            delegate?.didRegisterErrorOccur(errorMessage: "SignUp Failed")
        }
    }
}
