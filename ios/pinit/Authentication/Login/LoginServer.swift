import UIKit

public class LoginServer {
    
    var delegate: LoginServerDelegate?
    
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
            delegate?.didLoginSuccessfully()
        } catch {
            delegate?.didLoginErrorOccur(errorMessage: "Can't login")
        }
    }
}
