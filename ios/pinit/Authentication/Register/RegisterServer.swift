import UIKit

public class RegisterServer {
    
    var delegate: RegisterServerDelegate?
    
    func sendRequest(request: RegisterRequest) {
        let accountClient = AccountManagementServiceServiceClient(
            address: PinitConstants.serverAddress,
            secure: false,
            arguments: [])

        do {
            let registerResponse = try accountClient.register(request)
            delegate?.didRegisterSuccessfully(self)
            print("response")
        } catch {
            print("in catch")
        }
    }
}
