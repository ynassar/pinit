import Foundation
import UIKit

/// `LoginServerDelegate` is implemented by all the classes that use `LoginServer`
/// and want to be notified with the results of their request.
protocol LoginServerDelegate {
    
    /// Function called when the `LoginRequest` is sent and no error occured, so the
    /// delegate view controller is notified to do any actions dependant on that request.
    func didLoginSuccessfully(loginResponse: LoginResponse)
    
    /// Function called when the `LoginRequest` is sent and an error occured, so the
    /// delegate view controller is notified to do any actions dependant on that request, along
    /// with a `errorMessage` indicating the type of error that occured.
    func didLoginErrorOccur(errorMessage: String)
}
