import Foundation
import UIKit

/// `RegisterServerDelegate` is implemented by all the classes that use `RegisterServer` and
/// want to be notified with the results of their reqeusts.
protocol RegisterServerDelegate {
    
    /// Function called when the `RegisterRequest` is sent and no error occured, so the
    /// delegate view controller is notified to do any actions dependant on that request.
    func didRegisterSuccessfully()
    
    /// Function called when the `RegisterRequest` is sent and an error occured, so the
    /// delegate view controller is notified to do any actions dependant on that request, along
    /// with a `errorMessage` indicating the type of error that occured.
    func didRegisterErrorOccur(errorMessage: String)
}
