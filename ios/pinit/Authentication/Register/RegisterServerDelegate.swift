import Foundation
import UIKit

protocol RegisterServerDelegate {
    
    func didRegisterSuccessfully()
    
    func didRegisterErrorOccur(errorMessage: String)
}
