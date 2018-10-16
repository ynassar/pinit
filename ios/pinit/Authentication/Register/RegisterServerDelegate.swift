import Foundation
import UIKit

protocol RegisterServerDelegate {
    
    func didRegisterSuccessfully(_ sender: RegisterServer)
    
    func didRegisterErrorOccur(_ sender: RegisterServer)
}
