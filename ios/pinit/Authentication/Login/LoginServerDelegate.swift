import Foundation
import UIKit

protocol LoginServerDelegate {
    
    func didLoginSuccessfully()
    
    func didLoginErrorOccur(errorMessage: String)
}
