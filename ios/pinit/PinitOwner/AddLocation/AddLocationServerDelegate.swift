import Foundation
import UIKit

protocol AddLocationServerDelegate {
    
    func didAddLocationSuccessfully()
    
    func didAddLocationErrorOccur(errorMessage: String)
}
