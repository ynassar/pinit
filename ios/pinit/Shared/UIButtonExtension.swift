import UIKit

extension UIButton {
    
    func enableButton() {
        self.isEnabled = true
        self.alpha = 1.0
    }
    
    func disableButton() {
        self.isEnabled = false
        self.alpha = 0.5
    }
}

extension UIBarButtonItem {
    
    func enableButton() {
        self.isEnabled = true
    }
    
    func disableButton() {
        self.isEnabled = false
    }
}
