import UIKit

/// Extension for `UIButton` to add the functionality to enable/disable the button.
extension UIButton {
    
    /// Function to enable the `UIButton` by responding to different actions and setting its
    /// color to be fully opaque.
    func enableButton() {
        self.isEnabled = true
        self.alpha = 1.0
    }
    
    /// Function to disable the `UIButton` by not responding to any actions and setting its
    /// color to be half opaque.
    func disableButton() {
        self.isEnabled = false
        self.alpha = 0.5
    }
}

/// Extension for `UIBarButtonItem` to add the functionality to enable/disable the button.
extension UIBarButtonItem {
    
    /// Function to enable the `UIBarButtonItem` by responding to different actions.
    func enableButton() {
        self.isEnabled = true
    }
    
    /// Function to disable the `UIBarButtonItem` by not responding to any actions.
    func disableButton() {
        self.isEnabled = false
    }
}
