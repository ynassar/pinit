import UIKit

/// Extension for `UIButton` to add the functionality to enable/disable the button.
extension UIButton {
    
    /// Function to enable the `UIButton` by responding to different actions and setting its
    /// color to be fully opaque.
    public func enableButton() {
        self.isEnabled = true
        self.alpha = 1.0
    }
    
    /// Function to disable the `UIButton` by not responding to any actions and setting its
    /// color to be half opaque.
    public func disableButton() {
        self.isEnabled = false
        self.alpha = 0.5
    }
    
    public func addGradiant(colors: [CGColor]) {
        let gradiantLayer = CAGradientLayer()
        gradiantLayer.frame = self.bounds
        gradiantLayer.startPoint = CGPoint(x: 0, y: 0)
        gradiantLayer.endPoint = CGPoint(x: 1, y: 0)
        gradiantLayer.colors = colors
        gradiantLayer.cornerRadius = self.layer.cornerRadius
        self.layer.insertSublayer(gradiantLayer, at: 0)
    }
    
    public func makeButtonCircular() {
        self.layer.cornerRadius = self.bounds.width / 2.0
        self.clipsToBounds = true
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
