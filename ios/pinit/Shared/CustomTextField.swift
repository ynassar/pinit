import UIKit

/// Custom Textfield with left and right padding for the text inside
class CustomTextField : UITextField {
    
    /// The padding to be used with the `CustomTextField`.
    let padding = UIEdgeInsets(top: 0, left: 15, bottom: 0, right: 5)
    
    /// Override the initialization of the entire textfield rectangle.
    override open func textRect(forBounds bounds: CGRect) -> CGRect {
        return bounds.inset(by: padding)
    }
    
    /// Override the initialization of the placeholder rectangle.
    override open func placeholderRect(forBounds bounds: CGRect) -> CGRect {
        return bounds.inset(by: padding)
    }
    
    /// Override the initialization of the editing rectangle.
    override open func editingRect(forBounds bounds: CGRect) -> CGRect {
        return bounds.inset(by: padding)
    }
    
}
