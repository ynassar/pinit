import UIKit

extension UIView {
    
    /// Center the view horizontally in reference to another view superview
    public func addCenterXConstraint(relativeView: UIView) -> Self {
        let adjustedView = self
        self.translatesAutoresizingMaskIntoConstraints = false
        NSLayoutConstraint(item: adjustedView,
                           attribute: .centerX,
                           relatedBy: .equal,
                           toItem: relativeView,
                           attribute: .centerX,
                           multiplier: 1.0,
                           constant: 0).isActive = true
        return adjustedView
    }
    
    /// Center the view vertically in reference to another view
    public func addCenterYConstraint(relativeView: UIView) -> Self {
        let adjustedView = self
        self.translatesAutoresizingMaskIntoConstraints = false
        NSLayoutConstraint(item: adjustedView,
                           attribute: .centerY,
                           relatedBy: .equal,
                           toItem: relativeView,
                           attribute: .centerY,
                           multiplier: 1.0,
                           constant: 0).isActive = true
        return adjustedView
    }
    
    /// Width constraint that set the width of the view as a percentage
    /// (value of the `multiplier`) relative to the `relativeView`
    public func addWidthConstraint(relativeView: UIView, multipler: CGFloat) -> Self {
        let adjustedView = self
        self.translatesAutoresizingMaskIntoConstraints = false
        NSLayoutConstraint(item: adjustedView,
                           attribute: .width,
                           relatedBy: .equal,
                           toItem: relativeView,
                           attribute: .width,
                           multiplier: multipler,
                           constant: 0).isActive = true
        return adjustedView
    }
    
    /// Height constraint that set the height of the view as a percentage
    /// (value of the `multiplier`) relative to the `relativeView`
    public func addHeightConstraint(relativeView: UIView, multipler: CGFloat) -> Self {
        let adjustedView = self
        adjustedView.translatesAutoresizingMaskIntoConstraints = false
        NSLayoutConstraint(item: adjustedView,
                           attribute: .height,
                           relatedBy: .equal,
                           toItem: relativeView,
                           attribute: .height,
                           multiplier: multipler,
                           constant: 0).isActive = true
        return adjustedView
    }

    /// Get the ratio between the height and the width to adjust the height
    /// in case of a change in the value of the width from a constraint.
    public func keepHeightAspectRatio() -> Self {
        let adjustedView = self
        self.translatesAutoresizingMaskIntoConstraints = false
        let multiplier = adjustedView.frame.size.height / adjustedView.frame.size.width
        NSLayoutConstraint(item: adjustedView,
                           attribute: .height,
                           relatedBy: .equal,
                           toItem: adjustedView,
                           attribute: .width,
                           multiplier: multiplier,
                           constant: 0).isActive = true
        return adjustedView
    }
    
    /// Get the ratio between the height and the width to adjust the width
    /// in case of a change in the value of the height from a constraint.
    public func keepWidthAspectRatio() -> Self {
        let adjustedView = self
        self.translatesAutoresizingMaskIntoConstraints = false
        let multiplier = adjustedView.frame.size.width / adjustedView.frame.size.height
        NSLayoutConstraint(item: adjustedView,
                           attribute: .width,
                           relatedBy: .equal,
                           toItem: adjustedView,
                           attribute: .height,
                           multiplier: multiplier,
                           constant: 0).isActive = true
        return adjustedView
    }
    
    /// Set the top of the view with respect to any `attribute` of a second `relativeView`
    public func setConstraintWithConstant(
        selfAttribute: NSLayoutConstraint.Attribute,
        relativeView: UIView,
        relativeAttribute: NSLayoutConstraint.Attribute,
        constant: CGFloat
        ) -> Self {
        let adjustedView = self
        adjustedView.translatesAutoresizingMaskIntoConstraints = false
        NSLayoutConstraint(item: adjustedView,
                           attribute: selfAttribute,
                           relatedBy: .equal,
                           toItem: relativeView,
                           attribute: relativeAttribute,
                           multiplier: 1.0,
                           constant: constant).isActive = true
        return adjustedView
    }
    
    /// Set the top of the view with respect to any `attribute` of a second `relativeView`
    public func setEqualConstraint(
        selfAttribute: NSLayoutConstraint.Attribute,
        relativeView: UIView,
        relativeAttribute: NSLayoutConstraint.Attribute
    ) -> Self {
        let adjustedView = self
        adjustedView.translatesAutoresizingMaskIntoConstraints = false
        NSLayoutConstraint(item: adjustedView,
                           attribute: selfAttribute,
                           relatedBy: .equal,
                           toItem: relativeView,
                           attribute: relativeAttribute,
                           multiplier: 1.0,
                           constant: 0).isActive = true
        return adjustedView
    }
}
