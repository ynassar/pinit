import UIKit

extension UIView {
    
    public func centerViewHorizontalyToSuperView() -> Self {
        let adjustedView = self
        self.translatesAutoresizingMaskIntoConstraints = false
        NSLayoutConstraint(item: adjustedView,
                           attribute: .centerX,
                           relatedBy: .equal,
                           toItem: adjustedView.superview,
                           attribute: .centerX,
                           multiplier: 1.0,
                           constant: 0).isActive = true
        return adjustedView
    }
    
    public func centerViewVerticallyToSuperView() -> Self {
        let adjustedView = self
        self.translatesAutoresizingMaskIntoConstraints = false
        NSLayoutConstraint(item: adjustedView,
                           attribute: .centerY,
                           relatedBy: .equal,
                           toItem: adjustedView.superview,
                           attribute: .centerY,
                           multiplier: 1.0,
                           constant: 0).isActive = true
        return adjustedView
    }
    
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

    public func addTopConstraint(
        relativeView: UIView,
        attribute: NSLayoutAttribute,
        constant: CGFloat
    ) -> Self {
        let adjustedView = self
        adjustedView.translatesAutoresizingMaskIntoConstraints = false
        NSLayoutConstraint(item: adjustedView,
                           attribute: .top,
                           relatedBy: .equal,
                           toItem: relativeView,
                           attribute: attribute,
                           multiplier: 1.0,
                           constant: constant).isActive = true
        return adjustedView
    }
    
}
