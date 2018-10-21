import UIKit

/// 'MappingView' is responsible for having the start mapping button and
/// the imageview of the mapping image updated periodically.
class MappingView: UIView {
    
    var startMappingButton: UIButton
    
    /// Initializer of the `MappingView`.
    init() {
        startMappingButton = UIButton(frame: CGRect.zero)
        super.init(frame: CGRect.zero)
        self.addSubview(startMappingButton)
        startMappingButton.setImage(UIImage(named: "startMapping"), for: .normal)
    }
    
    /// Update the view by adding the constraints to make sure that the frame width
    /// is set. 
    public func updateView() {
        let spacing = self.frame.width * 0.05
        
        startMappingButton = startMappingButton
            .addHeightConstraint(relativeView: self, multipler: 0.1)
            .setEqualConstraint(selfAttribute: .width,
                                relativeView: startMappingButton,
                                relativeAttribute: .height)
            .setConstraintWithConstant(selfAttribute: .top,
                                       relativeView: self,
                                       relativeAttribute: .top,
                                       constant: spacing)
            .setConstraintWithConstant(selfAttribute: .right,
                                       relativeView: self,
                                       relativeAttribute: .right,
                                       constant: -spacing)
        
    }
    
    required init?(coder aDecoder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }
}
