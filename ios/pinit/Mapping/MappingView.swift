import UIKit

/// 'MappingView' is responsible for having the start mapping button and
/// the imageview of the mapping image updated periodically.
class MappingView: UIView {
    
    var startMappingButton: UIButton
    
    var mapImage: UIImageView!
    
    /// Initializer of the `MappingView`.
    init() {
        startMappingButton = UIButton(frame: CGRect.zero)
        mapImage = UIImageView(image: UIImage(named: "mapImage"))
        super.init(frame: CGRect.zero)
        self.addSubview(startMappingButton)
        self.addSubview(mapImage)
        startMappingButton.setImage(UIImage(named: "startMapping"), for: .normal)
    }
    
    /// Update the view by adding the constraints to make sure that the frame width
    /// is set. 
    public func updateView() {
        let spacing = self.frame.width * 0.02
        
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
        
        mapImage = mapImage
            .addWidthConstraint(relativeView: self, multipler: 1.0)
            .setConstraintWithConstant(selfAttribute: .top,
                                       relativeView: startMappingButton,
                                       relativeAttribute: .bottom,
                                       constant: spacing)
            .setConstraintWithConstant(selfAttribute: .bottom,
                                       relativeView: self,
                                       relativeAttribute: .bottom,
                                       constant: 0)
        
    }
    
    required init?(coder aDecoder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }
}
