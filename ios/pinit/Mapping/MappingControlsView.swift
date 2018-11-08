import UIKit

/// `MappingControlsView` is the view responsible for the controls used to
/// move the robot during mapping.
class MappingControlsView: UIView {
    
    /// Control to move the robot to the front.
    public var moveForwardButton: UIButton
    
    /// Control to move the robot to the right.
    public var moveBackwardButton: UIButton
    
    /// Control to move the robot to the left.
    public var moveLeftButton: UIButton
    
    /// Control to move the robot to the back.
    public var moveRightButton: UIButton
    
    /// Initializer of the `MappingView`.
    init() {
        moveForwardButton = UIButton(frame: CGRect.zero)
        moveRightButton = UIButton(frame: CGRect.zero)
        moveLeftButton = UIButton(frame: CGRect.zero)
        moveBackwardButton = UIButton(frame: CGRect.zero)
        super.init(frame: CGRect.zero)
                
        self.addSubview(moveForwardButton)
        self.addSubview(moveRightButton)
        self.addSubview(moveLeftButton)
        self.addSubview(moveBackwardButton)
    }
    
    /// Update the constraints of the view buttons and adding the images to the buttons.
    public func updateView() {
        moveForwardButton.setImage(UIImage(named: "moveForwardIcon"), for: .normal)
        moveRightButton.setImage(UIImage(named: "moveRightIcon"), for: .normal)
        moveLeftButton.setImage(UIImage(named: "moveLeftIcon"), for: .normal)
        moveBackwardButton.setImage(UIImage(named: "moveBackwardIcon"), for: .normal)

        moveForwardButton = moveForwardButton
            .addCenterXConstraint(relativeView: self)
            .addWidthConstraint(relativeView: self, multipler: 0.25)
            .addHeightConstraint(relativeView: self, multipler: 0.35)
            .setConstraintWithConstant(selfAttribute: .top,
                                       relativeView: self,
                                       relativeAttribute: .top,
                                       constant: 0)
        
        moveRightButton = moveRightButton
            .addCenterYConstraint(relativeView: self)
            .setEqualConstraint(selfAttribute: .height,
                                relativeView: moveForwardButton,
                                relativeAttribute: .width)
            .setEqualConstraint(selfAttribute: .width,
                                relativeView: moveForwardButton,
                                relativeAttribute: .height)
            .setConstraintWithConstant(selfAttribute: .left,
                                       relativeView: moveForwardButton,
                                       relativeAttribute: .right,
                                       constant: 0)
        
        moveLeftButton = moveLeftButton
            .addCenterYConstraint(relativeView: self)
            .setEqualConstraint(selfAttribute: .height,
                                relativeView: moveForwardButton,
                                relativeAttribute: .width)
            .setEqualConstraint(selfAttribute: .width,
                                relativeView: moveForwardButton,
                                relativeAttribute: .height)
            .setConstraintWithConstant(selfAttribute: .right,
                                       relativeView: moveForwardButton,
                                       relativeAttribute: .left,
                                       constant: 0)
        
        moveBackwardButton = moveBackwardButton
            .addCenterXConstraint(relativeView: self)
            .addWidthConstraint(relativeView: self, multipler: 0.25)
            .addHeightConstraint(relativeView: self, multipler: 0.35)
            .setConstraintWithConstant(selfAttribute: .bottom,
                                       relativeView: self,
                                       relativeAttribute: .bottom,
                                       constant: 0)
        
    }
    
    /// Function to disable all the control buttons.
    public func disableControls() {
        moveForwardButton.disableButton()
        moveRightButton.disableButton()
        moveLeftButton.disableButton()
        moveBackwardButton.disableButton()
    }
    
    /// Function to enable all the control buttons. 
    public func enableControls() {
        moveForwardButton.enableButton()
        moveRightButton.enableButton()
        moveLeftButton.enableButton()
        moveBackwardButton.enableButton()
    }
    
    required init?(coder aDecoder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }
    
}
