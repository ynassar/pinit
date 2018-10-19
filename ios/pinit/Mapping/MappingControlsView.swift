import UIKit

class MappingControlsView: UIView {
    
    public var moveForwardButton: UIButton
    
    public var moveBackwardButton: UIButton
    
    public var moveLeftButton: UIButton
    
    public var moveRightButton: UIButton
    
    init() {
        
        moveForwardButton = UIButton(frame: CGRect.zero)
        moveRightButton = UIButton(frame: CGRect.zero)
        moveLeftButton = UIButton(frame: CGRect.zero)
        moveBackwardButton = UIButton(frame: CGRect.zero)
        super.init(frame: CGRect.zero)
        
//        self.backgroundColor = .red
        
        self.addSubview(moveForwardButton)
        self.addSubview(moveRightButton)
        self.addSubview(moveLeftButton)
        self.addSubview(moveBackwardButton)
    }
    
    public func updateView() {
        
//        moveForwardButton.setTitle("Forward", for: .normal)
//        moveRightButton.setTitle("Right", for: .normal)
//        moveLeftButton.setTitle("Left", for: .normal)
//        moveBackwardButton.setTitle("Backwards", for: .normal)
        
        moveForwardButton.setImage(UIImage(named: "moveForwardIcon"), for: .normal)
        moveRightButton.setImage(UIImage(named: "moveRightIcon"), for: .normal)
        moveLeftButton.setImage(UIImage(named: "moveLeftIcon"), for: .normal)
        moveBackwardButton.setImage(UIImage(named: "moveBackwardIcon"), for: .normal)
        
        moveRightButton.semanticContentAttribute = UIApplication.shared
            .userInterfaceLayoutDirection == .rightToLeft ? .forceLeftToRight : .forceRightToLeft
        
        moveForwardButton = moveForwardButton
            .addCenterXConstraint()
            .addWidthConstraint(relativeView: self, multipler: 0.3)
            .addHeightConstraint(relativeView: self, multipler: 0.4)
            .addTopConstraint(relativeView: self, attribute: .top, constant: 0)
        
        moveRightButton = moveRightButton
            .addCenterYConstraint()
            .addHeightConstraint(relativeView: self, multipler: 0.4)
            .addRightConstraint(relativeView: self, attribute: .right, constant: 0)
            .addLeftConstraint(relativeView: moveForwardButton , attribute: .right, constant: 0)

        moveLeftButton = moveLeftButton
            .addCenterYConstraint()
            .addHeightConstraint(relativeView: self, multipler: 0.4)
            .addLeftConstraint(relativeView: self, attribute: .left, constant: 0)
            .addRightConstraint(relativeView: moveForwardButton , attribute: .left, constant: 0)
        
        moveBackwardButton = moveBackwardButton
            .addCenterXConstraint()
            .addWidthConstraint(relativeView: self, multipler: 0.3)
            .addHeightConstraint(relativeView: self, multipler: 0.4)
            .addBottomConstraint(relativeView: self, attribute: .bottom, constant: 0)
        
    }
    
    required init?(coder aDecoder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }
    
}
