import UIKit

class ProfileView : UIView, PinitViewProtocol {
    
    public var logoutButton: UIButton
    
    public var closeButton: UIButton

    init() {
        logoutButton = UIButton(frame: CGRect.zero)
        closeButton = UIButton(frame: CGRect.zero)
        super.init(frame: CGRect.zero)
        self.addSubview(logoutButton)
        self.addSubview(closeButton)
    }
    
    public func updateView() {
        logoutButton.setTitle("Logout", for: .normal)
        closeButton.setImage(UIImage(named: "closeIcon"), for: .normal)
        
        logoutButton = logoutButton
            .addCenterXConstraint(relativeView: self)
            .addCenterYConstraint(relativeView: self)
            .addWidthConstraint(relativeView: self, multipler: 0.8)
            .addHeightConstraint(relativeView: self, multipler: 0.05)
        
        logoutButton.layer.cornerRadius = 10
        logoutButton.titleLabel?.font = logoutButton.titleLabel?.font
            .withSize(PinitConstants.authenticationFontSize)
        
        let spacing = self.frame.size.width * 0.05
        
        closeButton = closeButton
            .addWidthConstraint(relativeView: self, multipler: 0.1)
            .setEqualConstraint(selfAttribute: .height,
                                relativeView: closeButton,
                                relativeAttribute: .width)
            .setConstraintWithConstant(selfAttribute: .top,
                                       relativeView: self,
                                       relativeAttribute: .top,
                                       constant: UIApplication.shared.statusBarFrame.size.height)
            .setConstraintWithConstant(selfAttribute: .right,
                                       relativeView: self,
                                       relativeAttribute: .right,
                                       constant: -spacing)

    }
    
    required init?(coder aDecoder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }
}
