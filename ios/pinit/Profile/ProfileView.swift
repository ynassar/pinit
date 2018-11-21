import UIKit

class ProfileView : UIView, PinitViewProtocol {
    
    public var logoutButton: UIButton
    
    init() {
        logoutButton = UIButton(frame: CGRect.zero)
        super.init(frame: CGRect.zero)
        self.addSubview(logoutButton)
    }
    
    public func updateView() {
        logoutButton.setTitle("Logout", for: .normal)
        
        logoutButton = logoutButton
            .addCenterXConstraint(relativeView: self)
            .addCenterYConstraint(relativeView: self)
            .addWidthConstraint(relativeView: self, multipler: 0.8)
            .addHeightConstraint(relativeView: self, multipler: 0.05)
        
        logoutButton.layer.cornerRadius = 10
        logoutButton.titleLabel?.font = logoutButton.titleLabel?.font
            .withSize(PinitConstants.authenticationFontSize)

    }
    
    required init?(coder aDecoder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }
}
