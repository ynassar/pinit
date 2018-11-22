import UIKit

class ProfileView : UIView, PinitViewProtocol {
    
    public var logoutButton: UIButton
    
    public var closeButton: UIButton!
    
    public var usernameLabel: UILabel!
    
    init() {
        logoutButton = UIButton(frame: CGRect.zero)
        closeButton = UIButton(frame: CGRect.zero)
        usernameLabel = UILabel(frame: CGRect.zero)
        super.init(frame: CGRect.zero)
        self.addSubview(logoutButton)
        self.addSubview(usernameLabel)
        self.addSubview(closeButton)
    }
    
    public func updateView() {
        logoutButton.setTitle("Logout", for: .normal)
        logoutButton.backgroundColor = .white
        logoutButton.setTitleColor(.black, for: .normal)
        
        closeButton.setImage(UIImage(named: "closeIconWhite"), for: .normal)
        
        let spacing = self.frame.size.width * 0.05

        logoutButton = logoutButton
            .addCenterXConstraint(relativeView: self)
            .addCenterYConstraint(relativeView: self)
            .addWidthConstraint(relativeView: self, multipler: 0.8)
            .addHeightConstraint(relativeView: self, multipler: 0.05)
        
        logoutButton.layer.cornerRadius = 10
        logoutButton.titleLabel?.font = logoutButton.titleLabel?.font
            .withSize(PinitConstants.authenticationFontSize)

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
    
    public func addGradiant() {
        let gradiantLayer = CAGradientLayer()
        gradiantLayer.frame = self.bounds
        gradiantLayer.startPoint = CGPoint(x: 0, y: 0)
        gradiantLayer.endPoint = CGPoint(x: 1, y: 0)
        gradiantLayer.colors = [PinitColors.yellow.cgColor,
                                PinitColors.red.cgColor]
        self.layer.insertSublayer(gradiantLayer, at: 0)

    }
    
    required init?(coder aDecoder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }
}
