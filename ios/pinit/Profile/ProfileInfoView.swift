import UIKit

class ProfileInfoView: UIView, PinitViewProtocol {
    
    public var closeButton: UIButton!
    
    public var usernameLabel: UILabel!
    
    init() {
        closeButton = UIButton(frame: CGRect.zero)
        usernameLabel = UILabel(frame: CGRect.zero)
        super.init(frame: CGRect.zero)
        self.addSubview(usernameLabel)
        self.addSubview(closeButton)

    }
    
    func updateView() {
        closeButton.setImage(UIImage(named: "closeIconWhite"), for: .normal)
        
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
