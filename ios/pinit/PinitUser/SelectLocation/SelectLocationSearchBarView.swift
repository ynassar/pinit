import UIKit

class SelectLocationSearchBarView: UIView, PinitViewProtocol {
    
    public var backButton: UIButton
    
    init() {
        backButton = UIButton(frame: CGRect.zero)
        super.init(frame: CGRect.zero)
        self.backgroundColor = .white
        
        self.addSubview(backButton)
    }
    
    required init?(coder aDecoder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }
    
    public func addGradient() {
        let gradiantLayer = CAGradientLayer()
        gradiantLayer.frame = self.bounds
        gradiantLayer.startPoint = CGPoint(x: 0, y: 0)
        gradiantLayer.endPoint = CGPoint(x: 0, y: 1)
        gradiantLayer.colors = [UIColor.white.cgColor,
                                PinitColors.gray.cgColor]
        self.layer.insertSublayer(gradiantLayer, at: 0)
    }
    
    func updateView() {
        
        backButton.setImage(UIImage(named: "backIcon"), for: .normal)
        addGradient()
        
        let leftPadding = self.bounds.width * 0.05
        
        backButton = backButton
            .addCenterYConstraint(relativeView: self)
            .addWidthConstraint(relativeView: self, multipler: 0.1)
            .setEqualConstraint(selfAttribute: .height,
                                relativeView: backButton,
                                relativeAttribute: .width)
            .setConstraintWithConstant(selfAttribute: .left,
                                       relativeView: self,
                                       relativeAttribute: .left,
                                       constant: leftPadding)
        return
    }
}
