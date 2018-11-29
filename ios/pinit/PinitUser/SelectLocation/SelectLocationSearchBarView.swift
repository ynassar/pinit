import UIKit

class SelectLocationSearchBarView: UIView, PinitViewProtocol {
    
    public var backButton: UIButton
    
    public var searchBar: CustomTextField
    
    init() {
        backButton = UIButton(frame: CGRect.zero)
        searchBar = CustomTextField()
        super.init(frame: CGRect.zero)
        self.backgroundColor = .white
        
        self.addSubview(backButton)
        self.addSubview(searchBar)
        
        searchBar.becomeFirstResponder()
    }
    
    required init?(coder aDecoder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }
    
    public func updateView() {
        
        addGradient()
        
        backButton.setImage(UIImage(named: "backIcon"), for: .normal)
        searchBar.customizeSearchBar()
        
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
        
        searchBar = searchBar
            .addCenterYConstraint(relativeView: self)
            .addWidthConstraint(relativeView: self, multipler: 0.70)
            .addHeightConstraint(relativeView: self, multipler: 0.5)
            .setConstraintWithConstant(selfAttribute: .left,
                                       relativeView: backButton,
                                       relativeAttribute: .right,
                                       constant: leftPadding)
        return
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

}

fileprivate extension CustomTextField {
    
    fileprivate func customizeSearchBar() {
        self.backgroundColor = .white
        self.placeholder = "Enter Location ... "
        self.layer.cornerRadius = 5
        self.font = UIFont(name: "Avenir", size: 12)
        
        self.layer.borderWidth = 0.5
        self.layer.borderColor = PinitColors.borderGray.cgColor
    }
}
