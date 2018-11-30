import UIKit

/// View responsible for the `SelectLocationViewController` search bar and other options.
class SelectLocationSearchBarView: UIView, PinitViewProtocol {
    
    /// Button responsible for returing back to the previous view controller.
    public var backButton: UIButton
    
    /// Custom search bar in whic the user types the name of the locations
    /// they're looking for.
    public var searchBar: CustomTextField
    
    /// Initializer for the search view in which the back button and search bar is
    /// added to the view.
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
    
    /// Function to add the constrains for the different UI elements inside the view.
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
    }
    
    /// Adding a gradiant background to the view.
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

/// Extension for the search bar text field.
fileprivate extension CustomTextField {
    
    /// Function to customize the search bar by setting the colors, font, border and placeholder.
    fileprivate func customizeSearchBar() {
        self.backgroundColor = .white
        self.placeholder = "Enter Location ... "
        self.layer.cornerRadius = 5
        self.font = UIFont(name: "Avenir", size: 12)
        
        self.layer.borderWidth = 0.5
        self.layer.borderColor = PinitColors.borderGray.cgColor
    }
}
