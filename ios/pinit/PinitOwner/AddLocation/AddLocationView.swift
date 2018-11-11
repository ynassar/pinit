import UIKit

class AddLocationView : UIView {
    
    public var closeButton: UIButton!
    
    public var doneButton: UIButton!
    
    public var locationName: CustomTextField!
    
    public var locationDescription: CustomTextField!
    
    init() {
        closeButton = UIButton(frame: CGRect.zero)
        doneButton = UIButton(frame: CGRect.zero)
        locationName = CustomTextField()
        locationDescription = CustomTextField()
        super.init(frame: CGRect.zero)
                
        self.addSubview(closeButton)
        self.addSubview(doneButton)
        self.addSubview(locationName)
        self.addSubview(locationDescription)
        
        locationName.placeholder = "Location Name"
        locationDescription.placeholder = "Location Description (Optional)"
    }
    
    public func updateView() {
        closeButton.setImage(UIImage(named: "closeIcon"), for: .normal)
        doneButton.setImage(UIImage(named: "doneIcon"), for: .normal)
        
        locationName = locationName.customizeTextField(superView: self)
        locationDescription = locationDescription.customizeTextField(superView: self)
        
        locationName.layer.borderWidth = 0.5
        locationName.layer.borderColor = PinitColors.borderGray.cgColor

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
            .setConstraintWithConstant(selfAttribute: .left,
                                       relativeView: self,
                                       relativeAttribute: .left,
                                       constant: spacing)
        
        doneButton = doneButton
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
        
        locationName = locationName
            .setConstraintWithConstant(selfAttribute: .top,
                                       relativeView: closeButton,
                                       relativeAttribute: .bottom,
                                       constant: spacing)
        
        locationDescription = locationDescription
            .setConstraintWithConstant(selfAttribute: .top,
                                       relativeView: locationName,
                                       relativeAttribute: .bottom,
                                       constant: 0)
    }
    
    required init?(coder aDecoder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }
    
}

fileprivate extension CustomTextField {
    fileprivate func customizeTextField(superView: UIView) -> CustomTextField {
        var adjustedTextField = self
        adjustedTextField = adjustedTextField
            .addWidthConstraint(relativeView: superView, multipler: 1.0)
            .addHeightConstraint(relativeView: superView, multipler: 0.05)
            .addCenterXConstraint(relativeView: superView)
        adjustedTextField.font = self.font?
            .withSize(PinitConstants.authenticationFontSize)
        return adjustedTextField
    }
}

