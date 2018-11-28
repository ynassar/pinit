import UIKit

class RobotRequestView : UIView, PinitViewProtocol {
    
    public var pickUpLocationTextFeild: CustomTextField
    
    public var destinationLocationTextFiled: CustomTextField

    public var getGpsCoordinatesButton: UIButton
    
    init() {
        getGpsCoordinatesButton = UIButton(frame: CGRect.zero)
        pickUpLocationTextFeild = CustomTextField()
        destinationLocationTextFiled = CustomTextField()
        super.init(frame: CGRect.zero)
        
        self.addSubview(getGpsCoordinatesButton)
        self.addSubview(pickUpLocationTextFeild)
        self.addSubview(destinationLocationTextFiled)
    }
    
    required init?(coder aDecoder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }
    
    func updateView() {
        
        pickUpLocationTextFeild.placeholder = "Enter your pick up location"
        destinationLocationTextFiled.placeholder = "Enter your destination"
        
        pickUpLocationTextFeild.customizeTextField()
        destinationLocationTextFiled.customizeTextField()
        
        pickUpLocationTextFeild.rightViewMode = .always
        destinationLocationTextFiled.rightViewMode = .always
        
        pickUpLocationTextFeild.rightView = UIImageView(image: UIImage(named: "exclamationMarkIcon"))
        destinationLocationTextFiled.rightView = UIImageView(image: UIImage(named: "exclamationMarkIcon"))
        
        getGpsCoordinatesButton.setTitle("Request Robot", for: .normal)
        getGpsCoordinatesButton.backgroundColor = UIColor.black
        getGpsCoordinatesButton.setTitleColor(.white, for: .normal)
        getGpsCoordinatesButton.titleLabel?.font = UIFont(name: "Avenir", size: 14.0)
        
        let spacing = self.bounds.height * 0.15
        
        pickUpLocationTextFeild = pickUpLocationTextFeild
            .addCenterXConstraint(relativeView: self)
            .addWidthConstraint(relativeView: self, multipler: 0.8)
            .addHeightConstraint(relativeView: self, multipler: 0.15)
            .setConstraintWithConstant(selfAttribute: .top,
                                       relativeView: self,
                                       relativeAttribute: .top,
                                       constant: 0)
        
        
        destinationLocationTextFiled = destinationLocationTextFiled
            .addCenterXConstraint(relativeView: self)
            .addWidthConstraint(relativeView: self, multipler: 0.8)
            .addHeightConstraint(relativeView: self, multipler: 0.15)
            .setConstraintWithConstant(selfAttribute: .top,
                                       relativeView: pickUpLocationTextFeild,
                                       relativeAttribute: .bottom,
                                       constant: spacing)
        
        getGpsCoordinatesButton = getGpsCoordinatesButton
            .addCenterXConstraint(relativeView: self)
            .addHeightConstraint(relativeView: self, multipler: 0.4)
            .setEqualConstraint(selfAttribute: .width,
                                relativeView: getGpsCoordinatesButton,
                                relativeAttribute: .height)
            .setConstraintWithConstant(selfAttribute: .top,
                                       relativeView: destinationLocationTextFiled,
                                       relativeAttribute: .bottom,
                                       constant: spacing)
    }
}

fileprivate extension CustomTextField {
    
    fileprivate func customizeTextField() {
        self.layer.cornerRadius = PinitConstants.cornerRadius
        self.layer.borderWidth = 0.5
        self.backgroundColor = .white
        self.font = UIFont(name: "Avenir", size: 14.0)
        self.layer.borderColor = PinitColors.borderGray.cgColor
        
        self.layer.masksToBounds = false
        self.layer.shadowRadius = 1.0
        self.layer.shadowColor = UIColor.black.cgColor
        self.layer.shadowOffset = CGSize(width: 1.0, height: 1.0)
        self.layer.shadowOpacity = 0.5
    }
    
}
