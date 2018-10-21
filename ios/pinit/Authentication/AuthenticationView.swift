import UIKit

class AuthenticationView : UIView {
    
    /// The pinit logo at the top of any authentication view
    var pinitLogo: UIImageView
    
    override init(frame: CGRect) {
        pinitLogo = UIImageView(image: UIImage(named: "pinitIcon"))
        super.init(frame: frame)
        self.addSubview(pinitLogo)
    
        pinitLogo = pinitLogo
            .addCenterXConstraint()
            .addHeightConstraint(relativeView: self, multipler: 0.15)
            .keepWidthAspectRatio()
            .setConstraintWithConstant(
                selfAttribute: .top,
                relativeView: self,
                relativeAttribute: .top,
                constant: 10)
        
    }
    
    internal func customizeTextfields(textfield: CustomTextField) -> CustomTextField {
        let adjustedTextfield = textfield
            .addCenterXConstraint()
            .addWidthConstraint(relativeView: self, multipler: 1)
            .addHeightConstraint(relativeView: self, multipler: 0.1)
        adjustedTextfield.layer.cornerRadius = PinitConstants.cornerRadius
        adjustedTextfield.backgroundColor = PinitColors.gray
        adjustedTextfield.layer.borderWidth = 0.5
        adjustedTextfield.layer.borderColor = PinitColors.borderGreen.cgColor
        adjustedTextfield.font = adjustedTextfield.font?
            .withSize(PinitConstants.authernticationFontSize)
        return adjustedTextfield
    }
    
    internal func customizeButton(button: UIButton) -> UIButton {
        let adjustedButton = button
            .addCenterXConstraint()
            .addWidthConstraint(relativeView: self, multipler: 1)
            .addHeightConstraint(relativeView: self, multipler: 0.1)
        adjustedButton.backgroundColor = PinitColors.blue
        adjustedButton.layer.cornerRadius = 10
        adjustedButton.titleLabel?.font = adjustedButton.titleLabel?.font
            .withSize(PinitConstants.authernticationFontSize)
        return adjustedButton
    }
    
    internal func customizeLinkLabel(label: UILabel) -> UILabel {
        let adjustedLabel = label
            .addCenterXConstraint()
            .addWidthConstraint(relativeView: self, multipler: 0.6)
        label.textColor = PinitColors.linkBlue
        label.textAlignment = .center
        label.adjustsFontSizeToFitWidth = true
        adjustedLabel.font = adjustedLabel.font
            .withSize(PinitConstants.authernticationFontSize)
        return adjustedLabel
    }
    
    required init?(coder aDecoder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }
}
