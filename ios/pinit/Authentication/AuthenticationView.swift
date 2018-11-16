import UIKit

/// `AuthenticationView` is a general view with the essentials of any authentication related
/// screen. Used by the `LoginView` and `RegisterView`.
class AuthenticationView : UIView {
    
    /// The pinit logo at the top of any authentication view
    var pinitLogo: UIImageView
    
    /// Initializes the `AithernticationView` along with the its variables.
    override init(frame: CGRect) {
        pinitLogo = UIImageView(image: UIImage(named: "pinitIcon"))
        super.init(frame: frame)
        self.addSubview(pinitLogo)
    
        pinitLogo = pinitLogo
            .addCenterXConstraint(relativeView: self)
            .addHeightConstraint(relativeView: self, multipler: 0.15)
            .keepWidthAspectRatio()
            .setConstraintWithConstant(
                selfAttribute: .top,
                relativeView: self,
                relativeAttribute: .top,
                constant: 10)
        
    }
    
    /// Method responsible for adjusting the look of any `CustomTextField` inside the view.
    internal func customizeTextfields(textfield: CustomTextField) -> CustomTextField {
        let adjustedTextfield = textfield
            .addCenterXConstraint(relativeView: self)
            .addWidthConstraint(relativeView: self, multipler: 1)
            .addHeightConstraint(relativeView: self, multipler: 0.1)
        adjustedTextfield.layer.cornerRadius = PinitConstants.cornerRadius
        adjustedTextfield.backgroundColor = PinitColors.gray
        adjustedTextfield.layer.borderWidth = 0.5
        adjustedTextfield.layer.borderColor = PinitColors.borderGray.cgColor
        adjustedTextfield.font = adjustedTextfield.font?
            .withSize(PinitConstants.authenticationFontSize)
        return adjustedTextfield
    }
    
    /// Method responsible for adjusting the look of any `UIButton` inside the view.
    internal func customizeButton(button: UIButton) -> UIButton {
        let adjustedButton = button
            .addCenterXConstraint(relativeView: self)
            .addWidthConstraint(relativeView: self, multipler: 1)
            .addHeightConstraint(relativeView: self, multipler: 0.1)
        adjustedButton.backgroundColor = PinitColors.blue
        adjustedButton.layer.cornerRadius = 10
        adjustedButton.titleLabel?.font = adjustedButton.titleLabel?.font
            .withSize(PinitConstants.authenticationFontSize)
        return adjustedButton
    }
    
    /// Method responsible for adjusting the look of any `UILabel` that acts as
    /// a link inside the view.
    internal func customizeLinkLabel(label: UILabel) -> UILabel {
        let adjustedLabel = label
            .addCenterXConstraint(relativeView: self)
            .addWidthConstraint(relativeView: self, multipler: 0.6)
        label.textColor = PinitColors.linkBlue
        label.textAlignment = .center
        label.adjustsFontSizeToFitWidth = true
        adjustedLabel.font = adjustedLabel.font
            .withSize(PinitConstants.authenticationFontSize)
        return adjustedLabel
    }
    
    required init?(coder aDecoder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }
}
