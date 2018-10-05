import UIKit

class LoginView : UIView {
    
    var usernameTextField: CustomTextfield
    
    var passwordTextFiled: CustomTextfield
    
    var loginButton: UIButton
    
    var signUpLabel: UILabel
    
    var pinitLogo: UIImageView
    
    init () {
        usernameTextField = CustomTextfield(frame: CGRect.zero)
        passwordTextFiled = CustomTextfield(frame: CGRect.zero)
        loginButton = UIButton(frame: CGRect.zero)
        signUpLabel = UILabel(frame: CGRect.zero)
        pinitLogo = UIImageView(image: UIImage(named: "pinitIcon"))
        super.init(frame: CGRect.zero)
        
        self.addSubview(pinitLogo)
        self.addSubview(usernameTextField)
        self.addSubview(passwordTextFiled)
        self.addSubview(loginButton)
        self.addSubview(signUpLabel)
        
        pinitLogo = pinitLogo
            .centerViewHorizontalyToSuperView()
            .addHeightConstraint(relativeView: self, multipler: 0.15)
            .keepWidthAspectRatio()
            .addTopConstraint(relativeView: self, attribute: .top, constant: 10)
    }
    
    public func adjustViews() {
        let spacing = self.frame.size.height * 0.05
        usernameTextField = customizeTextfields(textfield: usernameTextField)
        passwordTextFiled = customizeTextfields(textfield: passwordTextFiled)
        
        usernameTextField.placeholder = "Username"
        passwordTextFiled.placeholder = "Password"
        passwordTextFiled.isSecureTextEntry = true
        loginButton.backgroundColor = UIColor(red:0.20, green:0.74, blue:0.83, alpha:1.0)
        loginButton.layer.cornerRadius = 10
        loginButton.setTitle("Login", for: .normal)
        
        signUpLabel.text = "Don't have an account? Sign Up"
        signUpLabel.textColor = UIColor(red:0.24, green:0.60, blue:0.93, alpha:1.0)
        signUpLabel.textAlignment = .center
        signUpLabel.adjustsFontSizeToFitWidth = true

        usernameTextField = usernameTextField.addTopConstraint(
            relativeView: pinitLogo,
            attribute: .bottom,
            constant: spacing)
        
        passwordTextFiled = passwordTextFiled.addTopConstraint(
            relativeView: usernameTextField,
            attribute: .bottom,
            constant: spacing)
        
        loginButton = loginButton
            .centerViewHorizontalyToSuperView()
            .addWidthConstraint(relativeView: self, multipler: 1)
            .addHeightConstraint(relativeView: self, multipler: 0.1)
            .addTopConstraint(
                relativeView: passwordTextFiled,
                attribute: .bottom,
                constant: spacing)
        
        signUpLabel = signUpLabel
            .centerViewHorizontalyToSuperView()
            .addWidthConstraint(relativeView: self, multipler: 0.6)
            .addTopConstraint(
                relativeView: loginButton,
                attribute: .bottom,
                constant: spacing)
        
    }
    
    private func customizeTextfields(textfield: CustomTextfield) -> CustomTextfield {
        let adjustedTextfield = textfield
            .centerViewHorizontalyToSuperView()
            .addWidthConstraint(relativeView: self, multipler: 1)
            .addHeightConstraint(relativeView: self, multipler: 0.1)
        adjustedTextfield.layer.cornerRadius = 10
        adjustedTextfield.backgroundColor = UIColor(red:0.98, green:0.98, blue:0.98, alpha:1.0)
        adjustedTextfield.layer.borderWidth = 0.5
        adjustedTextfield.layer.borderColor =
            UIColor(red:0.60, green:0.69, blue:0.27, alpha:1.0).cgColor
        return adjustedTextfield
    }
    
    required init?(coder aDecoder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }
    
}
