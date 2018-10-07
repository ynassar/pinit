import UIKit

/// The `RegisterView` is responsible for registering in the app creating
/// a general user account.
class RegisterView : AuthenticationView {
    
    /// The username text field.
    var usernameTextField: CustomTextField
    
    /// The password text field.
    var passwordTextFiled: CustomTextField
    
    /// The confrim password text field.
    var confrimPasswordTextFiled: CustomTextField
    
    /// The username text field.
    var emailTextField: CustomTextField
    
    /// The button used to login.
    var registerButton: UIButton
    
    /// The sign in label when selected directs the user to `LoginView`.
    var signInLabel: UILabel
    
    
    /// The initializer of the `RegisterView` which creates the subviews and add them.
    init () {
        usernameTextField = CustomTextField(frame: CGRect.zero)
        passwordTextFiled = CustomTextField(frame: CGRect.zero)
        confrimPasswordTextFiled = CustomTextField(frame: CGRect.zero)
        emailTextField = CustomTextField(frame: CGRect.zero)
        registerButton = UIButton(frame: CGRect.zero)
        signInLabel = UILabel(frame: CGRect.zero)
        super.init(frame: CGRect.zero)
        
        self.addSubview(usernameTextField)
        self.addSubview(passwordTextFiled)
        self.addSubview(confrimPasswordTextFiled)
        self.addSubview(emailTextField)
        self.addSubview(registerButton)
        self.addSubview(signInLabel)
    }
    
    /// Updating the subviews after making sure that the `RegisterView` itself
    /// has been set its contraints. Called after the view appears.
    public func updateView() {
        let spacing = self.frame.size.height * 0.05
        usernameTextField = customizeTextfields(textfield: usernameTextField)
        passwordTextFiled = customizeTextfields(textfield: passwordTextFiled)
        confrimPasswordTextFiled = customizeTextfields(textfield: confrimPasswordTextFiled)
        emailTextField = customizeTextfields(textfield: emailTextField)
        
        usernameTextField.placeholder = "Username"
        passwordTextFiled.placeholder = "Password"
        confrimPasswordTextFiled.placeholder = "Confirm Password"
        emailTextField.placeholder = "Email"
        
        passwordTextFiled.isSecureTextEntry = true
        confrimPasswordTextFiled.isSecureTextEntry = true
        
        registerButton.setTitle("Register", for: .normal)
        
        signInLabel.text = "Already have an account? Sign In"
        
        usernameTextField = usernameTextField.addTopConstraint(
            relativeView: pinitLogo,
            attribute: .bottom,
            constant: spacing)
        
        passwordTextFiled = passwordTextFiled.addTopConstraint(
            relativeView: usernameTextField,
            attribute: .bottom,
            constant: spacing)
        
        confrimPasswordTextFiled = confrimPasswordTextFiled.addTopConstraint(
            relativeView: passwordTextFiled,
            attribute: .bottom,
            constant: spacing)
        
        emailTextField = emailTextField.addTopConstraint(
            relativeView: confrimPasswordTextFiled,
            attribute: .bottom,
            constant: spacing)
        
        registerButton = registerButton
            .addTopConstraint(
                relativeView: emailTextField,
                attribute: .bottom,
                constant: spacing)
        
        signInLabel = signInLabel
            .addTopConstraint(
                relativeView: registerButton,
                attribute: .bottom,
                constant: spacing)
    }
    
    required init?(coder aDecoder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }
    
}

