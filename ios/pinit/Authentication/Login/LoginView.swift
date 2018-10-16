import UIKit

/// The `LoginView` is responsible for logging in the app using the
/// correct username and password. 
class LoginView : AuthenticationView {
    
    /// The username text field.
    public var usernameTextField: CustomTextField
    
    /// The password text field.
    public var passwordTextFiled: CustomTextField
    
    /// The button used to login.
    public var loginButton: UIButton
    
    /// The sign up label when selected directs the user to `RegisterView`.
    public var signUpLabel: UILabel
    
    
    /// The initializer of the `LoginView` which creates the subviews and add them.
    init () {
        usernameTextField = CustomTextField(frame: CGRect.zero)
        passwordTextFiled = CustomTextField(frame: CGRect.zero)
        loginButton = UIButton(frame: CGRect.zero)
        signUpLabel = UILabel(frame: CGRect.zero)
        super.init(frame: CGRect.zero)
        
        self.addSubview(usernameTextField)
        self.addSubview(passwordTextFiled)
        self.addSubview(loginButton)
        self.addSubview(signUpLabel)
    }
    
    /// Updating the subviews after making sure that the `LoginView` itself
    /// has been set its contraints. Called after the view appears.
    public func updateView() {
        let spacing = self.frame.size.height * 0.05
        usernameTextField = customizeTextfields(textfield: usernameTextField)
        passwordTextFiled = customizeTextfields(textfield: passwordTextFiled)
        loginButton = customizeButton(button: loginButton)
        signUpLabel = customizeLinkLabel(label: signUpLabel)
        
        usernameTextField.placeholder = "Username"
        passwordTextFiled.placeholder = "Password"
        passwordTextFiled.isSecureTextEntry = true
        
        loginButton.setTitle("Login", for: .normal)
        
        signUpLabel.text = "Don't have an account? Sign Up"
        
        usernameTextField = usernameTextField.addTopConstraint(
            relativeView: pinitLogo,
            attribute: .bottom,
            constant: spacing)
        
        passwordTextFiled = passwordTextFiled.addTopConstraint(
            relativeView: usernameTextField,
            attribute: .bottom,
            constant: spacing)
        
        loginButton = loginButton
            .addTopConstraint(
                relativeView: passwordTextFiled,
                attribute: .bottom,
                constant: spacing)
        
        signUpLabel = signUpLabel
            .addCenterXConstraint()
            .addWidthConstraint(relativeView: self, multipler: 0.6)
            .addTopConstraint(
                relativeView: loginButton,
                attribute: .bottom,
                constant: spacing)
    }
    
    required init?(coder aDecoder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }
    
}
