import UIKit

/// The `RegisterView` is responsible for registering in the app creating
/// a general user account.
class RegisterView : AuthenticationView, PinitViewProtocol {
    
    /// The username text field.
    public var usernameTextField: CustomTextField
    
    /// The password text field.
    public var passwordTextField: CustomTextField

    /// The confrim password text field.
    public var confrimPasswordTextField: CustomTextField

    /// The username text field.
    public var emailTextField: CustomTextField

    /// The button used to login.
    public var registerButton: UIButton

    /// The sign in label when selected directs the user to `LoginView`.
    public var signInLabel: UILabel

    /// The initializer of the `RegisterView` which creates the subviews and add them.
    init () {
        usernameTextField = CustomTextField(frame: CGRect.zero)
        passwordTextField = CustomTextField(frame: CGRect.zero)
        confrimPasswordTextField = CustomTextField(frame: CGRect.zero)
        emailTextField = CustomTextField(frame: CGRect.zero)
        registerButton = UIButton(frame: CGRect.zero)
        signInLabel = UILabel(frame: CGRect.zero)
        super.init(frame: CGRect.zero)
        
        self.addSubview(usernameTextField)
        self.addSubview(passwordTextField)
        self.addSubview(confrimPasswordTextField)
        self.addSubview(emailTextField)
        self.addSubview(registerButton)
        self.addSubview(signInLabel)
    }
    
    /// Updating the subviews after making sure that the `RegisterView` itself
    /// has been set its contraints. Called after the view appears.
    public func updateView() {
        let spacing = self.frame.size.height * 0.05
        usernameTextField = customizeTextfields(textfield: usernameTextField)
        passwordTextField = customizeTextfields(textfield: passwordTextField)
        confrimPasswordTextField = customizeTextfields(textfield: confrimPasswordTextField)
        emailTextField = customizeTextfields(textfield: emailTextField)
        
        registerButton = customizeButton(button: registerButton)
        signInLabel = customizeLinkLabel(label: signInLabel)
        
        usernameTextField.placeholder = "Username"
        passwordTextField.placeholder = "Password"
        confrimPasswordTextField.placeholder = "Confirm Password"
        emailTextField.placeholder = "Email"
        
        passwordTextField.isSecureTextEntry = true
        confrimPasswordTextField.isSecureTextEntry = true
        
        registerButton.setTitle("Register", for: .normal)

        signInLabel.text = "Already have an account? Sign In"

        usernameTextField = usernameTextField
            .setConstraintWithConstant(
                selfAttribute: .top,
                relativeView: pinitLogo,
                relativeAttribute: .bottom,
                constant: spacing)

        passwordTextField = passwordTextField
            .setConstraintWithConstant(
                selfAttribute: .top,
                relativeView: usernameTextField,
                relativeAttribute: .bottom,
                constant: spacing)

        confrimPasswordTextField = confrimPasswordTextField
            .setConstraintWithConstant(
                selfAttribute: .top,
                relativeView: passwordTextField,
                relativeAttribute: .bottom,
                constant: spacing)
        
        emailTextField = emailTextField
            .setConstraintWithConstant(
                selfAttribute: .top,
                relativeView: confrimPasswordTextField,
                relativeAttribute: .bottom,
                constant: spacing)
        
        registerButton = registerButton
            .setConstraintWithConstant(
                selfAttribute: .top,
                relativeView: emailTextField,
                relativeAttribute: .bottom,
                constant: spacing)
        
        signInLabel = signInLabel
            .setConstraintWithConstant(
                selfAttribute: .top,
                relativeView: registerButton,
                relativeAttribute: .bottom,
                constant: spacing)        
    }
    
    required init?(coder aDecoder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }
}

