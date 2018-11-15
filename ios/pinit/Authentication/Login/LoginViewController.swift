import UIKit

/// `LoginViewController` is resopnsible for the process of logging in.
class LoginViewController : UIViewController, LoginServerDelegate , CAAnimationDelegate {

    /// The view that has the textfields to input the username and password and login button.
    var loginView: LoginView!
    
    /// The server which sends all the requests related to logging in.
    var loginServer: LoginServer!
    
    /// The function responsible for adding the action target to the login button
    /// and the textfields. Also responsible for adding and adjusting the
    /// login view to the screen.
    override func viewDidLoad() {
        super.viewDidLoad()
        loginView = LoginView()
        loginServer = LoginServer()
        self.view.addSubview(loginView)
        self.view.backgroundColor = .white
        
        loginServer.delegate = self
        loginView.loginButton.disableButton()
                
        let tapAnywhere = UITapGestureRecognizer(target: self, action: #selector(self.dismissKeyboard))
        self.view.addGestureRecognizer(tapAnywhere)
        
        // Adding action target to both text fields to track their editing status.
        loginView.usernameTextField.addTarget(self, action: #selector(self.textFieldEditChange), for: .editingChanged)
        loginView.passwordTextField.addTarget(self, action: #selector(self.textFieldEditChange), for: .editingChanged)
        
        let loginViewTopHeight = self.view.frame.size.height / 4
        loginView = self.loginView
            .addCenterXConstraint(relativeView: self.view)
            .addWidthConstraint(relativeView: self.view, multipler: 0.9)
            .addHeightConstraint(relativeView: self.view, multipler: 0.5)
            .setConstraintWithConstant(
                selfAttribute: .top,
                relativeView: self.view,
                relativeAttribute: .top,
                constant: loginViewTopHeight)
        
        // Add action target to the sign up label.
        let tap = UITapGestureRecognizer(target: self, action: #selector(self.signUpLabelTap(sender:)))
        loginView.signUpLabel.addGestureRecognizer(tap)
        loginView.signUpLabel.isUserInteractionEnabled = true
        
        // Add action target to the login view.
        loginView.loginButton.addTarget(
            self,
            action: #selector(self.loginInButtonClick),
            for: .touchUpInside)
        
    }
    
    /// Function to remove the keyboard if shown in the screen.
    @objc private func dismissKeyboard() {
        self.view.endEditing(true)
    }
    
    /// Function that sends the request to the `LoginServer` with the credentials to login.
    @objc private func loginInButtonClick() {
        loginServer.loginWithCredentials(
            username: loginView.usernameTextField.text!,
            password: loginView.passwordTextField.text!)
    }
    
    /// Function that transition to the `Register` screen.
    @objc private func signUpLabelTap(sender: UITapGestureRecognizer) {
        let transition = CATransition()
        transition.duration = 0.5
        transition.type = CATransitionType.fade
        transition.timingFunction = CAMediaTimingFunction(name: CAMediaTimingFunctionName.easeInEaseOut)
        self.view.window?.layer.add(transition, forKey: kCATransition)
        let registerViewController = RegisterViewController()
        self.present(registerViewController, animated: false, completion: nil)
    }
    
    /// Function called every time there is an edit change in one of the text fields.
    /// Responsible for disabling the login button in case one or both fields aren't filled.
    @objc private func textFieldEditChange() {
        if loginView.usernameTextField.hasText
            && loginView.passwordTextField.hasText {
            loginView.loginButton.enableButton()
        } else {
            loginView.loginButton.disableButton()
        }
    }
    
    /// Function called by the `LoginServer` when the login process was completed
    /// successfully. Responsible for navigating to the appropriate homescreen.
    func didLoginSuccessfully() {
        let pinitOwnerViewController = PinitOwnerViewController()
        self.present(pinitOwnerViewController, animated: true, completion: nil)
    }
    
    /// Function called by the `LoginServer` when the login process was completed
    /// with errors. Responsible for showing the error sent by the server with the
    /// appropraite `errorMessage`.
    func didLoginErrorOccur(errorMessage: String) {
        let alert = UIAlertController(
            title: "Error Occured",
            message: errorMessage,
            preferredStyle: UIAlertController.Style.alert)
        alert.addAction(UIAlertAction(title: "OK", style: UIAlertAction.Style.default, handler: nil))
        self.present(alert, animated: true, completion: nil)
    }

    /// Function responsible for updaing the views if needed when the main view appears.
    override func viewDidAppear(_ animated: Bool) {
        super.viewDidAppear(animated)
        loginView.updateView()
    }
}

extension CATransition {
    func fadeTransition() -> CATransition {
        let transition = CATransition()
        transition.duration = 0.4
        transition.type = CATransitionType.fade
        transition.subtype = CATransitionSubtype.fromRight
        
        return transition
    }
}
