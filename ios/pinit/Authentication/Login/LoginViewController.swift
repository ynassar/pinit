import UIKit

class LoginViewController : UIViewController, LoginServerDelegate  {

    var loginView: LoginView!
    
    var loginServer: LoginServer!
    
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
        
        loginView.usernameTextField.addTarget(self, action: #selector(self.textFieldEditChange), for: .editingChanged)
        loginView.passwordTextFiled.addTarget(self, action: #selector(self.textFieldEditChange), for: .editingChanged)
        
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
        
        let tap = UITapGestureRecognizer(target: self, action: #selector(self.signUpLabelTap(sender:)))
        loginView.signUpLabel.addGestureRecognizer(tap)
        loginView.signUpLabel.isUserInteractionEnabled = true
        
        loginView.loginButton.addTarget(
            self,
            action: #selector(self.loginInButtonClick),
            for: .touchUpInside)
        
    }
    
    @objc private func dismissKeyboard() {
        self.view.endEditing(true)
    }
    
    @objc private func loginInButtonClick() {
        loginServer.loginWithCredentials(
            username: loginView.usernameTextField.text!,
            password: loginView.passwordTextFiled.text!)
    }
    
    @objc private func signUpLabelTap(sender: UITapGestureRecognizer) {
        let registerViewController = RegisterViewController()
        if let navigationController = self.navigationController {
            navigationController.pushViewController(registerViewController, animated: true)
        }
    }
    
    @objc private func textFieldEditChange() {
        if loginView.usernameTextField.hasText
            && loginView.passwordTextFiled.hasText {
            loginView.loginButton.enableButton()
        } else {
            loginView.loginButton.disableButton()
        }
    }
    
    func didLoginSuccessfully() {
        let pinitOwnerViewController = PinitOwnerViewController()
        if let navigationController = self.navigationController {
            navigationController.popToRootViewController(animated: true)
            navigationController.pushViewController(pinitOwnerViewController, animated: true)
        }
    }
    
    func didLoginErrorOccur(errorMessage: String) {
        let alert = UIAlertController(
            title: "Error Occured",
            message: errorMessage,
            preferredStyle: UIAlertController.Style.alert)
        alert.addAction(UIAlertAction(title: "OK", style: UIAlertAction.Style.default, handler: nil))
        self.present(alert, animated: true, completion: nil)
    }
    
    override func viewDidAppear(_ animated: Bool) {
        super.viewDidAppear(animated)
        loginView.updateView()
    }
}
