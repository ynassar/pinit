import UIKit
import CgRPC

class RegisterViewController : UIViewController, RegisterServerDelegate {

    var registerView: RegisterView!
    
    var registerServer = RegisterServer()
    
    override func viewDidLoad() {
        super.viewDidLoad()
        registerView = RegisterView()
        self.view.addSubview(registerView)
        self.view.backgroundColor = .white
        
        registerView.registerButton.disableButton()
        
        registerServer.delegate = self
        
        registerView.usernameTextField.addTarget(self, action: #selector(self.textFieldEditChange), for: .editingChanged)
        registerView.passwordTextFiled.addTarget(self, action: #selector(self.textFieldEditChange), for: .editingChanged)
        registerView.confrimPasswordTextFiled.addTarget(self, action: #selector(self.textFieldEditChange), for: .editingChanged)
        registerView.emailTextField.addTarget(self, action: #selector(self.textFieldEditChange), for: .editingChanged)
        
        let tapAnywhere = UITapGestureRecognizer(target: self, action: #selector(self.dismissKeyboard))
        self.view.addGestureRecognizer(tapAnywhere)
        
        let topHeight = self.view.frame.size.height / 4
        
        registerView = self.registerView
            .addCenterXConstraint(relativeView: self.view)
            .addWidthConstraint(relativeView: self.view, multipler: 0.9)
            .addHeightConstraint(relativeView: self.view, multipler: 0.4)
            .setConstraintWithConstant(
                selfAttribute: .top,
                relativeView: self.view,
                relativeAttribute: .top,
                constant: topHeight)
        
        
        
        let tap = UITapGestureRecognizer(target: self, action: #selector(self.signInLabelTap(sender:)))
        registerView.signInLabel.addGestureRecognizer(tap)
        registerView.signInLabel.isUserInteractionEnabled = true
        
        registerView.registerButton.addTarget(
            self,
            action: #selector(self.regiserButtonClick),
            for: .touchUpInside)
    }
    
    @objc private func regiserButtonClick() {
        if (registerView.passwordTextFiled.text! !=
            registerView.confrimPasswordTextFiled.text!) {
            self.showAlertMessage(
                title: "Error Message",
                message: "PLease type the same passwords twice.")
        } else {
            registerServer.registerWithCredentials(
                username: registerView.usernameTextField.text!,
                password: registerView.passwordTextFiled.text!,
                email:  registerView.emailTextField.text!)
        }
    }
    
    @objc private func textFieldEditChange() {
        if registerView.usernameTextField.hasText
            && registerView.passwordTextFiled.hasText
            && registerView.confrimPasswordTextFiled.hasText
            && registerView.emailTextField.hasText {
            registerView.registerButton.enableButton()
        } else {
            registerView.registerButton.disableButton()
        }
    }
    
    func didRegisterSuccessfully() {
        if let navigationController = self.navigationController {
            let loginViewController = navigationController.viewControllers[0]
            navigationController.popToViewController(loginViewController, animated: true)
        }
    }
    
    func didRegisterErrorOccur(errorMessage: String) {
        let alert = UIAlertController(
            title: "Error Occured",
            message: errorMessage,
            preferredStyle: UIAlertController.Style.alert)
        alert.addAction(UIAlertAction(title: "OK", style: UIAlertAction.Style.default, handler: nil))
        self.present(alert, animated: true, completion: nil)
    }
    
    @objc private func dismissKeyboard() {
        self.view.endEditing(true)
    }
    
    @objc func signInLabelTap(sender: UITapGestureRecognizer) {
        if let navigationController = self.navigationController {
            let loginViewController = navigationController.viewControllers[0]
            navigationController.popToViewController(loginViewController, animated: true)
        }
    }
    
    override func viewDidAppear(_ animated: Bool) {
        registerView.updateView()
    }
    
    private func showAlertMessage(title: String, message: String) {
        let alert = UIAlertController(
            title: title,
            message: message,
            preferredStyle: UIAlertController.Style.alert)
        alert.addAction(UIAlertAction(title: "OK", style: UIAlertAction.Style.default, handler: nil))
        self.present(alert, animated: true, completion: nil)
    }
}
