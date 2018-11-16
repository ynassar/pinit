import UIKit
import CgRPC

/// `RegitserViewController` is responsible for the process of registering.
class RegisterViewController : UIViewController, RegisterServerDelegate {

    /// The view that has the textfields of the user info and the register button.
    var registerView: RegisterView!
    
    /// The server which sends all the requests related to registering.
    var registerServer = RegisterServer()
    
    /// The function responsible for adding the action target to the register button
    /// and the the textfields. Also responsible for adding and  adjusting the register
    /// view to the screen.
    override func viewDidLoad() {
        super.viewDidLoad()
        registerView = RegisterView()
        self.view.addSubview(registerView)
        self.view.backgroundColor = .white
        
        registerView.registerButton.disableButton()
        
        registerServer.delegate = self
        
        // Adding action target to all textfields to track their editing status.
        registerView.usernameTextField.addTarget(self, action: #selector(self.textFieldEditChange), for: .editingChanged)
        registerView.passwordTextField.addTarget(self, action: #selector(self.textFieldEditChange), for: .editingChanged)
        registerView.confrimPasswordTextField.addTarget(self, action: #selector(self.textFieldEditChange), for: .editingChanged)
        registerView.emailTextField.addTarget(self, action: #selector(self.textFieldEditChange), for: .editingChanged)
        
        let tapAnywhere = UITapGestureRecognizer(target: self, action: #selector(self.dismissKeyboard))
        self.view.addGestureRecognizer(tapAnywhere)
        
        let topHeight = self.view.frame.size.height / 4
        
        registerView = self.registerView
            .addCenterXConstraint(relativeView: self.view)
            .addWidthConstraint(relativeView: self.view, multipler: 0.9)
            .addHeightConstraint(relativeView: self.view, multipler: 0.5)
            .setConstraintWithConstant(
                selfAttribute: .top,
                relativeView: self.view,
                relativeAttribute: .top,
                constant: topHeight)
        
        // Add action target to the login label.
        let tap = UITapGestureRecognizer(target: self, action: #selector(self.signInLabelTap(sender:)))
        registerView.signInLabel.addGestureRecognizer(tap)
        registerView.signInLabel.isUserInteractionEnabled = true
        
        // Add action target to the register button.
        registerView.registerButton.addTarget(
            self,
            action: #selector(self.regiserButtonClick),
            for: .touchUpInside)
    }
    
    /// Function to remove the keyboard if shown in the screen.
    @objc private func dismissKeyboard() {
        self.view.endEditing(true)
    }
    
    /// Function that sends the request to the `RegisterView` with the information to register.
    @objc private func regiserButtonClick() {
        if (registerView.passwordTextField.text! !=
            registerView.confrimPasswordTextField.text!) {
            self.showAlertMessage(
                title: "Error Message",
                message: "PLease type the same passwords twice.")
        } else {
            registerServer.registerWithCredentials(
                username: registerView.usernameTextField.text!,
                password: registerView.passwordTextField.text!,
                email:  registerView.emailTextField.text!)
        }
    }
    
    /// Function called every time there is an edit change in one of the text fields.
    /// Responsible for disabling the register button in case any of the fields aren't filled.
    @objc private func textFieldEditChange() {
        if registerView.usernameTextField.hasText
            && registerView.passwordTextField.hasText
            && registerView.confrimPasswordTextField.hasText
            && registerView.emailTextField.hasText {
            registerView.registerButton.enableButton()
        } else {
            registerView.registerButton.disableButton()
        }
    }
    
    /// Function called by the `RegisterServer` when the register process was completed
    /// successfully. Responsible for navigating back to the `LoginViewController`.
    func didRegisterSuccessfully() {
        if let navigationController = self.navigationController {
            let loginViewController = navigationController.viewControllers[0]
            navigationController.popToViewController(loginViewController, animated: true)
        }
    }
    
    /// Function called by the `RegisterServer` when the register process was completed
    /// with errors. Responsible for showing the error sent by the server with the
    /// appropraite `errorMessage`.
    func didRegisterErrorOccur(errorMessage: String) {
        self.showAlertMessage(
            title: "Error Occured",
            message: errorMessage)
    }
    
    /// Function that transition to the `Login` screen.
    @objc func signInLabelTap(sender: UITapGestureRecognizer) {
        let transition = CATransition()
        transition.duration = 0.5
        transition.type = CATransitionType.fade
        transition.timingFunction = CAMediaTimingFunction(name: CAMediaTimingFunctionName.easeInEaseOut)
        self.view.window?.layer.add(transition, forKey: kCATransition)
        self.dismiss(animated: false, completion: nil)
    }
    
    /// Function responsible for updaing the views if needed when the main view appears.
    override func viewDidAppear(_ animated: Bool) {
        super.viewDidAppear(animated)
        registerView.updateView()
    }
    
    override func viewDidLayoutSubviews() {
        super.viewDidLayoutSubviews()
        registerView.updateView()
    }
    
    /// Function that shows an alert with and `OK` button. 
    private func showAlertMessage(title: String, message: String) {
        let alert = UIAlertController(
            title: title,
            message: message,
            preferredStyle: UIAlertController.Style.alert)
        alert.addAction(UIAlertAction(title: "OK", style: UIAlertAction.Style.default, handler: nil))
        self.present(alert, animated: true, completion: nil)
    }
}
