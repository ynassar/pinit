import UIKit

class LoginViewController : UIViewController  {
    
    var loginView: LoginView!
    
    override func viewDidLoad() {
        super.viewDidLoad()
        loginView = LoginView()
        
        self.view.addSubview(loginView)
        self.view.backgroundColor = .white
                
        let tapAnywhere = UITapGestureRecognizer(target: self, action: #selector(self.dismissKeyboard))
        self.view.addGestureRecognizer(tapAnywhere)
        
        let loginViewTopHeight = self.view.frame.size.height / 4
        loginView = self.loginView
            .addCenterXConstraint()
            .addWidthConstraint(relativeView: self.view, multipler: 0.9)
            .addHeightConstraint(relativeView: self.view, multipler: 0.5)
            .addTopConstraint(
                relativeView: self.view,
                attribute: .top,
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
        let pinitOwnerViewController = PinitOwnerViewController()
        if let navigationController = self.navigationController {
            navigationController.popToRootViewController(animated: true)
            navigationController.pushViewController(pinitOwnerViewController, animated: true)
        }
    }
    
    @objc private func signUpLabelTap(sender: UITapGestureRecognizer) {
        let registerViewController = RegisterViewController()
        if let navigationController = self.navigationController {
            navigationController.pushViewController(registerViewController, animated: true)
        }
    }
    
    override func viewDidAppear(_ animated: Bool) {
        super.viewDidAppear(animated)
        loginView.updateView()
    }
    
}

