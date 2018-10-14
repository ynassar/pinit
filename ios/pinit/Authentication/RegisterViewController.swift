import UIKit
import CgRPC

class RegisterViewController : UIViewController {

    var registerView: RegisterView!
    
    override func viewDidLoad() {
        super.viewDidLoad()
        registerView = RegisterView()
        self.view.addSubview(registerView)
        self.view.backgroundColor = .white
        
        let tapAnywhere = UITapGestureRecognizer(target: self, action: #selector(self.dismissKeyboard))
        self.view.addGestureRecognizer(tapAnywhere)
        
        let loginViewTopHeight = self.view.frame.size.height / 4
        
        registerView = self.registerView
            .addCenterXConstraint()
            .addWidthConstraint(relativeView: self.view, multipler: 0.9)
            .addHeightConstraint(relativeView: self.view, multipler: 0.4)
            .addTopConstraint(
                relativeView: self.view,
                attribute: .top,
                constant: loginViewTopHeight)
        
        let tap = UITapGestureRecognizer(target: self, action: #selector(self.signInLabelTap(sender:)))
        registerView.signInLabel.addGestureRecognizer(tap)
        registerView.signInLabel.isUserInteractionEnabled = true
                
    }
    
    @objc
    private func dismissKeyboard() {
        self.view.endEditing(true)
    }
    
    @objc
    func signInLabelTap(sender: UITapGestureRecognizer) {
        if let navigationController = self.navigationController {
            let loginViewController = navigationController.viewControllers[0]
            navigationController.popToViewController(loginViewController, animated: true)
        }
    }
    
    override func viewDidAppear(_ animated: Bool) {
        registerView.updateView()
    }
    
}

