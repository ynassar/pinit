import UIKit

class LoginViewController : UIViewController {
    
    var loginView: LoginView!
    
    override func viewDidLoad() {
        super.viewDidLoad()
        loginView = LoginView()
        self.view.addSubview(loginView)
        self.view.backgroundColor = .white
        
        let loginViewTopHeight = self.view.frame.size.height / 4
        
        loginView = self.loginView
            .addCenterXConstraint()
            .addWidthConstraint(relativeView: self.view, multipler: 0.9)
            .addHeightConstraint(relativeView: self.view, multipler: 0.6)
            .addTopConstraint(
                relativeView: self.view,
                attribute: .top,
                constant: loginViewTopHeight)
    }
    
    override func viewDidAppear(_ animated: Bool) {
        loginView.updateView()
    }
    
}

