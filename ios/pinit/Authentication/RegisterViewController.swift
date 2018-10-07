import UIKit

class RegisterViewController : UIViewController {
    
    var registerView: RegisterView!
    
    override func viewDidLoad() {
        super.viewDidLoad()
        registerView = RegisterView()
        self.view.addSubview(registerView)
        self.view.backgroundColor = .white
        
        let loginViewTopHeight = self.view.frame.size.height / 4
        
        registerView = self.registerView
            .addCenterXConstraint()
            .addWidthConstraint(relativeView: self.view, multipler: 0.9)
            .addHeightConstraint(relativeView: self.view, multipler: 0.5)
            .addTopConstraint(
                relativeView: self.view,
                attribute: .top,
                constant: loginViewTopHeight)
    }
    
    override func viewDidAppear(_ animated: Bool) {
        registerView.updateView()
    }
    
}

