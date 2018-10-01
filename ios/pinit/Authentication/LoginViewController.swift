import UIKit

class LoginViewController : UIViewController {
    
    var headerView: UIView!
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        headerView = UIView()
        headerView.backgroundColor = .red
        self.view.addSubview(headerView)
        
    }
    
}

