import UIKit

class HomepageViewController : PinitSideMenuNavigationController {
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        super.view.backgroundColor = .white
    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        self.addGradiantBackground(color: PinitColors.yellow.cgColor)
    }
    
}
