import UIKit

class TabBarNavigationController : UIViewController {
    
    var navbar: UINavigationBar!
  
    override func viewDidLoad() {
        super.viewDidLoad()
        
        navbar = UINavigationBar(frame: CGRect(x: 0,
                                               y: UIApplication.shared.statusBarFrame.height,
                                               width : UIScreen.main.bounds.size.width,
                                               height : PinitConstants.navigationBarHeight));
        navbar.barTintColor = .white
        let navItem = UINavigationItem()
        navItem.title = "pinit"
        navbar.items = [navItem]
        
        self.view.addSubview(navbar)
        self.edgesForExtendedLayout = []

    }
}
