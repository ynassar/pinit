import UIKit

/// `TabBarNavigationController` is a parent view controller to any controller that requires a
/// navigation bar.
class TabBarNavigationController : UIViewController {
    
    /// The navigation bar added on the top of the view controller.
    var navbar: UINavigationBar!
  
    /// Function to add the navigation bar with the apporpriate height along as adjusting its
    /// title and button colors. =
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
