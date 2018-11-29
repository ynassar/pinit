import UIKit

/// `PinitOwnervViewController` is the homepage of the pinit owner which constains all
/// the view controllers related to the owner and display them in the form of tabs.
class PinitOwnerViewController: UITabBarController, UITabBarControllerDelegate {
    
    override func viewDidLoad() {
        super.viewDidLoad()
        self.delegate = self
 
        self.tabBar.backgroundImage = UIImage()
        self.tabBar.barTintColor = .clear
        self.tabBar.shadowImage = UIImage()
        setUpTabBar()
    }
    
    private func setUpTabBar() {
        // The `MappingViewController` is used to start mapping, move the robot and save the map.
        let mappingViewController = UINavigationController(
            rootViewController: MappingViewController())
        mappingViewController.tabBarItem.image = UIImage(named: "mapIcon")
        mappingViewController.tabBarItem.selectedImage = UIImage(named: "mapIconSelected")
        mappingViewController.tabBarItem.imageInsets = UIEdgeInsets(top: 6, left: 0, bottom: -6, right: 0)

        
        let homepageViewController = UINavigationController(
            rootViewController: HomepageViewController())
        homepageViewController.tabBarItem.image = UIImage(named: "infoIcon")
        homepageViewController.tabBarItem.selectedImage = UIImage(named: "infoIconSelected")
        homepageViewController.tabBarItem.imageInsets = UIEdgeInsets(top: 6, left: 0, bottom: -6, right: 0)
        
        viewControllers = [homepageViewController, mappingViewController]
        
        
    }
}
