import UIKit

/// `PinitOwnervViewController` is the homepage of the pinit owner which constains all
/// the view controllers related to the owner and display them in the form of tabs.
class PinitOwnerViewController: UITabBarController, UITabBarControllerDelegate {
    
    override func viewDidLoad() {
        super.viewDidLoad()
        self.delegate = self
        
        self.tabBar.tintColor = .black
        self.tabBar.barTintColor = .white
//        self.tabBar.layer.borderWidth = 0
//        self.tabBar.clipsToBounds = true
//        self.view.backgroundColor = .white
        
//        let profileMenuButton = UIButton(frame: CGRect.zero)
//        profileMenuButton.setImage(UIImage(named: "menuIcon"), for: .normal)
//
//        self.navigationController?.navigationBar.topItem?.leftBarButtonItems = [
//            UIBarButtonItem(customView: profileMenuButton)
//        ]

        setUpTabBar()
    }
    
    private func setUpTabBar() {
        // The `MappingViewController` is used to start mapping, move the robot and save the map.
        let mappingViewController = UINavigationController(
            rootViewController: MappingViewController())
        mappingViewController.tabBarItem.image = UIImage(named: "MapIcon")
        
        // The `SettingsViewController` is a list of options related to the app
        // and the user can change them.
        let settingsViewController = UINavigationController(
            rootViewController: ProfileViewController())
        settingsViewController.tabBarItem.image = UIImage(named: "SettingsIcon")
        
        viewControllers = [mappingViewController, settingsViewController]
        
        
    }
}
