import UIKit

/// `PinitOwnervViewController` is the homepage of the pinit owner which constains all
/// the view controllers related to the owner and display them in the form of tabs.
class PinitOwnerViewController: UITabBarController, UITabBarControllerDelegate {
    
    /// The `MappingViewController` is used to start mapping, move the robot and save the map.
//    var mappingViewController: MappingViewController!
    
    /// The `SettingsViewController` is a list of options related to the app
    /// and the user can change them.
//    var settingsViewController: SettingsViewController!
    
    /// Instantiate the view controllers, add their icons and set the display of the
    /// tab controller.
    override func viewDidLoad() {
        super.viewDidLoad()
//        mappingViewController = MappingViewController()
//        settingsViewController = SettingsViewController()
        self.delegate = self
        
        self.tabBar.tintColor = .black
        self.tabBar.barTintColor = .white
        self.tabBar.backgroundColor = .white
        
        setUpTabBar()
        
        // Tab for mapping screen responsible for creating the map, adding different locations,
        // moving the robot aroung during the process and finally saving the map.
//        let mappingBarItem = UITabBarItem(
//            title: nil,
//            image: UIImage(named: "MapIcon"),
//            selectedImage: nil)
//        mappingViewController.tabBarItem = mappingBarItem
//
//        // Tab for settings which has options like changing password, logging out and enabling
//        // or disabling notifications.
//        let settingsBarItem = UITabBarItem(
//            title: nil,
//            image: UIImage(named: "SettingsIcon"),
//            selectedImage: nil)
//        settingsViewController.tabBarItem = settingsBarItem
//        self.viewControllers = [mappingViewController, settingsViewController]

    }
    
    private func setUpTabBar() {
        // The `MappingViewController` is used to start mapping, move the robot and save the map.
        let mappingViewController = UINavigationController(
            rootViewController: MappingViewController())
        mappingViewController.tabBarItem.image = UIImage(named: "MapIcon")
        
        // The `SettingsViewController` is a list of options related to the app
        // and the user can change them.
        let settingsViewController = UINavigationController(
            rootViewController: SettingsViewController())
        settingsViewController.tabBarItem.image = UIImage(named: "SettingsIcon")
        
        viewControllers = [mappingViewController, settingsViewController]
        
    }
}
