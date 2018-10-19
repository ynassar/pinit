import UIKit

class PinitOwnerViewController: UITabBarController, UITabBarControllerDelegate {
    
    var mappingViewController: MappingViewController!
    
    var settingsViewController: SettingsViewController!
    
    override func viewDidLoad() {
        super.viewDidLoad()
        mappingViewController = MappingViewController()
        settingsViewController = SettingsViewController()
        self.delegate = self
        
        self.tabBar.backgroundColor = .white
        
        let mappingBarItem = UITabBarItem(
            title: "Mapping",
            image: UIImage(named: "MapIcon"),
            selectedImage: nil)
        mappingViewController.tabBarItem = mappingBarItem
        
        let settingsBarItem = UITabBarItem(
            title: "Settings",
            image: UIImage(named: "SettingsIcon"),
            selectedImage: nil)
        settingsViewController.tabBarItem = settingsBarItem
        
        self.viewControllers = [mappingViewController, settingsViewController]
    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
    }
    
}
