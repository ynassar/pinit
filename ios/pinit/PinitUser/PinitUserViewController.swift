import UIKit

class PinitUserViewController: UITabBarController, UITabBarControllerDelegate {
    
    override func viewDidLoad() {
        super.viewDidLoad()
        self.delegate = self
        
        self.tabBar.tintColor = .black
        self.tabBar.barTintColor = .white
        self.view.backgroundColor = .white
        setUpTabBar()
    }
    
    private func setUpTabBar() {
        let mappingViewController = UINavigationController(
            rootViewController: RobotRequestViewController())
        mappingViewController.tabBarItem.image = UIImage(named: "MapIcon")
        
        let settingsViewController = UINavigationController(
            rootViewController: ProfileViewController())
        settingsViewController.tabBarItem.image = UIImage(named: "SettingsIcon")
        
        viewControllers = [mappingViewController, settingsViewController]
        
    }
}
