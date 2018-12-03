import UIKit

class LogoutNavigationController : PinitNavigationController {
    
    override func viewDidLoad() {
        super.viewDidLoad()
 
        let profileMenuButton = UIButton(frame: CGRect.zero)
        profileMenuButton.setImage(UIImage(named: "logoutIcon"), for: .normal)
        profileMenuButton.addTarget(self, action: #selector(self.menuButtonClicked), for: .touchDown)

        self.navigationController?.navigationBar.topItem?.leftBarButtonItems = [
            UIBarButtonItem(customView: profileMenuButton)
        ]
    }
    
    @objc public func menuButtonClicked() {
        
        if let tabController = self.tabBarController {
            tabController.tabBar.isHidden = true
        }
        
        if let navigationController = self.navigationController {
            let domain = Bundle.main.bundleIdentifier!
            let ipAddress = UserDefaults.standard.string(forKey: SettingsBundleHelper.ipAddressIdentifier)
            UserDefaults.standard.removePersistentDomain(forName: domain)
            
            // To keep the ip address in the pinit settings to avoid adding it again when
            // logging out
            if let ip = ipAddress {
                SettingsBundleHelper.saveIpAddress(ipAddress: ip)
            }
            
            let loginViewController = LoginViewController()
            navigationController.navigationBar.isHidden = true
            navigationController.viewControllers.insert(loginViewController, at: 0)
            navigationController.popToRootViewController(animated: true)            
        }
    }
}

