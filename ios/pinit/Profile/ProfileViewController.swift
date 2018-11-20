import UIKit

class ProfileViewController: PinitNavigationController, UINavigationControllerDelegate,
UIViewControllerTransitioningDelegate {
    
    private var profileView: ProfileView!
    
    override func viewDidLoad() {
        profileView = ProfileView()
        super.viewDidLoad()
        self.controllerViews.append(profileView)
        self.view.backgroundColor = UIColor.white
        self.view.addSubview(profileView)
                
        profileView = profileView
            .addCenterXConstraint(relativeView: self.view)
            .addCenterYConstraint(relativeView: self.view)
            .addWidthConstraint(relativeView: self.view, multipler: 1.0)
            .addHeightConstraint(relativeView: self.view, multipler: 1.0)
        
        profileView.logoutButton.addTarget(
            self,
            action: #selector(self.logoutButtonClick),
            for: .touchUpInside)
    }
    
    @objc private func logoutButtonClick() {
        let domain = Bundle.main.bundleIdentifier!
        
        let ipAddress = UserDefaults.standard.string(forKey: SettingsBundleHelper.ipAddressIdentifier)
        UserDefaults.standard.removePersistentDomain(forName: domain)
        
        if let ip = ipAddress {
            SettingsBundleHelper.saveIpAddress(ipAddress: ip)
        }
        
        let loginViewController = LoginViewController()
        if let navigationController = self.navigationController {
            navigationController.delegate = self
            navigationController.viewControllers.insert(loginViewController, at: 0)
            self.tabBarController?.tabBar.isHidden = true
            navigationController.navigationBar.isHidden = true
            navigationController.popToRootViewController(animated: true)
        }
    }
    
    override func viewDidAppear(_ animated: Bool) {
        super.viewDidAppear(animated)
        profileView.logoutButton.addGradiant(colors: [PinitColors.yellow.cgColor,
                                                      PinitColors.red.cgColor,
                                                      PinitColors.blue.cgColor,
                                                      PinitColors.green.cgColor])
    }
    
    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
    }
    
    func navigationController(
        _ navigationController: UINavigationController,
        animationControllerFor operation: UINavigationController.Operation,
        from fromVC: UIViewController,
        to toVC: UIViewController
        ) -> UIViewControllerAnimatedTransitioning? {
        return SlideDownAnimationTransitioning(operation: operation)
    }
    
}
