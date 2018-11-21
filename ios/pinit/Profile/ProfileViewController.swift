import UIKit

class ProfileViewController: PinitViewController {
    
    private var profileView: ProfileView!
    
    override func viewDidLoad() {
        profileView = ProfileView()
        super.viewDidLoad()
        self.controllerViews.append(profileView)
        self.view.backgroundColor = UIColor.white
        self.view.addSubview(profileView)
        
        self.addShadow()
                
        profileView = profileView
            .addCenterXConstraint(relativeView: self.view)
            .addCenterYConstraint(relativeView: self.view)
            .addWidthConstraint(relativeView: self.view, multipler: 1.0)
            .addHeightConstraint(relativeView: self.view, multipler: 1.0)
        
        profileView.logoutButton.addTarget(
            self,
            action: #selector(self.logoutButtonClick),
            for: .touchDown)
        
        profileView.closeButton.addTarget(
            self,
            action: #selector(self.closeProfile),
            for: .touchDown)
    }
    
    private func addShadow() {
        self.view.layer.borderWidth = 1.0
        self.view.layer.borderColor = UIColor.gray.cgColor
    }
    
    @objc private func logoutButtonClick() {
        let domain = Bundle.main.bundleIdentifier!
        
        let ipAddress = UserDefaults.standard.string(forKey: SettingsBundleHelper.ipAddressIdentifier)
        UserDefaults.standard.removePersistentDomain(forName: domain)
        
        if let ip = ipAddress {
            SettingsBundleHelper.saveIpAddress(ipAddress: ip)
        }
        
        let loginViewController = LoginViewController()
        self.present(loginViewController, animated: true, completion: nil)
    }
    
    @objc private func closeProfile() {
        transitioningDelegate = self
        dismiss(animated: true, completion: nil)
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
}

extension ProfileViewController: UIViewControllerTransitioningDelegate {
    
    public func animationController(
        forDismissed dismissed: UIViewController
    ) -> UIViewControllerAnimatedTransitioning? {
        return SideMenuDismissAnimationTransitioning()
    }
    
}
