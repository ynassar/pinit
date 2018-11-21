import UIKit

class ProfileViewController: PinitViewController {
    
    private var profileView: ProfileView!
    
    private var profileInfoView: ProfileInfoView!
    
    override func viewDidLoad() {
        profileView = ProfileView()
        profileInfoView = ProfileInfoView()
        super.viewDidLoad()
        self.controllerViews.append(profileView)
        self.controllerViews.append(profileInfoView)
        self.view.backgroundColor = UIColor.white
        self.view.addSubview(profileView)
        self.view.addSubview(profileInfoView)
        
        self.addShadow()
        
        profileInfoView = profileInfoView
            .addCenterXConstraint(relativeView: self.view)
            .setEqualConstraint(selfAttribute: .top, relativeView: self.view, relativeAttribute: .top)
            .addWidthConstraint(relativeView: self.view, multipler: 1.0)
            .addHeightConstraint(relativeView: self.view, multipler: 0.2)
                
        profileView = profileView
            .addCenterXConstraint(relativeView: self.view)
            .setEqualConstraint(selfAttribute: .top, relativeView: profileInfoView, relativeAttribute: .top)
            .addWidthConstraint(relativeView: self.view, multipler: 1.0)
            .addHeightConstraint(relativeView: self.view, multipler: 0.8)
        
        profileView.logoutButton.addTarget(
            self,
            action: #selector(self.logoutButtonClick),
            for: .touchDown)
        
        profileInfoView.closeButton.addTarget(
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
        profileInfoView.addGradiant()
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
