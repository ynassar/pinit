import UIKit

class PinitSideMenuNavigationController : PinitNavigationController {
    
    override func viewDidLoad() {
        super.viewDidLoad()
 
        let profileMenuButton = UIButton(frame: CGRect.zero)
        profileMenuButton.setImage(UIImage(named: "menuIcon"), for: .normal)
        profileMenuButton.addTarget(self, action: #selector(self.menuButtonClicked), for: .touchDown)

        self.navigationController?.navigationBar.topItem?.leftBarButtonItems = [
            UIBarButtonItem(customView: profileMenuButton)
        ]
    }
    
    @objc public func menuButtonClicked() {
        let profileViewController = ProfileViewController()
        profileViewController.loggingOutDelegate = self
        profileViewController.transitioningDelegate = self
        self.present(profileViewController, animated: true, completion: nil)
    }
}

extension PinitSideMenuNavigationController: LoggingOutDelegate {
    
    func userLoggedOut() {
        let loginViewController = LoginViewController()
        if let navigationController = self.navigationController {
            navigationController.viewControllers.insert(loginViewController, at: 0)
            navigationController.popToRootViewController(animated: true)
            self.navigationController?.navigationBar.isHidden = true
            self.tabBarController?.tabBar.isHidden = true
        }
    }    
}

extension PinitSideMenuNavigationController: UIViewControllerTransitioningDelegate {
    
    public func animationController(
        forPresented presented: UIViewController,
        presenting: UIViewController,
        source: UIViewController
        ) -> UIViewControllerAnimatedTransitioning? {
        return SideMenuShowAnimationTransitioning(sideMenuWidthMultiplier: 0.7)
    }
    
}
