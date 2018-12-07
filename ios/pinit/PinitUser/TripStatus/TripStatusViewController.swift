import UIKit

class TripStatusViewController: PinitViewController {
    
    private var tripStatusView: TripStatusView!
    
    private var tripInfoView: TripInfoView!
    
    private var tripStatusServer: TripStatusServer!
    
    override func viewDidLoad() {
        tripStatusView = TripStatusView()
        tripInfoView = TripInfoView()
        tripStatusServer = TripStatusServer()
        self.controllerViews.append(tripStatusView)
        self.controllerViews.append(tripInfoView)
        super.viewDidLoad()
        self.view.backgroundColor = .blue
        self.view.addSubview(tripStatusView)
        self.view.addSubview(tripInfoView)
        
        tripStatusView.confirmButton.disableButton()
        setupViews()
        
        tripStatusServer.delegate = self
        tripStatusServer.getTripStatus()
        
        tripStatusView.confirmButton.addTarget(
            self,
            action: #selector(self.confirmButtonClick),
            for: .touchDown)
        
        tripStatusView.homeButton.addTarget(
            self,
            action: #selector(self.homeButtonClick),
            for: .touchDown)
    }
    
    private func setupViews() {
        
        let statusHeight = UIApplication.shared.statusBarFrame.size.height
        let topSpacing = self.view.bounds.size.height * 0.2
        
        tripInfoView = tripInfoView
            .addCenterXConstraint(relativeView: self.view)
            .setConstraintWithConstant(selfAttribute: .top,
                                       relativeView: self.view,
                                       relativeAttribute: .top,
                                       constant: statusHeight + topSpacing)
            .addWidthConstraint(relativeView: self.view, multipler: 0.9)
            .addHeightConstraint(relativeView: self.view, multipler: 0.3)
        
        tripStatusView = tripStatusView
            .addCenterXConstraint(relativeView: self.view)
            .setConstraintWithConstant(selfAttribute: .top,
                                       relativeView: tripInfoView,
                                       relativeAttribute: .bottom,
                                       constant: 0)
            .addWidthConstraint(relativeView: self.view, multipler: 0.8)
            .addHeightConstraint(relativeView: self.view, multipler: 0.2)
        
    }
    
    @objc private func confirmButtonClick() {
        tripStatusServer.confrimTrip()
    }
    
    
    @objc private func homeButtonClick() {
        if let navigationController = self.navigationController {
            navigationController.delegate = self
            let previousController = navigationController.viewControllers.count - 2
            if let userController = navigationController.viewControllers[previousController]
                as? ResetViewControllerProtocol {
                userController.resetViewController()
            }
            navigationController.popViewController(animated: true)
        }
    }
    
    public func setLocationNames(pickUpLocation: String, destinationLocation: String) {
        tripInfoView.fromLabelValue.text = pickUpLocation
        tripInfoView.toLabelValue.text = destinationLocation
    }
    
    /// Function called when the view will be apppearing, and in which the navigation bar
    /// will be hidden.
    override func viewWillAppear(_ animated: Bool) {
        self.navigationController?.setNavigationBarHidden(true, animated: animated)
        super.viewWillAppear(animated)
        self.addGradiantBackground(color: PinitColors.red.cgColor)
    }
    
    /// Function called when the view will be disappearing, and in which the navigation bar
    /// shoul be shown again.
    override func viewWillDisappear(_ animated: Bool) {
        self.navigationController?.setNavigationBarHidden(false, animated: animated)
        super.viewWillDisappear(animated)
    }
}

extension TripStatusViewController : TripStatusServerDelegate {
    
    func updateTripStatus(status: PinitTripStatus) {
        tripInfoView.updateStatus(tripStatus: status)
        switch status {
        case .WaitingForConfrimation:
            tripStatusView.enableConfirmButton()
        case .GoingToDestination:
            tripStatusView.showHomeButton()
            tripStatusView.homeButton.disableButton()
        case .TripCompleted:
            tripStatusView.enableHomeButton()
        default:
            return
        }
    }

}

extension TripStatusViewController : UINavigationControllerDelegate {
    
    func navigationController(
        _ navigationController: UINavigationController,
        animationControllerFor operation: UINavigationController.Operation,
        from fromVC: UIViewController,
        to toVC: UIViewController
        ) -> UIViewControllerAnimatedTransitioning? {
        return SlideDownAnimationTransitioning(operation: operation)
    }
}
