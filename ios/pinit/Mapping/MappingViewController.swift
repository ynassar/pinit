import UIKit

/// `MappingViewController` is a tab controller responsible for mapping the
/// robot in its new environment through the app.
class MappingViewController: TabBarNavigationController, MappingServerDelegate, UINavigationControllerDelegate, UIViewControllerTransitioningDelegate {
    
    /// The view that has the arrow controls to move the robot.
    var mappingControlsView: MappingControlsView!
    
    /// The view that has the map image which is regularly updated
    var mappingView: MappingView!
    
    /// The server which sends all the requests related to mapping.
    var mappingServer = MappingServer()
    
    var saveMappingButtonItem: UIBarButtonItem!
    
    var addLocationButtonItem: UIBarButtonItem!
    
    var temp = CGRect.zero

    /// The function is responsible for adding the targets to the different control buttons,
    /// one for button hold and the other for release. Also adding the different views in the
    /// view covering the entire screen.
    override func viewDidLoad() {
        mappingControlsView = MappingControlsView()
        mappingView = MappingView()
        super.viewDidLoad()
        self.controllerViews.append(mappingControlsView)
        self.controllerViews.append(mappingView)
        self.view.backgroundColor = UIColor.white
        self.view.addSubview(mappingControlsView)
        self.view.addSubview(mappingView)
        
        mappingServer.delegate = self
        
        let startMappingButton = UIButton(frame: CGRect.zero)
        startMappingButton.setImage(UIImage(named: "startMapping"), for: .normal)
        startMappingButton.addTarget(self, action: #selector(self.startMappingClick), for: .touchDown)

        let saveMappingButton = UIButton(frame: CGRect.zero)
        saveMappingButton.setImage(UIImage(named: "saveMapping"), for: .normal)
        saveMappingButton.addTarget(self, action: #selector(self.saveMappingClick), for: .touchDown)
        self.saveMappingButtonItem = UIBarButtonItem(customView: saveMappingButton)
    
        let addLocationButton = UIButton(frame: CGRect.zero)
        addLocationButton.setImage(UIImage(named: "plusIcon"), for: .normal)
        addLocationButton.addTarget(self, action: #selector(self.addLocationClick), for: .touchDown)
        self.addLocationButtonItem = UIBarButtonItem(customView: addLocationButton)
        
        self.navigationController?.navigationBar.topItem?.rightBarButtonItems = [
            UIBarButtonItem(customView: startMappingButton),
            self.addLocationButtonItem
        ]
        
        self.navigationController?.navigationBar.topItem?.leftBarButtonItems = [
            self.saveMappingButtonItem
        ]
        
        mappingControlsView.disableControls()
        mappingView.disableMapView()
        saveMappingButtonItem.disableButton()
        //addLocationButtonItem.disableButton()
        
        let spacing = self.view.frame.size.height * 0.02
        
        mappingControlsView = self.mappingControlsView
            .addCenterXConstraint(relativeView: self.view)
            .addWidthConstraint(relativeView: self.view, multipler: 0.6)
            .addHeightConstraint(relativeView: self.view, multipler: 0.3)
            .setConstraintWithConstant(selfAttribute: .bottom,
                                       relativeView: self.view,
                                       relativeAttribute: .bottom,
                                       constant: -spacing)
        
        mappingView = self.mappingView
            .addCenterXConstraint(relativeView: self.view)
            .addWidthConstraint(relativeView: self.view, multipler: 1.0)
            .setConstraintWithConstant(selfAttribute: .top,
                                       relativeView: self.view,
                                       relativeAttribute: .top,
                                       constant: 0)
            .setConstraintWithConstant(selfAttribute: .bottom,
                                       relativeView: mappingControlsView,
                                       relativeAttribute: .top,
                                       constant: -spacing)

        
        mappingControlsView.moveForwardButton.addTarget(
            self, action:  #selector(self.moveForwardButtonClick), for: .touchDown)
        
        mappingControlsView.moveRightButton.addTarget(
            self, action:  #selector(self.moveRightButtonClick), for: .touchDown)
        
        mappingControlsView.moveLeftButton.addTarget(
            self, action:  #selector(self.moveLeftButtonClick), for: .touchDown)
        
        mappingControlsView.moveBackwardButton.addTarget(
            self, action:  #selector(self.moveBackwardButtonClick), for: .touchDown)
        
        mappingControlsView.moveForwardButton.addTarget(
            self, action:  #selector(self.stopMovemenetClick), for: .touchUpInside)
        
        mappingControlsView.moveRightButton.addTarget(
            self, action:  #selector(self.stopMovemenetClick), for: .touchUpInside)
        
        mappingControlsView.moveLeftButton.addTarget(
            self, action:  #selector(self.stopMovemenetClick), for: .touchUpInside)
        
        mappingControlsView.moveBackwardButton.addTarget(
            self, action:  #selector(self.stopMovemenetClick), for: .touchUpInside)
    }
    
    /// Function that sends a request to the `MapServer` to move the robot forward.
    @objc private func moveForwardButtonClick() {
        mappingServer.moveRobotForward()
    }
    
    /// Function that sends a request to the `MapServer` to move the robot right.
    @objc private func moveRightButtonClick() {
        mappingServer.moveRobotRight()
    }
    
    /// Function that sends a request to the `MapServer` to move the robot left.
    @objc private func moveLeftButtonClick() {
        mappingServer.moveRobotLeft()
    }
    
    /// Function that sends a request to the `MapServer` to move the robot backwards.
    @objc private func moveBackwardButtonClick() {
        mappingServer.moveRobotBackward()
    }
    
    /// Function that sends a request to the `MapServer` to stop the robot.
    @objc private func stopMovemenetClick() {
        mappingServer.moveRobotStop()
    }
    
    /// Function that sends a request to the `MapServer` start mapping and eventually
    /// enabling the controls and start creating the map.
    @objc private func startMappingClick() {
        mappingServer.startMappingRequest()
    }
    
    /// Function that sends a request to the `MapServer` to save the map.
    @objc public func saveMappingClick() {
        mappingServer.saveMappingRequest()
    }
    
    /// Function that navigates to the `AddLocation` screen.
    @objc public func addLocationClick() {
        let addLocationViewController = AddLocationViewController()
        if let navigationController = self.navigationController {
            navigationController.delegate = self
            navigationController.pushViewController(addLocationViewController, animated: true)
        }
    }
    
    /// Function responsible for handling any error that comes from the `MapServer` after
    /// sending a request.
    func didMappingErrorOccur(_ errorMessage: String) {
        let alert = UIAlertController(
            title: "Error Occured",
            message: errorMessage,
            preferredStyle: UIAlertController.Style.alert)
        alert.addAction(UIAlertAction(title: "OK", style: UIAlertAction.Style.default, handler: nil))
        self.present(alert, animated: true, completion: nil)
        
    }
    
    /// Function responsible for showing a confitmation alert that the map has been saved.
    func mapSaveConfirmation() {
        let alert = UIAlertController(
            title: "Map Saved",
            message: nil,
            preferredStyle: UIAlertController.Style.alert)
        alert.addAction(UIAlertAction(title: "OK", style: UIAlertAction.Style.default, handler: nil))
        self.present(alert, animated: true, completion: nil)
        mappingControlsView.disableControls()
        saveMappingButtonItem.disableButton()
        //addLocationButtonItem.disableButton()
    }
    
    /// Function responsible for enable the different UI elements including the controls
    /// buttons and the map image when the a `MappingRequest` for starting mapping has been sent.
    func mapStartConfirmation() {
        mappingControlsView.enableControls()
        saveMappingButtonItem.enableButton()
        addLocationButtonItem.enableButton()
        mappingView.enableMapView()
        mappingServer.mapImageRequestAsynchronous()
    }
    
    /// Function called when the map image requested periodically has been recieved and a
    /// assigns the image to the `mapImage` in the `mappingView`.
    func mapImageUpdate(newImage: UIImage) {
        mappingView.mapImage.image = newImage
    }
    
    override func viewDidAppear(_ animated: Bool) {
        super.viewDidAppear(animated)
        self.tabBarController?.tabBar.isHidden = false
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
        return SlideUpAnimationTransitioning(operation: operation)
    }
}

