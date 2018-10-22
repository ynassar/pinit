import UIKit

/// `MappingViewController` is a tab controller responsible for mapping the
/// robot in its new environment through the app.
class MappingViewController: TabBarNavigationController,  MappingServerDelegate {
    
    /// The view that has the arrow controls to move the robot.
    var mappingControlsView: MappingControlsView!
    
    /// The view that has the map image which is regularly updated
    var mappingView: MappingView!
    
    /// The server which sends all the requests related to mapping.
    var mappingServer = MappingServer()
    
    var saveMappingButton: UIButton!
    
    /// The function is responsible for adding the targets to the different control buttons,
    /// one for button hold and the other for release. Also adding the different views in the
    /// view covering the entire screen.
    override func viewDidLoad() {
        mappingControlsView = MappingControlsView()
        mappingView = MappingView()
        super.viewDidLoad()
        self.view.backgroundColor = UIColor.white
        self.view.addSubview(mappingControlsView)
        self.view.addSubview(mappingView)
        
        mappingServer.delegate = self
        mappingControlsView.disableControls()
        mappingView.disableMapView()
        
        let startMappingButton = UIButton(frame: CGRect.zero)
        startMappingButton.setImage(UIImage(named: "startMapping"), for: .normal)
        startMappingButton.addTarget(self, action: #selector(self.startMappingClick), for: .touchDown)

        saveMappingButton = UIButton(frame: CGRect.zero)
        saveMappingButton.setImage(UIImage(named: "saveMapping"), for: .normal)
        saveMappingButton.addTarget(self, action: #selector(self.saveMappingClick), for: .touchDown)
        saveMappingButton.disableButton()
        

        self.navbar.topItem?.rightBarButtonItems = [
            UIBarButtonItem(customView: startMappingButton),
            UIBarButtonItem(customView: saveMappingButton)
        ]

        
        let spacing = self.view.frame.size.height * 0.02

        mappingView = self.mappingView
            .addCenterXConstraint(relativeView: self.view)
            .addWidthConstraint(relativeView: self.view, multipler: 1.0)
            .setConstraintWithConstant(selfAttribute: .top,
                                       relativeView: self.navbar,
                                       relativeAttribute: .bottom,
                                       constant: 0)
            .setConstraintWithConstant(selfAttribute: .bottom,
                                       relativeView: mappingControlsView,
                                       relativeAttribute: .top,
                                       constant: -spacing)
        
        mappingControlsView = self.mappingControlsView
            .addCenterXConstraint(relativeView: self.view)
            .addWidthConstraint(relativeView: self.view, multipler: 0.6)
            .addHeightConstraint(relativeView: self.view, multipler: 0.25)
            .setConstraintWithConstant(selfAttribute: .bottom,
                                       relativeView: self.view,
                                       relativeAttribute: .bottom,
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
    
    @objc public func saveMappingClick() {
        mappingServer.saveMappingRequest()
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
    
    func mapSaveConfirmation() {
        let alert = UIAlertController(
            title: "Map Saved",
            message: nil,
            preferredStyle: UIAlertController.Style.alert)
        alert.addAction(UIAlertAction(title: "OK", style: UIAlertAction.Style.default, handler: nil))
        self.present(alert, animated: true, completion: nil)
    }
    
    func mapStartConfirmation() {
        mappingControlsView.enableControls()
        saveMappingButton.enableButton()
    }
    
    override func viewDidAppear(_ animated: Bool) {
        mappingControlsView.updateView()
        mappingView.updateView()
    }
    
    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
    }
    
}