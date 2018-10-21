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
    
    /// The `MappingRequest` being sent to the `MappingServer` which could be
    /// reused by all the functions called by the different arrow controls.
    var mappingDirectionRequest = MappingRequest()
    
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
        
        mappingDirectionRequest.robotName = "nemo"
        var serverToRosMappingRequest = ServerToRosMappingRequest()
        serverToRosMappingRequest.requestType = .direction
        mappingDirectionRequest.mappingRequest = serverToRosMappingRequest
        
        mappingView = self.mappingView
            .addCenterXConstraint()
            .setConstraintWithConstant(selfAttribute: .top, relativeView: self.navbar, relativeAttribute: .bottom, constant: 0)
            .addWidthConstraint(relativeView: self.view, multipler: 1.0)
            .setConstraintWithConstant(selfAttribute: .bottom, relativeView: mappingControlsView, relativeAttribute: .top, constant: 0)
        
        let spacing = self.view.frame.size.height * 0.05
        
        mappingControlsView = self.mappingControlsView
            .addCenterXConstraint()
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
        
        mappingView.startMappingButton.addTarget(
            self, action:  #selector(self.startMappingClick), for: .touchDown)
    }
    
    /// Function that sends a request to the `MapServer` to move the robot forward.
    @objc private func moveForwardButtonClick() {
        mappingDirectionRequest.mappingRequest.direction = .forward
        mappingServer.sendMovementRequest(mappingRequest: mappingDirectionRequest)
    }
    
    /// Function that sends a request to the `MapServer` to move the robot right.
    @objc private func moveRightButtonClick() {
        mappingDirectionRequest.mappingRequest.direction = .right
        mappingServer.sendMovementRequest(mappingRequest: mappingDirectionRequest)
    }
    
    /// Function that sends a request to the `MapServer` to move the robot left.
    @objc private func moveLeftButtonClick() {
        mappingDirectionRequest.mappingRequest.direction = .left
        mappingServer.sendMovementRequest(mappingRequest: mappingDirectionRequest)
    }
    
    /// Function that sends a request to the `MapServer` to move the robot backwards.
    @objc private func moveBackwardButtonClick() {
        mappingDirectionRequest.mappingRequest.direction = .backward
        mappingServer.sendMovementRequest(mappingRequest: mappingDirectionRequest)
    }
    
    /// Function that sends a request to the `MapServer` to stop the robot.
    @objc private func stopMovemenetClick() {
        mappingDirectionRequest.mappingRequest.direction = .stop
        mappingServer.sendMovementRequest(mappingRequest: mappingDirectionRequest)
    }
    
    /// Function that sends a request to the `MapServer` start mapping and eventually
    /// enabling the controls and start creating the map.
    @objc private func startMappingClick() {
        print("startMapping")
        var startMappingRequest = MappingRequest()
        startMappingRequest.robotName = "nemo"
        var serverToRosMappingRequest = ServerToRosMappingRequest()
        serverToRosMappingRequest.requestType = .startMapping
        startMappingRequest.mappingRequest = serverToRosMappingRequest
        mappingServer.startMappingRequest(mappingRequest: startMappingRequest)
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
    
    override func viewDidAppear(_ animated: Bool) {
        mappingControlsView.updateView()
        mappingView.updateView()
    }
    
    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
    }
    
}
