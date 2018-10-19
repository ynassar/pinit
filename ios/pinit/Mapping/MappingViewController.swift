import UIKit

class MappingViewController: UIViewController {
    
    var mappingControlsView: MappingControlsView!
    
    var mappingServer = MappingServer()
    
    override func viewDidLoad() {
        mappingControlsView = MappingControlsView()
        super.viewDidLoad()
        self.view.backgroundColor = UIColor.white
        self.view.addSubview(mappingControlsView)
        
        self.edgesForExtendedLayout = .top
        
        let spacing = self.view.frame.size.height * 0.01
        
        mappingControlsView = self.mappingControlsView
            .addCenterXConstraint()
            .addWidthConstraint(relativeView: self.view, multipler: 0.6)
            .addHeightConstraint(relativeView: self.view, multipler: 0.25)
            .addBottomConstraint(
                relativeView: self.view,
                attribute: .bottom,
                constant: spacing)
        
        mappingControlsView.moveForwardButton.addTarget(
            self,
            action: #selector(self.moveForwardButtonClick),
            for: .touchUpInside)
        
        mappingControlsView.moveRightButton.addTarget(
            self,
            action: #selector(self.moveRightButtonClick),
            for: .touchUpInside)
        
        mappingControlsView.moveLeftButton.addTarget(
            self,
            action: #selector(self.moveLeftButtonClick),
            for: .touchUpInside)
        
        mappingControlsView.moveBackwardButton.addTarget(
            self,
            action: #selector(self.moveBackwardsButtonClick),
            for: .touchUpInside)
    }
    
    @objc private func moveForwardButtonClick() {
        print("Forward")
        var mappingRequest = MappingRequest()
        mappingRequest.robotName = "nemo"
        
        var serverToRosMappingRequest = ServerToRosMappingRequest()
        serverToRosMappingRequest.direction = .forward
        serverToRosMappingRequest.requestType = .direction
    
        mappingRequest.mappingRequest = serverToRosMappingRequest
        mappingServer.sendMovementRequest(mappingRequest: mappingRequest)
    }
    
    @objc private func moveRightButtonClick() {
        print("Right")
        
        var mappingRequest = MappingRequest()
        mappingRequest.robotName = "nemo"
        
        var serverToRosMappingRequest = ServerToRosMappingRequest()
        serverToRosMappingRequest.direction = .right
        serverToRosMappingRequest.requestType = .direction
        
        mappingRequest.mappingRequest = serverToRosMappingRequest
        mappingServer.sendMovementRequest(mappingRequest: mappingRequest)
    }
    
    @objc private func moveLeftButtonClick() {
        print("Left")
        
        var mappingRequest = MappingRequest()
        mappingRequest.robotName = "nemo"
        
        var serverToRosMappingRequest = ServerToRosMappingRequest()
        serverToRosMappingRequest.direction = .left
        serverToRosMappingRequest.requestType = .direction
        
        mappingRequest.mappingRequest = serverToRosMappingRequest
        mappingServer.sendMovementRequest(mappingRequest: mappingRequest)
    }
    
    @objc private func moveBackwardsButtonClick() {
        print("Backward")
        
        var mappingRequest = MappingRequest()
        mappingRequest.robotName = "nemo"
        
        var serverToRosMappingRequest = ServerToRosMappingRequest()
        serverToRosMappingRequest.direction = .backward
        serverToRosMappingRequest.requestType = .direction
        
        mappingRequest.mappingRequest = serverToRosMappingRequest
        mappingServer.sendMovementRequest(mappingRequest: mappingRequest)
    }
    
    override func viewDidAppear(_ animated: Bool) {
        mappingControlsView.updateView()
    }
    
    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
    }
    
}
