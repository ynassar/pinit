import UIKit

fileprivate enum FieldSelected: Int {
    case PickUpLocation = 0
    case DestinationLocation = 1
    case None = 2
}

class PinitUserViewController: LogoutNavigationController {
    
    private var robotRequestView: RobotRequestView!
        
    private var fieldSelected: FieldSelected!
    
    private var robotRequestServer: RequestRobotServer!
    
    override func viewDidLoad() {
        robotRequestView = RobotRequestView()
        fieldSelected = .None
        robotRequestServer = RequestRobotServer()
        self.controllerViews.append(robotRequestView)
        super.viewDidLoad()

        self.view.addSubview(robotRequestView)
        self.view.backgroundColor = .white
        
        robotRequestServer.delegate = self

        robotRequestView = robotRequestView
            .addCenterXConstraint(relativeView: self.view)
            .addCenterYConstraint(relativeView: self.view)
            .addWidthConstraint(relativeView: self.view, multipler: 1.0)
            .addHeightConstraint(relativeView: self.view, multipler: 0.5)

        robotRequestView.pickUpLocationTextFeild
            .addTarget(self,
                       action: #selector(self.showSearchController(sender:)),
                       for: .editingDidBegin)
        
        robotRequestView.destinationLocationTextField
            .addTarget(self,
                       action: #selector(self.showSearchController(sender:)),
                       for: .editingDidBegin)
        
        robotRequestView.pickUpLocationTextFeild.tag = FieldSelected.PickUpLocation.rawValue
        robotRequestView.destinationLocationTextField.tag = FieldSelected.DestinationLocation.rawValue
        
        robotRequestView.requestButton.addTarget(
            self,
            action: #selector(self.requestRobot),
            for: .touchDown)
    }
    
    @objc private func showSearchController(sender: UITextField) {
        
        if sender.tag == FieldSelected.PickUpLocation.rawValue {
            fieldSelected = .PickUpLocation
            robotRequestView.pickUpLocationTextFeild.endEditing(true)
        } else if sender.tag == FieldSelected.DestinationLocation.rawValue{
            fieldSelected = .DestinationLocation
            robotRequestView.destinationLocationTextField.endEditing(true)
        } else {
            fieldSelected = .None
        }
        
        let selectLocationController = SelectLocationViewController()
        selectLocationController.searchResultDelegate = self
        if let navigationController = self.navigationController {
            navigationController.delegate = self
            navigationController.pushViewController(selectLocationController, animated: true)
        }
    }
    
    @objc private func requestRobot() {
        let robotRequest = RobotRequest(
            pickUp: robotRequestView.pickUpLocationTextFeild.text ?? "",
            destination: robotRequestView.destinationLocationTextField.text ?? "")
        robotRequestServer.sendRequest(robotRequest: robotRequest)
    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        self.addGradiantBackground(color: PinitColors.red.cgColor)
    }
    
    override func viewDidAppear(_ animated: Bool) {
        super.viewDidAppear(animated)
        robotRequestView.requestButton.makeButtonCircular()
    }
}

extension PinitUserViewController : UINavigationControllerDelegate {
    
    func navigationController(
        _ navigationController: UINavigationController,
        animationControllerFor operation: UINavigationController.Operation,
        from fromVC: UIViewController,
        to toVC: UIViewController
        ) -> UIViewControllerAnimatedTransitioning? {
        return SlideUpAnimationTransitioning(operation: operation)
    }
    
}

extension PinitUserViewController : SelectLocationResultDelegate {
    
    func getLocationSelected(location: Location) {
        switch fieldSelected! {
        case .PickUpLocation:
            robotRequestView.pickUpLocationTextFeild.text = location.name
            robotRequestView.pickUpLocationTextFeild.markChecked()
        case .DestinationLocation:
            robotRequestView.destinationLocationTextField.text = location.name
            robotRequestView.destinationLocationTextField.markChecked()
        case .None:
            return
        }
        
        if robotRequestView.pickUpLocationTextFeild.hasText &&
            robotRequestView.destinationLocationTextField.hasText {
            
            robotRequestView.enableRequestButton()
        }
    }
}

extension PinitUserViewController: RobotRequestServerDelegate {
    
    func robotRequested() {
        let tripStatusViewController = TripStatusViewController()
        tripStatusViewController.setLocationNames(
            pickUpLocation: robotRequestView.pickUpLocationTextFeild.text ?? "",
            destinationLocation: robotRequestView.destinationLocationTextField.text ?? "")
        if let navigationController = self.navigationController {
            navigationController.delegate = self
            navigationController.pushViewController(tripStatusViewController, animated: true)
        }
    }
    
}

extension PinitUserViewController: ResetViewControllerProtocol {
    
    func resetViewController() {
        robotRequestView.resetView()
    }
}
