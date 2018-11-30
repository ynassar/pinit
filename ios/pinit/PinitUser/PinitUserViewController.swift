import UIKit

fileprivate enum FieldSelected: Int {
    case PickUpLocation = 0
    case DestinationLocation = 1
    case None = 2
}

class PinitUserViewController: PinitSideMenuNavigationController {
    
    private var robotRequestView: RobotRequestView!
        
    private var fieldSelected: FieldSelected!
    
    override func viewDidLoad() {
        robotRequestView = RobotRequestView()
        fieldSelected = .None
        self.controllerViews.append(robotRequestView)
        super.viewDidLoad()

        self.view.addSubview(robotRequestView)
        self.view.backgroundColor = .white

        robotRequestView = robotRequestView
            .addCenterXConstraint(relativeView: self.view)
            .addCenterYConstraint(relativeView: self.view)
            .addWidthConstraint(relativeView: self.view, multipler: 1.0)
            .addHeightConstraint(relativeView: self.view, multipler: 0.5)

        robotRequestView.pickUpLocationTextFeild
            .addTarget(self,
                       action: #selector(self.showSearchController(sender:)),
                       for: .editingDidBegin)
        
        robotRequestView.destinationLocationTextFiled
            .addTarget(self,
                       action: #selector(self.showSearchController(sender:)),
                       for: .editingDidBegin)
        
        robotRequestView.pickUpLocationTextFeild.tag = FieldSelected.PickUpLocation.rawValue
        robotRequestView.destinationLocationTextFiled.tag = FieldSelected.DestinationLocation.rawValue
        
        robotRequestView.requestButton.disableButton()
    }
    
    @objc private func showSearchController(sender: UITextField) {
        
        if sender.tag == FieldSelected.PickUpLocation.rawValue {
            fieldSelected = .PickUpLocation
            robotRequestView.pickUpLocationTextFeild.endEditing(true)
        } else if sender.tag == FieldSelected.DestinationLocation.rawValue{
            fieldSelected = .DestinationLocation
            robotRequestView.destinationLocationTextFiled.endEditing(true)
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
            robotRequestView.destinationLocationTextFiled.text = location.name
            robotRequestView.destinationLocationTextFiled.markChecked()
        case .None:
            return
        }
        
        if robotRequestView.pickUpLocationTextFeild.hasText &&
            robotRequestView.destinationLocationTextFiled.hasText {
            
            robotRequestView.enableRequestButton()
        }
    }
    
}
