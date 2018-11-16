import UIKit

class AddLocationViewController :
PinitViewController,
AddLocationServerDelegate,
UINavigationControllerDelegate,
UIViewControllerTransitioningDelegate
{
    
    private var addLocationView : AddLocationView!
    
    private var addLocationServer: AddLocationServer!
    
    override func viewDidLoad() {
        addLocationView = AddLocationView()
        addLocationServer = AddLocationServer()
        super.viewDidLoad()
        self.controllerViews.append(addLocationView)
        self.view.backgroundColor = .white
        self.view.addSubview(addLocationView)
        
        addLocationServer.delegate = self
        
        addLocationView.doneButton.disableButton()
        
        addLocationView = self.addLocationView
            .addWidthConstraint(relativeView: self.view, multipler: 1.0)
            .addHeightConstraint(relativeView: self.view, multipler: 1.0)
            .addCenterXConstraint(relativeView: self.view)
            .addCenterYConstraint(relativeView: self.view)
        
        addLocationView.locationName.addTarget(self, action: #selector(self.textFieldEditChange), for: .editingChanged)
        
        addLocationView.closeButton.addTarget(
            self,
            action: #selector(self.closeButtonClick),
            for: .touchUpInside)
        
        addLocationView.doneButton.addTarget(
            self,
            action: #selector(self.doneButtonClick),
            for: .touchUpInside)
        
        addLocationView.locationName.becomeFirstResponder()
    }
    
    @objc private func closeButtonClick() {
        if let navigationController = self.navigationController {
            navigationController.delegate = self
            navigationController.popViewController(animated: true)
        }
    }
    
    @objc private func doneButtonClick() {
        addLocationServer.addLocation(
            locationName: addLocationView.locationName.text ?? "",
            locationDescription: addLocationView.locationDescription.text ?? "")
    }
    
    @objc private func textFieldEditChange() {
        if addLocationView.locationName.hasText {
            addLocationView.doneButton.enableButton()
        } else {
            addLocationView.doneButton.disableButton()
        }
    }
    
    public func didAddLocationSuccessfully() {
        self.closeButtonClick()
    }
    
    public func didAddLocationErrorOccur(errorMessage: String) {
        print("Error Message")
    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        self.navigationController?.setNavigationBarHidden(true, animated: animated)
        self.tabBarController?.tabBar.isHidden = true
    }
    
    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        self.navigationController?.setNavigationBarHidden(false, animated: animated)
    }
    
    
    func navigationController(
        _ navigationController: UINavigationController,
        animationControllerFor operation: UINavigationController.Operation,
        from fromVC: UIViewController,
        to toVC: UIViewController
        ) -> UIViewControllerAnimatedTransitioning? {
        return SlideDownAnimationTransitioning(operation: operation)
    }
}


