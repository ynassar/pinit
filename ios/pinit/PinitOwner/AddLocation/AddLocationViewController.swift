import UIKit

class AddLocationViewController : UIViewController, AddLocationServerDelegate, UINavigationControllerDelegate {
    
    var addLocationView : AddLocationView!
    
    var addLocationServer: AddLocationServer!
    
    override func viewDidLoad() {
        addLocationView = AddLocationView()
        addLocationServer = AddLocationServer()
        super.viewDidLoad()        
        self.view.backgroundColor = .white
        self.view.addSubview(addLocationView)
        
        self.navigationController?.delegate = self
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
            action: #selector(self.closeButtonClick),
            for: .touchUpInside)
        
        addLocationView.locationName.becomeFirstResponder()
        
    }
    
    @objc private func closeButtonClick() {
        if let navigationController = self.navigationController {
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
    
    func didAddLocationSuccessfully() {
        self.closeButtonClick()
    }
    
    func didAddLocationErrorOccur(errorMessage: String) {
        print("Error Message")
    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        self.navigationController?.setNavigationBarHidden(true, animated: animated)
    }
    
    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        self.navigationController?.setNavigationBarHidden(false, animated: animated)
    }
    
    override func viewDidLayoutSubviews() {
        super.viewDidLayoutSubviews()
        addLocationView.updateView()
        print("bekhh")
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


