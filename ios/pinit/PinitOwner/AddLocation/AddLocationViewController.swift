import UIKit

class AddLocationViewController : UIViewController, AddLocationServerDelegate {
    
    var addLocationView : AddLocationView!
    
    var addLocationServer: AddLocationServer!
    
    override func viewDidLoad() {
        addLocationView = AddLocationView()
        addLocationServer = AddLocationServer()
        super.viewDidLoad()        
        self.view.backgroundColor = .white
        self.view.addSubview(addLocationView)
        
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
        let transition = CATransition()
        transition.duration = 0.2
        transition.type = CATransitionType.push
        transition.subtype = CATransitionSubtype.fromBottom
        transition.timingFunction = CAMediaTimingFunction(name: CAMediaTimingFunctionName.easeInEaseOut)
        if let navigationController = self.navigationController {
            navigationController.view.window?.layer.add(transition, forKey: kCATransition)
            navigationController.popViewController(animated: false)
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
    
    override func viewDidLayoutSubviews() {
        super.viewDidLayoutSubviews()
        addLocationView.updateView()
    }
}


