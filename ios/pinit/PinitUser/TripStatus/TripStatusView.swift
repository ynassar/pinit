import UIKit

class TripStatusView: UIView, PinitViewProtocol {
    
    public var confirmButton: UIButton!
    
    public var homeButton: UIButton!
    
    init() {
        confirmButton = UIButton()
        homeButton = UIButton()
        super.init(frame: CGRect.zero)
        
        self.addSubview(confirmButton)
        self.addSubview(homeButton)
        confirmButton.setImage(UIImage(named: "confirmButtonIcon"), for: .normal)
        homeButton.setImage(UIImage(named: "homeButtonIcon"), for: .normal)
        
        homeButton.isHidden = true
    }
    
    required init?(coder aDecoder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }
    
    func updateView() {
        confirmButton = confirmButton
            .addCenterXConstraint(relativeView: self)
            .addCenterYConstraint(relativeView: self)
            .addWidthConstraint(relativeView: self, multipler: 0.3)
            .setEqualConstraint(selfAttribute: .height,
                                relativeView: confirmButton,
                                relativeAttribute: .width)
        
        homeButton = homeButton
            .addCenterXConstraint(relativeView: self)
            .addCenterYConstraint(relativeView: self)
            .addWidthConstraint(relativeView: self, multipler: 0.3)
            .setEqualConstraint(selfAttribute: .height,
                                relativeView: confirmButton,
                                relativeAttribute: .width)
        
    }
    
    public func enableConfirmButton() {
        confirmButton.setImage(UIImage(named: "confirmButtonColoredIcon"), for: .normal)
        confirmButton.enableButton()
    }
    
    public func enableHomeButton() {
        homeButton.setImage(UIImage(named: "homeButtonColoredIcon"), for: .normal)
        homeButton.enableButton()
    }
    
    public func showHomeButton() {
        confirmButton.isHidden = true
        homeButton.isHidden = false
    }
    
}
