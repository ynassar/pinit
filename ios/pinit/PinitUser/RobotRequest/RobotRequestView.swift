import UIKit

class RobotRequestView : UIView, PinitViewProtocol {

    public var getGpsCoordinatesButton: UIButton
    
    init() {
        getGpsCoordinatesButton = UIButton(frame: CGRect.zero)
        super.init(frame: CGRect.zero)
        
        self.addSubview(getGpsCoordinatesButton)
    }
    
    required init?(coder aDecoder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }
    
    func updateView() {
        
        
        getGpsCoordinatesButton.setTitle("Request Robot", for: .normal)
        getGpsCoordinatesButton.backgroundColor = PinitColors.gray
        getGpsCoordinatesButton.setTitleColor(.white, for: .normal)
        getGpsCoordinatesButton.titleLabel?.font = UIFont(name: "Avenir", size: 14.0)
                        
        getGpsCoordinatesButton = getGpsCoordinatesButton
            .addCenterXConstraint(relativeView: self)
            .addCenterYConstraint(relativeView: self)
            .addWidthConstraint(relativeView: self, multipler: 0.5)
            .setEqualConstraint(selfAttribute: .height,
                                relativeView: getGpsCoordinatesButton,
                                relativeAttribute: .width)
    }
    
    
}
