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
        
        getGpsCoordinatesButton.setTitle("Get GPS Coordinates", for: .normal)
        getGpsCoordinatesButton.backgroundColor = .white
        getGpsCoordinatesButton.setTitleColor(.black, for: .normal)
                
        getGpsCoordinatesButton = getGpsCoordinatesButton
            .addCenterXConstraint(relativeView: self)
            .addCenterYConstraint(relativeView: self)
            .addWidthConstraint(relativeView: self, multipler: 0.8)
            .addHeightConstraint(relativeView: self, multipler: 0.1)
    }
    
    
}
