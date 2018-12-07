import UIKit

class TripInfoView: UIView, PinitViewProtocol {
    
    private var fromLabel: UILabel
    
    public var fromLabelValue: UILabel
    
    private var toLabel: UILabel
    
    public var toLabelValue: UILabel
    
    private var statusLabel: UILabel
    
    private var statusLabelValue: UILabel
    
    private var progressbar: UIProgressView!
    
    init() {
        fromLabel = UILabel(frame: CGRect.zero)
        fromLabelValue = UILabel(frame: CGRect.zero)
        toLabel = UILabel(frame: CGRect.zero)
        toLabelValue = UILabel(frame: CGRect.zero)
        statusLabel = UILabel(frame: CGRect.zero)
        statusLabelValue = UILabel(frame: CGRect.zero)
        progressbar = UIProgressView(progressViewStyle: .bar)
        super.init(frame: CGRect.zero)
        
        self.addSubview(fromLabel)
        self.addSubview(fromLabelValue)
        self.addSubview(toLabel)
        self.addSubview(toLabelValue)
        self.addSubview(statusLabel)
        self.addSubview(statusLabelValue)
        self.addSubview(progressbar)
        
        progressbar.setProgress(0, animated: true)
    }
    
    required init?(coder aDecoder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }
    
    public func updateView() {
        
        setShadowBorder()
        
        progressbar.trackTintColor = PinitColors.gray
        progressbar.tintColor = PinitColors.red
        
        let leadingSpacing = self.frame.size.width * 0.2
        let topSpacing = self.frame.size.height * 0.3
        
        fromLabel = fromLabel.customizeTitleLabel(leadingSpacing: leadingSpacing, text: "From", view: self)
        toLabel = toLabel.customizeTitleLabel(leadingSpacing: leadingSpacing, text: "To", view: self)
        statusLabel = statusLabel.customizeTitleLabel(leadingSpacing: leadingSpacing, text: "Status", view: self)
        
        fromLabel = fromLabel
            .setConstraintWithConstant(selfAttribute: .top,
                                       relativeView: self,
                                       relativeAttribute: .top,
                                       constant: topSpacing)
        
        toLabel = toLabel
            .setConstraintWithConstant(selfAttribute: .top,
                                       relativeView: fromLabel,
                                       relativeAttribute: .bottom,
                                       constant: 0)
        
        statusLabel = statusLabel
            .setConstraintWithConstant(selfAttribute: .top,
                                       relativeView: toLabel,
                                       relativeAttribute: .bottom,
                                       constant: 0)
        
        let labelsSpacing = self.frame.size.width * 0.05
        fromLabelValue = fromLabelValue.customizeLabel(view: self)
        toLabelValue = toLabelValue.customizeLabel(view: self)
        statusLabelValue = statusLabelValue.customizeLabel(view: self)
        
        fromLabelValue = fromLabelValue
            .setConstraintWithConstant(selfAttribute: .top,
                                       relativeView: self,
                                       relativeAttribute: .top,
                                       constant: topSpacing)
            .setConstraintWithConstant(selfAttribute: .left,
                                       relativeView: fromLabel,
                                       relativeAttribute: .right,
                                       constant: labelsSpacing)
        
        toLabelValue = toLabelValue
            .setConstraintWithConstant(selfAttribute: .top,
                                       relativeView: fromLabelValue,
                                       relativeAttribute: .bottom,
                                       constant: 0)
            .setConstraintWithConstant(selfAttribute: .left,
                                       relativeView: toLabel,
                                       relativeAttribute: .right,
                                       constant: labelsSpacing)
        
        statusLabelValue = statusLabelValue
            .setConstraintWithConstant(selfAttribute: .top,
                                       relativeView: toLabel,
                                       relativeAttribute: .bottom,
                                       constant: 0)
            .setConstraintWithConstant(selfAttribute: .left,
                                       relativeView: statusLabel,
                                       relativeAttribute: .right,
                                       constant: labelsSpacing)
        
        
        let spacing = self.frame.size.height * 0.2

        progressbar = progressbar
            .addWidthConstraint(relativeView: self, multipler: 0.9)
            .addHeightConstraint(relativeView: self, multipler: 0.05)
            .addCenterXConstraint(relativeView: self)
            .setConstraintWithConstant(selfAttribute: .top,
                                       relativeView: statusLabel,
                                       relativeAttribute: .bottom,
                                       constant: spacing)

    }
    
    private func setShadowBorder() {
        
        self.layer.borderWidth = 0.5
        self.backgroundColor = .white
        self.layer.borderColor = PinitColors.borderGray.cgColor
        
        self.layer.masksToBounds = false
        self.layer.shadowRadius = 1.0
        self.layer.shadowColor = UIColor.black.cgColor
        self.layer.shadowOffset = CGSize(width: 1.0, height: 1.0)
        self.layer.shadowOpacity = 0.5
        
    }
    
    public func updateStatus(tripStatus: PinitTripStatus) {
        switch tripStatus {
        case .GoingToPickUp:
            statusLabelValue.text = "Going To Pickup"
            progressbar.setProgress(0.25, animated: true)
        case .WaitingForConfrimation:
            statusLabelValue.text = "Waiting For Confirmation"
            progressbar.setProgress(0.5, animated: true)
        case .GoingToDestination:
            statusLabelValue.text = "Going To Destination"
            progressbar.setProgress(0.75, animated: true)
        case .TripCompleted:
            statusLabelValue.text = "Trip Completed"
            progressbar.setProgress(1.0, animated: true)
        default:
            statusLabelValue.text = "Status Undefined"
        }
    }
    
}

fileprivate extension UILabel {
    
    fileprivate func customizeTitleLabel(leadingSpacing: CGFloat, text: String, view: UIView) -> UILabel{
        var adjustedLabel = self
        adjustedLabel.text = text
        adjustedLabel.font = UIFont(name: "Avenir-Heavy", size: 14)
        adjustedLabel = adjustedLabel
            .addWidthConstraint(relativeView: view, multipler: 0.2)
            .addHeightConstraint(relativeView: view, multipler: 0.1)
            .setConstraintWithConstant(selfAttribute: .left,
                                       relativeView: view,
                                       relativeAttribute: .left,
                                       constant: leadingSpacing)
        return adjustedLabel
    }
    
    fileprivate func customizeLabel(view: UIView) -> UILabel {
        var adjustedLabel = self
        adjustedLabel.font = UIFont(name: "Avenir", size: 12)
        adjustedLabel = adjustedLabel
            .addWidthConstraint(relativeView: view, multipler: 0.6)
            .addHeightConstraint(relativeView: view, multipler: 0.1)
        return adjustedLabel
    }
    
}
