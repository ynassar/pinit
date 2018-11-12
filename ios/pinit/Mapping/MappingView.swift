import UIKit

/// 'MappingView' is responsible for having the start mapping button and
/// the imageview of the mapping image updated periodically.
class MappingView: UIView, UIScrollViewDelegate {
    
    var mapImage: UIImageView!
    
    var scrollImage: UIScrollView!
    
    /// Initializer of the `MappingView`.
    init() {
        mapImage = UIImageView(image: UIImage(named: "defaultMapImage"))
        scrollImage = UIScrollView(frame: CGRect.zero)
        super.init(frame: CGRect.zero)
        scrollImage.delegate = self
//        self.addSubview(mapImage)
        self.addSubview(scrollImage)
        
        scrollImage.minimumZoomScale = 1.0
        scrollImage.maximumZoomScale = 6.0
        scrollImage.addSubview(mapImage)
    }
    
    /// Update the view by adding the constraints to make sure that the frame width
    /// is set. 
    public func updateView() {
        
        let spacing = self.frame.size.height * 0.02
        
        print("Faridaaa", self.frame.size.height)
        
        scrollImage = scrollImage
            .addCenterXConstraint(relativeView: self)
            .addWidthConstraint(relativeView: self, multipler: 1.0)
            .setConstraintWithConstant(selfAttribute: .top,
                                       relativeView: self,
                                       relativeAttribute: .top,
                                       constant: spacing)
            .setConstraintWithConstant(selfAttribute: .bottom,
                                       relativeView: self,
                                       relativeAttribute: .bottom,
                                       constant: 0)
        
        mapImage = mapImage
            .addCenterXConstraint(relativeView: scrollImage)
            .addCenterYConstraint(relativeView: scrollImage)
            .addWidthConstraint(relativeView: scrollImage, multipler: 1.0)
            .addHeightConstraint(relativeView: scrollImage, multipler: 1.0)
    }
    
    /// Function to disable the `scrollImage` from the ability to zoom in and out
    /// using fingers gestures.
    public func disableMapView() {
        scrollImage.isScrollEnabled = false
    }
    
    /// Function to enable the `scrollImage` to zoom in and out using fingers gestures.
    public func enableMapView() {
        scrollImage.isScrollEnabled = true
    }
    
    required init?(coder aDecoder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }
}
