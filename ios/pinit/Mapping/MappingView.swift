import UIKit

/// 'MappingView' is responsible for having the start mapping button and
/// the imageview of the mapping image updated periodically.
class MappingView: UIView, UIScrollViewDelegate {
    
    var defaultMapImage: UIImageView!
    
    var scrollImage: UIScrollView!
    
    /// Initializer of the `MappingView`.
    init() {
        defaultMapImage = UIImageView(image: UIImage(named: "defaultMapImage"))
        scrollImage = UIScrollView(frame: CGRect.zero)
        super.init(frame: CGRect.zero)
        scrollImage.delegate = self
        self.addSubview(defaultMapImage)
        self.addSubview(scrollImage)
        
        scrollImage.minimumZoomScale = 1.0
        scrollImage.maximumZoomScale = 6.0
        scrollImage.addSubview(defaultMapImage)
    }
    
    /// Update the view by adding the constraints to make sure that the frame width
    /// is set. 
    public func updateView() {
        
        let spacing = self.frame.size.height * 0.02
        
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
        
        defaultMapImage = defaultMapImage
            .addCenterXConstraint(relativeView: scrollImage)
            .addCenterYConstraint(relativeView: scrollImage)
            .addWidthConstraint(relativeView: scrollImage, multipler: 1.0)
            .keepHeightAspectRatio()
    }
    
    public func disableMapView() {
        scrollImage.isScrollEnabled = false
    }
    
    public func enableMapView() {
        scrollImage.isScrollEnabled = true
    }
    
    func viewForZooming(in scrollView: UIScrollView) -> UIView? {
        return defaultMapImage
    }
    
    required init?(coder aDecoder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }
}
