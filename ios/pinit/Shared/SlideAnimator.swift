import UIKit

class SlideAnimator: NSObject, UIViewControllerAnimatedTransitioning {
    var duration: TimeInterval = 1.0
    
    func transitionDuration(using transitionContext: UIViewControllerContextTransitioning?) -> TimeInterval {
        return duration
    }
    
    func animateTransition(using transitionContext: UIViewControllerContextTransitioning) {
        let containerView = transitionContext.containerView
        let toView = transitionContext.view(forKey: .to)!
        
        containerView.addSubview(toView)
        toView.frame = CGRect(x: 0, y: containerView.bounds.height, width: containerView.bounds.width, height: containerView.bounds.height)
        
        let timingFunction = CAMediaTimingFunction(controlPoints: 0/6, 0.8, 3/6, 1.0)
        CATransaction.begin()
        CATransaction.setAnimationTimingFunction(timingFunction)
        UIView.animate(withDuration: duration, animations: {
            toView.frame = containerView.frame
        }, completion: { _ in
            transitionContext.completeTransition(true)
        } )
        CATransaction.commit()
    }
}
