import UIKit

class SideMenuDismissAnimationTransitioning : NSObject, UIViewControllerAnimatedTransitioning {
  
    func transitionDuration(using transitionContext: UIViewControllerContextTransitioning?) -> TimeInterval {
        return 0.5
    }
    
    func animateTransition(using transitionContext: UIViewControllerContextTransitioning) {
        guard
            let fromViewController = transitionContext.viewController(forKey: .from),
            let toViewController = transitionContext.viewController(forKey: .to)
            else {
                return
        }
        
        let containerView = transitionContext.containerView
        
        containerView.insertSubview(toViewController.view, belowSubview: fromViewController.view)
        
        let widthOffset = fromViewController.view.bounds.width
                
        toViewController.view.frame = finalFrameForViewController.offsetBy(
            dx: widthOffset,
            dy: 0.0)
        toViewController.view.alpha = 1.0
        
        let snapshot = containerView.viewWithTag(PinitConstants.snapshotTag)
        snapshot?.removeFromSuperview()
        
        UIView.animate(withDuration: transitionDuration(using: transitionContext),
                       delay: 0,
                       options: UIView.AnimationOptions.curveEaseOut,
                       animations: {
                        toViewController.view.frame = finalFrameForViewController.offsetBy(
                            dx: 0.0,
                            dy: 0.0)
                        fromViewController.view.frame = fromViewController.view.bounds.offsetBy(
                            dx: -widthOffset,
                            dy: 0.0)
        },
                       completion: { (finished) in
                        
                        transitionContext.completeTransition(true)
        })
    }
}

