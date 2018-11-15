import UIKit

class SlideDownAnimationTransitioning : NSObject, UIViewControllerAnimatedTransitioning {
    
    let operation: UINavigationController.Operation
    
    init(operation: UINavigationController.Operation) {
        self.operation = operation
        super.init()
    }
    
    func transitionDuration(using transitionContext: UIViewControllerContextTransitioning?) -> TimeInterval {
        return 0.3
    }
    
    func animateTransition(using transitionContext: UIViewControllerContextTransitioning) {
        guard
            let fromViewController = transitionContext.viewController(forKey: .from),
            let toViewController = transitionContext.viewController(forKey: .to)
            else {
                return
        }
        
        let finalFrameForViewController = transitionContext.finalFrame(for: toViewController)
        let containerView = transitionContext.containerView
    
        if operation == .pop {
            
            toViewController.view.frame = finalFrameForViewController
            toViewController.view.alpha = 0.5
            containerView.insertSubview(toViewController.view, belowSubview: fromViewController.view)
            
            UIView.animate(withDuration: transitionDuration(using: transitionContext),
                           delay: 0,
                           options: UIView.AnimationOptions.curveEaseOut,
                           animations: {
                            fromViewController.view.frame = containerView.bounds.offsetBy(
                                dx: 0.0,
                                dy: toViewController.view.frame.size.height)
                            toViewController.view.alpha = 1.0
            },
                           completion: { (finished) in
                            transitionContext.completeTransition(true)
            })
        }
        
    }
}
