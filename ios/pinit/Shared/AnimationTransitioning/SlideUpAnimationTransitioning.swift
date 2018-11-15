import UIKit

class SlideUpAnimationTransitioning : NSObject, UIViewControllerAnimatedTransitioning {
    
    let operation: UINavigationController.Operation
    
    init(operation: UINavigationController.Operation) {
        self.operation = operation
        super.init()
    }
    
    func transitionDuration(using transitionContext: UIViewControllerContextTransitioning?) -> TimeInterval {
        return 0.3
    }
    
    func animateTransition(using transitionContext: UIViewControllerContextTransitioning) {
        print("animation slide up started")
        guard
            let toViewController = transitionContext.viewController(forKey: .to),
            let fromViewController = transitionContext.viewController(forKey: .from)
        else {
                return
        }
        
        let finalFrameForViewController = transitionContext.finalFrame(for: toViewController)
        let containerView = transitionContext.containerView
        let bounds = UIScreen.main.bounds
        
        print("slide up", fromViewController.view.bounds)
        print("final frame", finalFrameForViewController)

        
        if operation == .push {
            toViewController.view.frame = finalFrameForViewController.offsetBy(
                dx: 0.0,
                dy: bounds.size.height)
            
            containerView.addSubview(toViewController.view)
            
            UIView.animate(withDuration: transitionDuration(using: transitionContext),
                           delay: 0,
                           options: UIView.AnimationOptions.curveEaseOut,
                           animations: {
                            toViewController.view.frame = finalFrameForViewController
                            fromViewController.view.alpha = 0.5
                            },
                           completion: { (finished) in
                            transitionContext.completeTransition(true)
                            fromViewController.view.alpha = 1.0
                            })
        }
        
    }
}
