import UIKit

class SlideDownAnimationTransitioning : NSObject, UIViewControllerAnimatedTransitioning {
    
    let operation: UINavigationController.Operation
    
    init(operation: UINavigationController.Operation) {
        self.operation = operation
        super.init()
    }
    
    func transitionDuration(using transitionContext: UIViewControllerContextTransitioning?) -> TimeInterval {
        return 0.4
    }
    
    func animateTransition(using transitionContext: UIViewControllerContextTransitioning) {
        guard
            let fromViewController = transitionContext.viewController(forKey: .from),
            let toViewController = transitionContext.viewController(forKey: .to)
            else {
                return
        }
        
        let containerView = transitionContext.containerView
        if operation == .pop {
            
            containerView.insertSubview(toViewController.view, belowSubview: fromViewController.view)
            toViewController.view.bounds = containerView.bounds
            toViewController.viewDidLoad()
            
            print(toViewController.view.bounds)
            
            UIView.animate(withDuration: transitionDuration(using: transitionContext),
                           delay: 0,
                           options: UIView.AnimationOptions.curveEaseOut,
                           animations: {
                            fromViewController.view.frame = containerView.bounds.offsetBy(
                                dx: 0.0,
                                dy: containerView.frame.size.height)
            },
                           completion: { (finished) in
                            transitionContext.completeTransition(true)
            })
        }
        
    }
}
