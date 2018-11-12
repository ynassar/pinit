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
        guard let toViewController = transitionContext.viewController(forKey: .to)
        else {
                return
        }
        
        let containerView = transitionContext.containerView
        if operation == .push {
            toViewController.view.frame = containerView.bounds.offsetBy(
                dx: 0.0,
                dy: containerView.frame.size.height)
                        
            containerView.addSubview(toViewController.view)
            
            UIView.animate(withDuration: transitionDuration(using: transitionContext),
                           delay: 0,
                           options: UIView.AnimationOptions.curveEaseOut,
                           animations: {
                            toViewController.view.frame = containerView.bounds.offsetBy(
                                dx: 0.0,
                                dy: 0.0)
                            },
                           completion: { (finished) in
                            transitionContext.completeTransition(true)
                            })
        }
        
    }
}
