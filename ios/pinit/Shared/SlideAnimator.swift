import UIKit

class SlideAnimationTransitioning : NSObject, UIViewControllerAnimatedTransitioning {
    
    let operation: UINavigationController.Operation
    
    init(operation: UINavigationController.Operation) {
        self.operation = operation
        super.init()
    }
    
    func transitionDuration(using transitionContext: UIViewControllerContextTransitioning?) -> TimeInterval {
        return 0.5
    }
    
    func animateTransition(using transitionContext: UIViewControllerContextTransitioning) {
        guard let fromViewController = transitionContext.viewController(
            forKey: UITransitionContextViewControllerKey.from),
            let toViewController = transitionContext.viewController(
                forKey: UITransitionContextViewControllerKey.to)
        else {
                return
        }
        
        let containerView = transitionContext.containerView
        
        switch operation {
        case .push:
            toViewController.view.frame = containerView.bounds.offsetBy(
                dx: containerView.frame.size.width,
                dy: 0.0)
            
            containerView.addSubview(toViewController.view)
            
            print("HEREEE")
            
            UIView.animate(withDuration: transitionDuration(using: transitionContext),
                           delay: 0,
                           options: UIView.AnimationOptions.curveEaseOut,
                           animations: {
                            toViewController.view.frame = containerView.bounds
                            fromViewController.view.frame = containerView.bounds.offsetBy(
                                dx: -containerView.frame.size.width,
                                dy: 0.0)
                            },
                           completion: { (finished) in
                            transitionContext.completeTransition(true)
                            })
        case .pop:
            break
        default:
            break
        }
        
    }
    
    
}
