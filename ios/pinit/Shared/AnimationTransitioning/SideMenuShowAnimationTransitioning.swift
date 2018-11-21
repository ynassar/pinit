import UIKit

class SideMenuShowAnimationTransitioning : NSObject, UIViewControllerAnimatedTransitioning {
    
    let sideMenuWidthMultiplier: CGFloat
    
    init(sideMenuWidthMultiplier: CGFloat) {
        self.sideMenuWidthMultiplier = sideMenuWidthMultiplier
        super.init()
    }
    
    func transitionDuration(using transitionContext: UIViewControllerContextTransitioning?) -> TimeInterval {
        return 0.5
    }
    
    func animateTransition(using transitionContext: UIViewControllerContextTransitioning) {
        guard
            let fromViewController = transitionContext.viewController(forKey: .from),
            let toViewController = transitionContext.viewController(forKey: .to),
            let snapshot = fromViewController.view.snapshotView(afterScreenUpdates: false)
            else {
                return
        }
        
        var finalFrameForViewController = transitionContext.finalFrame(for: toViewController)
        let containerView = transitionContext.containerView
        
        finalFrameForViewController = CGRect(
            x: 0,
            y: 0,
            width: finalFrameForViewController.width * sideMenuWidthMultiplier,
            height: finalFrameForViewController.height)
        
        let widthOffset = finalFrameForViewController.width
        
        toViewController.view.frame = finalFrameForViewController.offsetBy(
            dx: -widthOffset,
            dy: 0.0)
        
        
        containerView.insertSubview(toViewController.view, belowSubview: fromViewController.view)

        snapshot.tag = PinitConstants.snapshotTag
        containerView.insertSubview(snapshot, aboveSubview: toViewController.view)
        fromViewController.view.isHidden = true
        
        UIView.animate(withDuration: transitionDuration(using: transitionContext),
                       delay: 0,
                       options: UIView.AnimationOptions.curveEaseOut,
                       animations: {
                        toViewController.view.frame = finalFrameForViewController.offsetBy(
                            dx: 0.0,
                            dy: 0.0)
                        snapshot.frame = snapshot.frame.offsetBy(
                            dx: widthOffset,
                            dy: 0.0)
                        },
                       completion: { (finished) in
                        fromViewController.view.isHidden = false
                        transitionContext.completeTransition(true)
        })
    }
}

