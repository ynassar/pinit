import UIKit

class PinitViewController : UIViewController {
    
    internal var controllerViews: [PinitViewProtocol] = []
    
    override func viewDidLoad() {
        super.viewDidLoad()
    }
    
    override func viewDidLayoutSubviews() {
        super.viewDidLayoutSubviews()
        
        for controllerView in controllerViews {
            controllerView.updateView()
        }
    }
    
    public func addGradiantBackground(color: CGColor) {
        let gradiantLayer = CAGradientLayer()
        gradiantLayer.frame = self.view.bounds
        gradiantLayer.startPoint = CGPoint(x: 0, y: 0.5)
        gradiantLayer.endPoint = CGPoint(x: 0, y: 1)
        gradiantLayer.colors = [UIColor.white.cgColor,
                                color]
        self.view.layer.insertSublayer(gradiantLayer, at: 0)
    }
}
