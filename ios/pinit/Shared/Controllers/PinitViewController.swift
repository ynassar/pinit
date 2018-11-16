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
    
}
