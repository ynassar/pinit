import UIKit

class TabBarNavigationController : PinitViewController {

    override func viewDidLoad() {
        super.viewDidLoad()
        self.navigationController?.navigationBar.barTintColor = .white
        self.navigationController?.navigationBar.backgroundColor = .white
        self.navigationController?.navigationBar.topItem?.title = "pinit"
        self.edgesForExtendedLayout = []
    }

}

