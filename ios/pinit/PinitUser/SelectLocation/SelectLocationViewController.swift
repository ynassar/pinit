import UIKit

class SelectLocationViewController : PinitViewController, UITableViewDelegate, UITableViewDataSource {
    
    private let identifier = "id"
    
    private let tableViewCellHeight: CGFloat = 66
    
    private var locationsTableView: UITableView!
    
    private var searchBarView: SelectLocationSearchBarView!
    
    override func viewDidLoad() {
        locationsTableView = UITableView()
        searchBarView = SelectLocationSearchBarView()
        self.controllerViews.append(searchBarView)
        super.viewDidLoad()
        
        self.view.addSubview(searchBarView)
        self.view.addSubview(locationsTableView)
        
        self.view.backgroundColor = .white
        
        let statusBar = UIApplication.shared.statusBarFrame.size.height
        
        searchBarView = searchBarView
            .addCenterXConstraint(relativeView: self.view)
            .addWidthConstraint(relativeView: self.view, multipler: 1.0)
            .addHeightConstraint(relativeView: self.view, multipler: 0.1)
            .setConstraintWithConstant(selfAttribute: .top,
                                       relativeView: self.view,
                                       relativeAttribute: .top,
                                       constant: statusBar)
        
        locationsTableView = locationsTableView
            .addCenterXConstraint(relativeView: self.view)
            .addWidthConstraint(relativeView: self.view, multipler: 1.0)
            .setConstraintWithConstant(selfAttribute: .top,
                                       relativeView: searchBarView,
                                       relativeAttribute: .bottom,
                                       constant: 0)
            .setConstraintWithConstant(selfAttribute: .bottom,
                                       relativeView: self.view,
                                       relativeAttribute: .bottom,
                                       constant: 0)
        
        locationsTableView.register(SelectLocationTableViewCell.self, forCellReuseIdentifier: identifier)
        
        locationsTableView.delegate = self
        locationsTableView.dataSource = self
        
        searchBarView.backButton.addTarget(
            self,
            action: #selector(self.backButtonClick),
            for: .touchDown)
    }
    
    @objc private func backButtonClick() {
        if let navigationController = self.navigationController {
            navigationController.delegate = self
            navigationController.popViewController(animated: true)
        }
    }
    
    func tableView(_ tableView: UITableView, numberOfRowsInSection section: Int) -> Int {
        return 20
    }
    
    func tableView(_ tableView: UITableView, cellForRowAt indexPath: IndexPath) -> UITableViewCell {
        return tableView.dequeueReusableCell(withIdentifier: self.identifier, for: indexPath)
    }
    
    func tableView(_ tableView: UITableView, heightForRowAt indexPath: IndexPath) -> CGFloat {
        return self.tableViewCellHeight
    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        self.navigationController?.setNavigationBarHidden(true, animated: animated)
    }
    
    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        self.navigationController?.setNavigationBarHidden(false, animated: animated)
    }
}

extension SelectLocationViewController : UINavigationControllerDelegate {
    func navigationController(
        _ navigationController: UINavigationController,
        animationControllerFor operation: UINavigationController.Operation,
        from fromVC: UIViewController,
        to toVC: UIViewController
        ) -> UIViewControllerAnimatedTransitioning? {
        return SlideDownAnimationTransitioning(operation: operation)
    }
}
