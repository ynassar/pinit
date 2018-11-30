import UIKit

class SelectLocationViewController : PinitViewController , SelectLocationServerDelegate{
    
    private let identifier = "id"
    
    private let tableViewCellHeight: CGFloat = 66
    
    private var locationsTableView: UITableView!
    
    private var searchBarView: SelectLocationSearchBarView!
    
    private var selectLocationServer: SelectLocationServer!
    
    private var locationList: [Location]!
    
    private var originalLocationList: [Location]!
    
    public var searchResultDelegate: SelectLocationResultDelegate?
        
    override func viewDidLoad() {
        locationsTableView = UITableView()
        searchBarView = SelectLocationSearchBarView()
        selectLocationServer = SelectLocationServer()
        locationList = []
        originalLocationList = []
        self.controllerViews.append(searchBarView)
        super.viewDidLoad()
        self.view.backgroundColor = .white

        self.view.addSubview(searchBarView)
        self.view.addSubview(locationsTableView)

        self.addViewsConstraints()
        
        locationsTableView.register(SelectLocationTableViewCell.self, forCellReuseIdentifier: identifier)
        
        locationsTableView.delegate = self
        locationsTableView.dataSource = self
        
        selectLocationServer.delegate = self
        selectLocationServer.getLocations()
        
        searchBarView.backButton.addTarget(
            self,
            action: #selector(self.backButtonClick),
            for: .touchDown)
        
        searchBarView.searchBar.addTarget(self, action: #selector(self.search), for: .editingChanged)
    }
    
    private func addViewsConstraints() {
        
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
        
    }
    
    @objc private func backButtonClick() {
        if let navigationController = self.navigationController {
            navigationController.delegate = self
            navigationController.popViewController(animated: true)
        }
    }
    
    @objc private func search() {
        if let searchText = searchBarView.searchBar.text {
            if searchText == "" {
                locationList = originalLocationList
            } else {
                let key = searchText.lowercased()
                locationList = originalLocationList.filter {
                    $0.name.lowercased().range(of: key) != nil
                }
            }
            locationsTableView.reloadData()
        }
    }
    
    override func viewWillAppear(_ animated: Bool) {
        self.navigationController?.setNavigationBarHidden(true, animated: animated)
        super.viewWillAppear(animated)
    }
    
    override func viewWillDisappear(_ animated: Bool) {
        self.navigationController?.setNavigationBarHidden(false, animated: animated)
        super.viewWillDisappear(animated)
    }
    
    public func didUpdateLocations(locations: [Location]) {
        locationList = locations
        originalLocationList = locations
        locationsTableView.reloadData()
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

extension SelectLocationViewController: UITableViewDelegate, UITableViewDataSource {
    
    func tableView(_ tableView: UITableView, numberOfRowsInSection section: Int) -> Int {
        return locationList.count
    }
    
    func tableView(_ tableView: UITableView, cellForRowAt indexPath: IndexPath) -> UITableViewCell {
        let cell = tableView.dequeueReusableCell(
            withIdentifier: self.identifier,
            for: indexPath) as! SelectLocationTableViewCell
        let values = locationList[indexPath.row]
        cell.locationName.text = values.name
        cell.locationDescription.text = (values.description == "")
            ? "No Description Available" : values.description
        return cell
    }
    
    func tableView(_ tableView: UITableView, heightForRowAt indexPath: IndexPath) -> CGFloat {
        return self.tableViewCellHeight
    }
    
    func tableView(_ tableView: UITableView, didSelectRowAt indexPath: IndexPath) {
        searchResultDelegate?.getLocationSelected(location: locationList[indexPath.row])
        if let navigationController = navigationController {
            navigationController.delegate = self
            navigationController.popViewController(animated: true)
        }
    }
    
}

extension SelectLocationViewController: UIScrollViewDelegate {
    
    public func scrollViewDidScroll(_ scrollView: UIScrollView) {
        view.endEditing(true)
    }
    
}
