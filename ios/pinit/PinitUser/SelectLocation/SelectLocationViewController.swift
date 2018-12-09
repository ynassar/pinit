import UIKit

/// Controller showing all the locations of the closest robot to the user, searching
/// through this list and choosing the desired one.
class SelectLocationViewController : PinitViewController , SelectLocationServerDelegate{
    
    /// Identifier for choosing the `SelectLocationTableViewCell` custom view.
    private let identifier = "id"
    
    /// Indentifier for choosing the `NoResultTableViewCell` custom view.
    private let noResultsIdentifier = "noResultsCell"
    
    /// THe height of a single cell in the `UITableView`.
    private let tableViewCellHeight: CGFloat = 66
    
    /// The table view in which the locations are shown and selected from.
    private var locationsTableView: UITableView!
    
    /// The search bar in which the user will type the location name they want
    /// to look for.
    private var searchBarView: SelectLocationSearchBarView!
    
    /// The server responsible for sending the gps coordinates of the server and
    /// returning a list of all locations of the closest robot.
    private var selectLocationServer: SelectLocationServer!
    
    /// The list being populated to the table view of the search controller and
    /// updated when the server returns the result or when the user narrows down
    /// the list using search.
    private var locationList: [Location]!
    
    /// The original list that has been returned by the `SelectLocationServer`.
    private var originalLocationList: [Location]!
    
    /// The delegate of the calling controller to return the location selected in
    /// the table view.
    public var searchResultDelegate: SelectLocationResultDelegate?
    
    /// Function responsible for initializing all the variables, adding the views to the
    /// controller view, specifying the delegates and adding the targets to the UI elements.
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
        locationsTableView.register(NoResultsTableViewCell.self, forCellReuseIdentifier: noResultsIdentifier)
        
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
    
    /// Function responsible for adding the contraints for the two views shown on the screen. One for
    /// the view which has the search bar and the back button, and the other is for the table view
    /// showing the locations.
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
    
    /// Function that returns back to the previous controller.
    @objc private func backButtonClick() {
        if let navigationController = self.navigationController {
            navigationController.delegate = self
            navigationController.popViewController(animated: true)
        }
    }
    
    /// Function that contains the search logic when the user types in the search bar. It is
    /// triggered every time the user edits the text field. If the search is empty then show
    /// all the locations, otherwise, filter the list using string compare functions and
    /// using lowecase to avoid having a case sensitvive search.
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
    
    /// Function called when the view will be apppearing, and in which the navigation bar
    /// will be hidden.
    override func viewWillAppear(_ animated: Bool) {
        self.navigationController?.setNavigationBarHidden(true, animated: animated)
        super.viewWillAppear(animated)
    }
    
    /// Function called when the view will be disappearing, and in which the navigation bar
    /// shoul be shown again.
    override func viewWillDisappear(_ animated: Bool) {
        self.navigationController?.setNavigationBarHidden(false, animated: animated)
        super.viewWillDisappear(animated)
    }
    
    /// Function called when the locations are retrieved by the `SelectLocationServer`.
    public func didUpdateLocations(locations: [Location]) {
        locationList = locations
        originalLocationList = locations
        locationsTableView.reloadData()
    }
}

/// Extenstion to adpot to the `UINavigationControllerDelegate` to call the custom
/// transitioning class when popping from/pushing into the navigation controller.
extension SelectLocationViewController : UINavigationControllerDelegate {
    
    /// Function that returns the `SlideDownAnimationTransitioning` instead of
    /// the default transitioning classes the navigation controller uses.
    func navigationController(
        _ navigationController: UINavigationController,
        animationControllerFor operation: UINavigationController.Operation,
        from fromVC: UIViewController,
        to toVC: UIViewController
        ) -> UIViewControllerAnimatedTransitioning? {
        return SlideDownAnimationTransitioning(operation: operation)
    }
}

/// Extenstion to adpot to the `UITableViewDelegate` that calls the functions responsible for
/// populating the table view and responding to events.
extension SelectLocationViewController: UITableViewDelegate, UITableViewDataSource {
    
    /// Function to return the number of rows in the table view. This will be number of locations.
    func tableView(_ tableView: UITableView, numberOfRowsInSection section: Int) -> Int {
        return (locationList.count > 0) ? locationList.count : 1
    }
    
    /// Function to show how the data will showin in each cell. It will assin the values
    /// to the variables of each cell which is a custom `SelectLocationTableViewCell`.
    func tableView(_ tableView: UITableView, cellForRowAt indexPath: IndexPath) -> UITableViewCell {
        if locationList.count > 0 {
            let cell = tableView.dequeueReusableCell(
                withIdentifier: self.identifier,
                for: indexPath) as! SelectLocationTableViewCell
            let values = locationList[indexPath.row]
            cell.locationName.text = values.name
            cell.locationDescription.text = (values.description == "")
                ? "No Description Available" : values.description
            return cell
        } else {
            let cell = tableView.dequeueReusableCell(
                withIdentifier: noResultsIdentifier,
                for: indexPath) as! NoResultsTableViewCell
            cell.noResultsLabel.text = "No Locations Available"
            return cell
        }
    }
    
    /// Function to return the height of every row.
    func tableView(_ tableView: UITableView, heightForRowAt indexPath: IndexPath) -> CGFloat {
        return self.tableViewCellHeight
    }
    
    /// Function called when any of the rows is being selected.
    func tableView(_ tableView: UITableView, didSelectRowAt indexPath: IndexPath) {
        searchResultDelegate?.getLocationSelected(location: locationList[indexPath.row])
        if let navigationController = navigationController {
            navigationController.delegate = self
            navigationController.popViewController(animated: true)
        }
    }
    
}

/// Extension responsible for responding the scroll view.
extension SelectLocationViewController: UIScrollViewDelegate {
    
    /// Function that hides the keyboard when the user starts scrolling in
    /// the list of location.
    public func scrollViewDidScroll(_ scrollView: UIScrollView) {
        view.endEditing(true)
    }
    
}
