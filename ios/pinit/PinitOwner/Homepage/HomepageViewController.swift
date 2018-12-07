import UIKit

class HomepageViewController : LogoutNavigationController {
    
    private var tableView: UITableView!
    
    private let dailySectionTableViewCellHeight: CGFloat = 66
    
    private let frequentSectiontodayTableViewCellHeight: CGFloat = 44
    
    private let inBetweenSectionSpacing: CGFloat = 66
    
    private let dailyCellIdentifier =  "dailyTripCell"
    
    private let frequentCellIdentifier = "freqLocationCell"
    
    private let noResultsIdentifier = "noResultsCell"
    
    private var homepageServer: HomepageServer!
    
    private var dailyTrips: [TripInfo]!
    
    private var frequentLocations: [LocationInfo]!
    
    override func viewDidLoad() {
        tableView = UITableView(frame: CGRect.zero, style: .grouped)
        homepageServer = HomepageServer()
        dailyTrips = []
        frequentLocations = []
        super.viewDidLoad()
        
        self.view.addSubview(tableView)
        
        homepageServer.delegate = self
        
        homepageServer.getTodayTrips()
        homepageServer.getFrequentocations()
        
        tableView.register(TodayTripsTableViewCell.self, forCellReuseIdentifier: dailyCellIdentifier)
        tableView.register(NoResultsTableViewCell.self, forCellReuseIdentifier: noResultsIdentifier)
        tableView.register(FrequentLocationsTableViewCell.self, forCellReuseIdentifier: frequentCellIdentifier)
        
        tableView.tableFooterView = UIView(frame: CGRect(x: 0.0, y: 0.0, width: 0.0, height: 0.01))
        tableView.tableHeaderView = UIView(frame: CGRect(x: 0.0, y: 0.0, width: 0.0, height: 0.01))
        tableView.contentInsetAdjustmentBehavior = .never
        
        tableView.delegate = self
        tableView.dataSource = self
        tableView.backgroundColor = .clear
        
        let refreshButton = UIButton(frame: CGRect.zero)
        refreshButton.setImage(UIImage(named: "refreshIcon"), for: .normal)
        refreshButton.addTarget(self, action: #selector(self.refreshData), for: .touchDown)
        
        self.navigationController?.navigationBar.topItem?.rightBarButtonItems = [
            UIBarButtonItem(customView: refreshButton),
        ]
        
        self.addViewsConstraints()
    }
    
    private func addViewsConstraints() {
        
        let tabBarHeight = self.tabBarController?.tabBar.bounds.height ?? 0.0
                
        tableView = tableView
            .addCenterXConstraint(relativeView: self.view)
            .setConstraintWithConstant(selfAttribute: .top,
                                       relativeView: self.view,
                                       relativeAttribute: .top,
                                       constant: 0)
            .addWidthConstraint(relativeView: self.view, multipler: 1.0)
            .setConstraintWithConstant(selfAttribute: .bottom,
                                       relativeView: self.view,
                                       relativeAttribute: .bottom,
                                       constant: 0 - tabBarHeight)
    }
    
    @objc private func refreshData() {
        homepageServer.getTodayTrips()
        homepageServer.getFrequentocations()
    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        self.addGradiantBackground(color: PinitColors.yellow.cgColor)
    }
    
}

extension HomepageViewController : UITableViewDelegate, UITableViewDataSource {
    
    func numberOfSections(in tableView: UITableView) -> Int {
        return 2
    }
    
    func tableView(_ tableView: UITableView, titleForHeaderInSection section: Int) -> String? {
        switch section {
        case 0:
            return "Today's Trips"
        case 1:
            return "Most Frequent Destinations"
        default:
            return nil
        }
    }
    
    func tableView(_ tableView: UITableView, numberOfRowsInSection section: Int) -> Int {
        switch section {
        case 0:
            return (dailyTrips.count > 0) ? dailyTrips.count : 1
        case 1:
            return (frequentLocations.count > 0) ? frequentLocations.count : 1
        default:
            return 0
        }
    }
    
    func tableView(_ tableView: UITableView, cellForRowAt indexPath: IndexPath) -> UITableViewCell {
        switch indexPath.section {
        case 0:
            if dailyTrips.count > 0 {
                let cell = tableView.dequeueReusableCell(
                    withIdentifier: dailyCellIdentifier,
                    for: indexPath) as! TodayTripsTableViewCell
                let value = dailyTrips[indexPath.row]
                cell.pickupLocationName.text = value.pickUpName
                cell.destinationLocationName.text = value.destinationName
                cell.timeLabel.text = value.time
                return cell
            } else {
                let cell = tableView.dequeueReusableCell(
                    withIdentifier: noResultsIdentifier,
                    for: indexPath) as! NoResultsTableViewCell
                cell.noResultsLabel.text = "No trips done today"
                return cell
            }
        case 1:
            if frequentLocations.count > 0 {
                let cell = tableView.dequeueReusableCell(
                    withIdentifier: frequentCellIdentifier,
                    for: indexPath) as! FrequentLocationsTableViewCell
                cell.locationName.text = frequentLocations[indexPath.row].name
                cell.locationFrequency.text = frequentLocations[indexPath.row].frequncyString
                return cell
            } else {
                let cell = tableView.dequeueReusableCell(
                    withIdentifier: noResultsIdentifier,
                    for: indexPath) as! NoResultsTableViewCell
                cell.noResultsLabel.text = "No Locations Available"
                return cell
            }
            
        default:
            return UITableViewCell(frame: CGRect.zero)
        }
    }
    
    func tableView(_ tableView: UITableView, didSelectRowAt indexPath: IndexPath) {
        switch indexPath.section {
        case 1:
            let alert = UIAlertController(
                title: "Username",
                message: dailyTrips[indexPath.row].username,
                preferredStyle: UIAlertController.Style.alert)
            alert.addAction(UIAlertAction(title: "OK", style: UIAlertAction.Style.default, handler: nil))
            self.present(alert, animated: true, completion: nil)
        default:
            return
        }
    }
    
    func tableView(_ tableView: UITableView, heightForRowAt indexPath: IndexPath) -> CGFloat {
        switch indexPath.section {
        case 0:
            return dailySectionTableViewCellHeight
        case 1:
            return frequentSectiontodayTableViewCellHeight
        default:
            return 0
        }
    }
    
    func tableView(_ tableView: UITableView, viewForHeaderInSection section: Int) -> UIView? {
        let sectionHeader = HomepageSectionHeaderView(tableView: tableView)
        sectionHeader.headerSectionTitle.text = self
            .tableView(self.tableView, titleForHeaderInSection: section)
        return sectionHeader
    }
    
    func tableView(_ tableView: UITableView, viewForFooterInSection section: Int) -> UIView? {
        return UIView(frame: CGRect(x: 0.0, y: 0.0, width: 0.0, height: 0.01))
    }

    func tableView(_ tableView: UITableView, heightForFooterInSection section: Int) -> CGFloat {
        return 0
    }
    
    func tableView(_ tableView: UITableView, heightForHeaderInSection section: Int) -> CGFloat {
        return ( section == 0 ) ? 44 : self.inBetweenSectionSpacing
    }
}

extension HomepageViewController: HomepageServerDelegate {
    
    func didUpdateTodayTrips(trips: [TripInfo]) {
        dailyTrips = trips
        tableView.reloadData()
    }
    
    func didGetFrequentLocations(locations: [LocationInfo]) {
        frequentLocations = locations
        tableView.reloadData()
    }

}
