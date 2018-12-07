import UIKit

class HomepageViewController : LogoutNavigationController {
    
//    private var todayTripsView: TodayTripsView!
    
    private var tableView: UITableView!
    
    private let todayTableViewCellHeight: CGFloat = 66
    
    private let tableViewIdentifier =  "tableID"
    
    override func viewDidLoad() {
        tableView = UITableView(frame: CGRect.zero, style: .grouped)
        super.viewDidLoad()
        
        self.view.addSubview(tableView)
        
        tableView.register(TodayTripsTableViewCell.self, forCellReuseIdentifier: tableViewIdentifier)
        tableView.tableFooterView = UIView(frame: CGRect(x: 0.0, y: 0.0, width: 0.0, height: 0.01))
        tableView.tableHeaderView = UIView(frame: CGRect(x: 0.0, y: 0.0, width: 0.0, height: 0.01))
        tableView.contentInsetAdjustmentBehavior = .never
        
        tableView.delegate = self
        tableView.dataSource = self
        tableView.backgroundColor = .clear
        
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
            return "Most Frequent Trips"
        default:
            return nil
        }
    }
    
    func tableView(_ tableView: UITableView, numberOfRowsInSection section: Int) -> Int {
        switch section {
        case 0:
            return 8
        case 1:
            return 10
        default:
            return 0
        }
    }
    
    func tableView(_ tableView: UITableView, cellForRowAt indexPath: IndexPath) -> UITableViewCell {
        let cell = tableView.dequeueReusableCell(
            withIdentifier: tableViewIdentifier,
            for: indexPath)
        return cell
    }
    
    func tableView(_ tableView: UITableView, heightForRowAt indexPath: IndexPath) -> CGFloat {
        return self.todayTableViewCellHeight
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
        return ( section == 0 ) ? 44 : self.todayTableViewCellHeight
    }
}
