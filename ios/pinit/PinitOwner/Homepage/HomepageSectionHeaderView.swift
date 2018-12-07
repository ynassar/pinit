import UIKit

class HomepageSectionHeaderView: UIView {
    
    public var headerSectionTitle: UILabel
    
    init(tableView: UITableView) {
        headerSectionTitle = UILabel()
        super.init(frame: CGRect.zero)
        
        self.addSubview(headerSectionTitle)

        headerSectionTitle.translatesAutoresizingMaskIntoConstraints = false
        
        let leftSpacing = tableView.bounds.width * 0.05
        
        headerSectionTitle.widthAnchor.constraint(equalTo: self.widthAnchor, multiplier: 1.0).isActive = true
        headerSectionTitle.heightAnchor.constraint(equalTo: self.heightAnchor, multiplier: 0.5).isActive = true
        headerSectionTitle.leftAnchor.constraint(equalTo: self.leftAnchor, constant: leftSpacing).isActive = true
        headerSectionTitle.bottomAnchor.constraint(equalTo: self.bottomAnchor, constant: 0).isActive = true

        
        headerSectionTitle.font = UIFont(name: "Avenir", size: 14.0)
        headerSectionTitle.textColor = PinitColors.borderGray
        
    }
    
    required init?(coder aDecoder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }
    
}
