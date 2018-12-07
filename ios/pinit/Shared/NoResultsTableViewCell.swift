
import UIKit

class NoResultsTableViewCell: UITableViewCell {
    
    public var noResultsLabel: UILabel
  
    override init(style: UITableViewCell.CellStyle, reuseIdentifier: String?) {
        noResultsLabel = UILabel()
        super.init(style: style, reuseIdentifier: reuseIdentifier)
        
        addSubview(noResultsLabel)
        
        noResultsLabel.translatesAutoresizingMaskIntoConstraints = false
        
        self.backgroundColor = .clear
        self.isUserInteractionEnabled = false
        
        noResultsLabel.centerXAnchor.constraint(equalTo: self.centerXAnchor).isActive = true
        noResultsLabel.centerYAnchor.constraint(equalTo: self.centerYAnchor).isActive = true
        noResultsLabel.widthAnchor.constraint(equalTo: self.widthAnchor).isActive = true
        noResultsLabel.heightAnchor.constraint(equalTo: self.heightAnchor).isActive = true
        
        noResultsLabel.textAlignment = .center
    
        noResultsLabel.font = UIFont(name: "Avenir-BookOblique", size: 14.0)
        noResultsLabel.textColor = .black
    }
    
    required init?(coder aDecoder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }
}
