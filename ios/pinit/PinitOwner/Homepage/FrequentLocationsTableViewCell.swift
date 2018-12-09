import UIKit

class FrequentLocationsTableViewCell: UITableViewCell {
    
    public var locationName: UILabel
    
    public var locationFrequency: UILabel
    
    override init(style: UITableViewCell.CellStyle, reuseIdentifier: String?) {
        locationName = UILabel()
        locationFrequency = UILabel()
        super.init(style: style, reuseIdentifier: reuseIdentifier)
        
        addSubview(locationName)
        addSubview(locationFrequency)
        
        self.backgroundColor = .clear
        self.isUserInteractionEnabled = false
        
        locationName.translatesAutoresizingMaskIntoConstraints = false
        locationFrequency.translatesAutoresizingMaskIntoConstraints = false
        
        let leftSpacing = self.bounds.width * 0.1
        
        locationName.leftAnchor.constraint(equalTo: self.leftAnchor, constant: leftSpacing).isActive = true
        locationName.centerYAnchor.constraint(equalTo: self.centerYAnchor).isActive = true
        locationName.heightAnchor.constraint(equalTo: self.heightAnchor).isActive = true
        locationName.widthAnchor.constraint(equalTo: self.widthAnchor, multiplier: 0.7).isActive = true
        
        locationFrequency.leftAnchor.constraint(equalTo: locationName.rightAnchor).isActive = true
        locationFrequency.centerYAnchor.constraint(equalTo: self.centerYAnchor).isActive = true
        locationFrequency.heightAnchor.constraint(equalTo: self.heightAnchor).isActive = true
        locationFrequency.widthAnchor.constraint(equalTo: locationFrequency.heightAnchor).isActive = true
        
        locationFrequency.textAlignment = .center
        
        locationFrequency.font = UIFont(name: "Avenir", size: 14.0)
        locationName.font = UIFont(name: "Avenir", size: 14.0)
                
    }
    
    required init?(coder aDecoder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }
    
}
