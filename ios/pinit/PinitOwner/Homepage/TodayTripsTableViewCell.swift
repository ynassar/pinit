import UIKit

class TodayTripsTableViewCell: UITableViewCell {
    
    public var timeLabel: UILabel
    
    public var pickupLocationName: UILabel
    
    public var destinationLocationName: UILabel
    
    private var routeImage: UIImageView
    
    override init(style: UITableViewCell.CellStyle, reuseIdentifier: String?) {
        timeLabel = UILabel()
        pickupLocationName = UILabel()
        destinationLocationName = UILabel()
        routeImage = UIImageView(image: UIImage(named: "routeIcon"))
        super.init(style: style, reuseIdentifier: reuseIdentifier)
        
        addSubview(timeLabel)
        addSubview(pickupLocationName)
        addSubview(destinationLocationName)
        addSubview(routeImage)
        
        self.backgroundColor = .clear
        
        timeLabel.translatesAutoresizingMaskIntoConstraints = false
        pickupLocationName.translatesAutoresizingMaskIntoConstraints = false
        destinationLocationName.translatesAutoresizingMaskIntoConstraints = false
        routeImage.translatesAutoresizingMaskIntoConstraints = false
        
        let leftSpacing = self.bounds.width * 0.1
        let spacing = self.bounds.height * 0.2
    
        timeLabel.leftAnchor.constraint(equalTo: self.leftAnchor, constant: leftSpacing).isActive = true
        timeLabel.centerYAnchor.constraint(equalTo: self.centerYAnchor).isActive = true
        timeLabel.heightAnchor.constraint(equalTo: self.heightAnchor, multiplier: 1.0).isActive = true
        timeLabel.widthAnchor.constraint(equalTo: self.widthAnchor, multiplier: 0.1).isActive = true
        
        routeImage.leftAnchor.constraint(equalTo: timeLabel.rightAnchor, constant: 0).isActive = true
        routeImage.centerYAnchor.constraint(equalTo: self.centerYAnchor).isActive = true
        routeImage.widthAnchor.constraint(equalTo: self.widthAnchor, multiplier: 0.1).isActive = true
        routeImage.heightAnchor.constraint(equalTo: routeImage.widthAnchor).isActive = true
        
        pickupLocationName.leftAnchor.constraint(equalTo: routeImage.rightAnchor, constant: 0).isActive = true
        pickupLocationName.topAnchor.constraint(equalTo: self.topAnchor, constant: spacing).isActive = true
        pickupLocationName.widthAnchor.constraint(equalTo: self.widthAnchor, multiplier: 0.5).isActive = true
        pickupLocationName.heightAnchor.constraint(equalTo: self.heightAnchor, multiplier: 0.35).isActive = true
        
        destinationLocationName.leftAnchor.constraint(equalTo: routeImage.rightAnchor, constant: 0).isActive = true
        destinationLocationName.topAnchor.constraint(equalTo: pickupLocationName.bottomAnchor, constant: 0).isActive = true
        destinationLocationName.widthAnchor.constraint(equalTo: self.widthAnchor, multiplier: 0.5).isActive = true
        destinationLocationName.heightAnchor.constraint(equalTo: pickupLocationName.heightAnchor).isActive = true
        
        pickupLocationName.font = UIFont(name: "Avenir", size: 14.0)
        destinationLocationName.font = UIFont(name: "Avenir", size: 14.0)
        timeLabel.font = UIFont(name: "Avenir", size: 11.0)
        
        timeLabel.textColor = PinitColors.borderGray
        
        pickupLocationName.text = "Pick up Location"
        destinationLocationName.text = "Destination location"
        timeLabel.text = "Date"
    }
    
    required init?(coder aDecoder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }
}
