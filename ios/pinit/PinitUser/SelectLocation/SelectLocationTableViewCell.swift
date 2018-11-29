import UIKit

class SelectLocationTableViewCell: UITableViewCell {
    
    public var locationName: UILabel
    
    public var locationDescription: UILabel
    
    public var pinImage: UIImageView
    
    override init(style: UITableViewCell.CellStyle, reuseIdentifier: String?) {
        locationName = UILabel()
        locationDescription = UILabel()
        pinImage = UIImageView(image: UIImage(named: "mapPinIcon"))
        super.init(style: style, reuseIdentifier: reuseIdentifier)
        
        addSubview(locationName)
        addSubview(locationDescription)
        addSubview(pinImage)
        
        let leftSpacing = self.bounds.width * 0.1
        let spacing = self.bounds.height * 0.2
        
        pinImage.translatesAutoresizingMaskIntoConstraints = false
        locationName.translatesAutoresizingMaskIntoConstraints = false
        locationDescription.translatesAutoresizingMaskIntoConstraints = false
        
        pinImage.leftAnchor.constraint(equalTo: self.leftAnchor, constant: leftSpacing).isActive = true
        pinImage.centerYAnchor.constraint(equalTo: self.centerYAnchor).isActive = true
        pinImage.heightAnchor.constraint(equalTo: self.heightAnchor, multiplier: 0.4).isActive = true
        pinImage.widthAnchor.constraint(equalTo: pinImage.heightAnchor).isActive = true
        
        locationName.leftAnchor.constraint(equalTo: pinImage.rightAnchor, constant: leftSpacing).isActive = true
        locationName.topAnchor.constraint(equalTo: self.topAnchor, constant: spacing).isActive = true
        locationName.widthAnchor.constraint(equalTo: self.widthAnchor, multiplier: 0.5).isActive = true
        locationName.heightAnchor.constraint(equalTo: self.heightAnchor, multiplier: 0.35).isActive = true
        
        locationDescription.leftAnchor.constraint(equalTo: pinImage.rightAnchor, constant: leftSpacing).isActive = true
        locationDescription.topAnchor.constraint(equalTo: locationName.bottomAnchor, constant: 0).isActive = true
        locationDescription.widthAnchor.constraint(equalTo: self.widthAnchor, multiplier: 0.5).isActive = true
        locationDescription.heightAnchor.constraint(equalTo: locationName.heightAnchor).isActive = true
        
        locationName.font = UIFont(name: "Avenir", size: 14.0)
        locationDescription.font = UIFont(name: "Avenir-Oblique", size: 12.0)
        locationDescription.textColor = PinitColors.borderGray
    }
    
    required init?(coder aDecoder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }
}
