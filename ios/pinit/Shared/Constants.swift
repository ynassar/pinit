import UIKit

/// All the UIColor values of the colors used in pinit.
struct PinitColors {
    
    /// Gray color used for backgrounds.
    static public let gray = UIColor(red:0.98, green:0.98, blue:0.98, alpha:1.0)
    
    /// Gray color used for borders.
    static public let borderGray = UIColor(red:0.70, green:0.70, blue:0.70, alpha:1.0)
    
    /// Blue color used for different `UIButton`.
    static public let blue = UIColor(red:0.20, green:0.74, blue:0.83, alpha:1.0)

    static public let green = UIColor(red:0.60, green:0.69, blue:0.27, alpha:1.0)
    
    static public let red = UIColor(red:0.87, green:0.31, blue:0.34, alpha:1.0)

    static public let yellow = UIColor(red:0.96, green:0.81, blue:0.33, alpha:1.0)
    
    static public let gradiantColorsList = [PinitColors.yellow.cgColor,
                                            PinitColors.red.cgColor,
                                            PinitColors.blue.cgColor,
                                            PinitColors.green.cgColor]
    
    /// Light Blue color used for `UILabel` that acts like a link.
    static public let linkBlue = UIColor(red:0.24, green:0.60, blue:0.93, alpha:1.0)
    
}

/// All the common constant used in the different view in the app.
struct PinitConstants {
    
    /// The value of the corner radius of `UITextField` and `UIButton`.
    static public let cornerRadius: CGFloat = 10.0
    
    /// The font size of all the UI elements in the `AuthentucationView`
    static public let authenticationFontSize: CGFloat = 12.0
    
    /// The font size of all the UI elements in the `AuthentucationView`
    static public let addLocationFontSize: CGFloat = 12.0
    
    static public let navigationBarHeight: CGFloat = 44.0
    
    static public let savedProfileKey = "ProfileKey"
    
    static public let snapshotTag = 777
    
    static public var authenticationServerAddress: String {
        get {
            let ipAddress = UserDefaults.standard
                .string(forKey:SettingsBundleHelper.ipAddressIdentifier) ?? "localhost"
            return "\(ipAddress):50051"
        }
    }
    
    static public var robotServerAddress: String {
        get {
            let ipAddress = UserDefaults.standard
                .string(forKey:SettingsBundleHelper.ipAddressIdentifier) ?? "localhost"
            return "\(ipAddress):50052"
        }
    }

}
