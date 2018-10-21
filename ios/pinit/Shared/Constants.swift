import UIKit

/// All the UIColor values of the colors used in pinit.
struct PinitColors {
    
    static public let gray = UIColor(red:0.98, green:0.98, blue:0.98, alpha:1.0)
    
    static public let blue = UIColor(red:0.20, green:0.74, blue:0.83, alpha:1.0)

    static public let borderGreen = UIColor(red:0.60, green:0.69, blue:0.27, alpha:1.0)
    
    static public let linkBlue = UIColor(red:0.24, green:0.60, blue:0.93, alpha:1.0)
    
}

/// All the common constant used in the different view in the app.
struct PinitConstants {
    
    static public let cornerRadius: CGFloat = 10.0
    
    static public let authernticationFontSize: CGFloat = 12.0
    
    static public let authenticationServerAddress = "localhost:50051"
    
    static public let robotServerAddress = "localhost:50052"
    
    static public let tempRobotServerAddress = "10.40.37.149:50052"
    
    static public let navigationBarHeight: CGFloat = 44.0
}
