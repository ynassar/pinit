import Foundation

class SettingsBundleHelper {
    
    public static let ipAddressIdentifier = "IP_ADDRESS_IDENTIFIER"
    
    public class func setIpAddress() {
        UserDefaults.standard.register(defaults: [String : Any]())
    }
    
    public class func saveIpAddress(ipAddress: String) {
        UserDefaults.standard.set(ipAddress, forKey: ipAddressIdentifier)
    }
}
