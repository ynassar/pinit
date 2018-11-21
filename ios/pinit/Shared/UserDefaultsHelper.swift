import Foundation

class UserDefaultsHelper {
    
    public class func getUserProfile() -> Profile? {
        let userDefaults = UserDefaults.standard
        if let userProfileObject = userDefaults.object(forKey: PinitConstants.savedProfileKey) {
            let userProfileDecoded = userProfileObject as! Data
            let userProfile = NSKeyedUnarchiver.unarchiveObject(with: userProfileDecoded) as! Profile
            return userProfile
        }
        return nil
    }

}

