import UIKit

public class Profile : NSObject, NSCoding {
    
    public private(set) var username: String
    
    public private(set) var token: String
    
    public private(set) var ownerStatus: Bool
        
    init(username: String, token: String, ownerStatus: Bool) {
        self.username = username
        self.token = token
        self.ownerStatus = ownerStatus
    }
    
    public required convenience init?(coder aDecoder: NSCoder) {
        let username = aDecoder.decodeObject(forKey: "username") as! String
        let token = aDecoder.decodeObject(forKey: "token") as! String
        let ownerStatus = aDecoder.decodeBool(forKey: "ownerStatus")
        self.init(username: username, token: token, ownerStatus: ownerStatus)
    }
    
    public func encode(with aCoder: NSCoder) {
        aCoder.encode(self.username, forKey: "username")
        aCoder.encode(self.token, forKey: "token")
        aCoder.encode(self.ownerStatus, forKey: "ownerStatus")
    }
}
