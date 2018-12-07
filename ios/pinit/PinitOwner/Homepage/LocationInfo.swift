import Foundation
import SwiftProtobuf

public class LocationInfo {
    
    public private(set) var name: String
    
    public private(set) var frequency: Int
    
    public var frequncyString: String {
        return "\(frequency)"
    }
    
    init(name: String, frequency: Int) {
        self.name = name
        self.frequency = frequency
    }
    
}
