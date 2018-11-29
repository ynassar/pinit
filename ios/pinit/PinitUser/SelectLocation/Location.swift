import Foundation
import SwiftProtobuf

class Location {
    
    public private(set) var name: String
    
    public private(set) var description: String
    
    init(name: String, description: String) {
        self.name = name
        self.description = description
    }
    
    init(waypoint: Waypoint) {
        self.name = waypoint.waypointName
        self.description = waypoint.description_p
    }
}
