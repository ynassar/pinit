import Foundation
import SwiftProtobuf

/// `Location` class is responsible for representing a location.
class Location {
    
    /// The name of the location.
    public private(set) var name: String
    
    /// The description of the location.
    public private(set) var description: String
    
    /// Initializer of the location using the parameter.
    init(name: String, description: String) {
        self.name = name
        self.description = description
    }
    
    /// Initializer of the location from a `Waypoint` struct.
    init(waypoint: Waypoint) {
        self.name = waypoint.waypointName
        self.description = waypoint.description_p
    }
}
