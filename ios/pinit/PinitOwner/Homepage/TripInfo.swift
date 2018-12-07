import Foundation
import SwiftProtobuf

public class TripInfo {
    
    public private(set) var pickUpName: String
    
    public private(set) var destinationName: String
    
    public private(set) var date: Date
    
    public private(set) var username: String
    
    public var time: String {
        get {
            let dateFormatter = DateFormatter()
            dateFormatter.dateFormat = "hh:mm a"
            return dateFormatter.string(from: date)
        }
    }
    
    init(pickUpName: String, destinationName: String, date: Date, username: String) {
        self.pickUpName = pickUpName
        self.destinationName = destinationName
        self.date = date
        self.username = username
    }
    
    init (trip: Trip) {
        self.pickUpName = trip.startWaypoint
        self.destinationName = trip.endWaypoint
        self.date = trip.timestamp.date
        self.username = trip.creator
    }
    
}
