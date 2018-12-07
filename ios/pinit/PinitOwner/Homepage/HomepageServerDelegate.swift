import UIKit

public protocol HomepageServerDelegate {
    
    func didUpdateTodayTrips(trips: [TripInfo])
    
    func didGetFrequentLocations(locations: [LocationInfo])
}
