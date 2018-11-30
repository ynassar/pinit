import UIKit

/// `SelectLocationServerDelegate` is implemented by all the classes that use
/// `SelectLocationServer` and want to be notified with the results of their request.
protocol SelectLocationServerDelegate {
    
    /// Function called when the `SelectLocationServer` get the `WaypointLIst` response
    /// and convert it to a list of `Location` to send to the delegate view controller
    /// to use these locations.
    func didUpdateLocations(locations: [Location])
    
}
