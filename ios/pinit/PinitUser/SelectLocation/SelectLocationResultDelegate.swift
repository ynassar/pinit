import UIKit

/// `SelectLocationResultDelegate` is implemented by all the classes that use
/// `SelectLocationViewController` to choose a location and want to be notified
/// which location has been selected.
protocol SelectLocationResultDelegate {
    
    /// Function which returns the `Location` that has been selected in the
    /// `SelectLocationViewController` to the delegate view controller which
    /// called the select view controller.
    func getLocationSelected(location: Location)
    
}
