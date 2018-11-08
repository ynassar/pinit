import Foundation
import UIKit

/// `MappingServerDelegate` is implemented by all the classes that use `MappingServer`
/// and want to be notified with the results of their request.
protocol MappingServerDelegate {
    
    /// Function called when an error occur when sending the `MappingRequest`. The
    /// `errorMessage` has more info about the type and source of error.
    func didMappingErrorOccur(_ errorMessage: String)
    
    /// Function called when the `MappingRequest` with they type `stopMapping` is
    /// sent indicating that no error occured, so the delegate view controller is
    /// notified if any actions depends on that request.
    func mapSaveConfirmation()
    
    /// Function called when the `MappingRequest` with they type `startMapping` is
    /// sent indicating that no error occured, so the delegate view controller is
    /// notified if any actions depends on that request.
    func mapStartConfirmation()
    
    /// Function that depends on  the method `mapImageRequestAsynchronous` of
    /// a `MappingServer` instance. The map image request function has a timer which
    /// gets the latest map in the database every 3 seconds, and `mapImageUpdate`
    /// notifies the deleagte view controller that the latest image has been received.
    func mapImageUpdate(newImage: UIImage)
}
