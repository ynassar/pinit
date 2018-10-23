import Foundation
import UIKit

/// `MappingServerDelegate` is implemented by all the classes that use `MappingServer`
/// and want to be notified with the results of their request.
protocol MappingServerDelegate {
    
    /// Function called when an error occur when sending the `MappingRequest`. The
    /// `errorMessage` has more info about the type and source of error.
    func didMappingErrorOccur(_ errorMessage: String)
    
    func mapSaveConfirmation()
    
    func mapStartConfirmation()
    
    func mapImageUpdate(newImage: UIImage)
}
