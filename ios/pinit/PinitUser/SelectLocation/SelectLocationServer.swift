import UIKit
import SwiftProtobuf
import CoreLocation

/// Class responsible for getting the current gps coordinates of the user, sending it to
/// gRPC server, and getting the list of locations of the nearest robot to the user.
class SelectLocationServer: NSObject, CLLocationManagerDelegate {
    
    /// The delegate which the server will notify when the locations are retrieved.
    public var delegate: SelectLocationServerDelegate?
    
    /// The location manager used to start and stop getting the phone's location.
    private var locationManager: CLLocationManager
    
    /// Initializer of the server.
    override init() {
        locationManager = CLLocationManager()
        super.init()
        locationManager.requestWhenInUseAuthorization()
    }
    
    /// Function called to retrieve the locations of the closest robot. It starts by getting
    /// the gps coordinates.
    ///
    public func getLocations() {
        locationManager.desiredAccuracy = kCLLocationAccuracyBest
        locationManager.delegate = self
        locationManager.startUpdatingLocation()
    }
    
    /// Function responsible for calling sending the gps coordinates after they are recieved
    /// to the gRPC server to get the `WaypointList` and converting it to a list of `Location`
    /// and returing this list to the delegate.
    func locationManager(
        _ manager: CLLocationManager,
        didUpdateLocations locations: [CLLocation]
    ) {
        if let location = locations.first {
            locationManager.stopUpdatingLocation()
            locationManager.delegate = nil
            
            let accountClient = RosServiceServiceClient(
                address: PinitConstants.robotServerAddress,
                secure: false,
                arguments: [])
            
            var sentGpsCoordinates = GpsCoordinates()
            sentGpsCoordinates.longitude = Float(location.coordinate.longitude)
            sentGpsCoordinates.latitude = Float(location.coordinate.latitude)
            
            do {
                let waypointList = try accountClient.getNearbyWaypoints(sentGpsCoordinates) as WaypointList
                let robotLocations = waypointList.waypoints.map { Location(waypoint: $0) }
                delegate?.didUpdateLocations(locations: robotLocations)
            } catch {
                print("Error")
            }

        }
    }
}

