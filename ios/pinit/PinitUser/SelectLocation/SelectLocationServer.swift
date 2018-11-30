import UIKit
import SwiftProtobuf
import CoreLocation

class SelectLocationServer: NSObject, CLLocationManagerDelegate {
    
    public var delegate: SelectLocationServerDelegate?
    
    private var locationManager: CLLocationManager
    
    override init() {
        locationManager = CLLocationManager()
        super.init()
        
        locationManager.requestWhenInUseAuthorization()
    }
    
    public func getLocations() {
        locationManager.desiredAccuracy = kCLLocationAccuracyBest
        locationManager.delegate = self
        locationManager.startUpdatingLocation()
    }
    
    func locationManager(_ manager: CLLocationManager, didUpdateLocations locations: [CLLocation]) {
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

