import UIKit
import CoreLocation

class RobotRequestViewController : TabBarNavigationController, CLLocationManagerDelegate, RobotRequestServerDelegate {

    private var robotRequestView: RobotRequestView!
    
    private var locationManager: CLLocationManager!
    
    private var robotRequestServer: RobotRequestServer!
    
    private var gpsCoordinates: CLLocationCoordinate2D!
    
    override func viewDidLoad() {
        robotRequestView = RobotRequestView()
        locationManager = CLLocationManager()
        robotRequestServer = RobotRequestServer()
        self.controllerViews.append(robotRequestView)
        super.viewDidLoad()
        self.view.addSubview(robotRequestView)
        
        robotRequestServer.delegate = self
        
        locationManager.delegate = self
        locationManager.requestWhenInUseAuthorization()

        robotRequestView = robotRequestView
            .addCenterYConstraint(relativeView: self.view)
            .addCenterYConstraint(relativeView: self.view)
            .addWidthConstraint(relativeView: self.view, multipler: 1.0)
            .addHeightConstraint(relativeView: self.view, multipler: 1.0)
        
        robotRequestView.getGpsCoordinatesButton.addTarget(
            self,
            action: #selector(self.getGpsCoordinatesButtonClick),
            for: .touchUpInside)
    }
    
    @objc private func getGpsCoordinatesButtonClick() {
        locationManager.desiredAccuracy = kCLLocationAccuracyBest
        locationManager.startUpdatingLocation()
    }
    
    func locationManager(_ manager: CLLocationManager, didUpdateLocations locations: [CLLocation]) {
        if let location = locations.first {

            if self.gpsCoordinates == nil {
                robotRequestServer.requestRobotToLocation(gpsCoordinates: location)
                print("Longtitude: \(location.coordinate.longitude)")
                print("Latitude: \(location.coordinate.latitude)")
            }
            
            locationManager.stopUpdatingLocation()
        }
    }
    
    func locationManager(_ manager: CLLocationManager, didFailWithError error: Error) {
        print("Failed to find user's location: \(error.localizedDescription)")
    }
    
    override func viewDidLayoutSubviews() {
        super.viewDidLayoutSubviews()
        let gradiantLayer = CAGradientLayer()
        gradiantLayer.frame = robotRequestView.bounds
        gradiantLayer.colors = [PinitColors.yellow.cgColor,
                                PinitColors.red.cgColor,
                                PinitColors.blue.cgColor,
                                PinitColors.green.cgColor]
        robotRequestView.layer.insertSublayer(gradiantLayer, at: 0)
    }
    
    func didRequestSuccessfully() {
        print("Success")
        self.gpsCoordinates = nil
    }
    
    func didFailToRequestRobot(erroMessage: String) {
        print("Eroorrr")
    }
    
}
