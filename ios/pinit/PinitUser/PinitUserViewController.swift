import UIKit
import CoreLocation

class PinitUserViewController: PinitSideMenuNavigationController, CLLocationManagerDelegate {
    
    private var robotRequestView: RobotRequestView!
    
    private var locationManager: CLLocationManager!
    
    override func viewDidLoad() {
        robotRequestView = RobotRequestView()
        locationManager = CLLocationManager()
        self.controllerViews.append(robotRequestView)
        super.viewDidLoad()

        self.view.addSubview(robotRequestView)
        
        self.view.backgroundColor = .white

        locationManager.delegate = self
        locationManager.requestWhenInUseAuthorization()

        robotRequestView = robotRequestView
            .addCenterXConstraint(relativeView: self.view)
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
            print("Longtitude: \(location.coordinate.longitude)")
            print("Latitude: \(location.coordinate.latitude)")
            locationManager.stopUpdatingLocation()
        }
    }
    
    func locationManager(_ manager: CLLocationManager, didFailWithError error: Error) {
        print("Failed to find user's location: \(error.localizedDescription)")
    }
    
}
