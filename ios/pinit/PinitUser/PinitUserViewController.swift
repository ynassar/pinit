import UIKit
import CoreLocation

class PinitUserViewController: PinitSideMenuNavigationController, CLLocationManagerDelegate, RobotRequestServerDelegate {
    
    private var robotRequestView: RobotRequestView!
    
    private var robotRequestServer: RobotRequestServer!
    
    private var locationManager: CLLocationManager!
    
    override func viewDidLoad() {
        robotRequestView = RobotRequestView()
        locationManager = CLLocationManager()
        robotRequestServer = RobotRequestServer()
        self.controllerViews.append(robotRequestView)
        super.viewDidLoad()

        self.view.addSubview(robotRequestView)
        
        self.view.backgroundColor = .white

        locationManager.delegate = self
        locationManager.requestWhenInUseAuthorization()
        
        robotRequestServer.delegate = self

        robotRequestView = robotRequestView
            .addCenterXConstraint(relativeView: self.view)
            .addCenterYConstraint(relativeView: self.view)
            .addWidthConstraint(relativeView: self.view, multipler: 1.0)
            .addHeightConstraint(relativeView: self.view, multipler: 0.5)

        robotRequestView.getGpsCoordinatesButton.addTarget(
            self,
            action: #selector(self.getGpsCoordinatesButtonClick),
            for: .touchUpInside)

    }
        
    @objc private func getGpsCoordinatesButtonClick() {
        locationManager.desiredAccuracy = kCLLocationAccuracyBest
        locationManager.delegate = self
        locationManager.startUpdatingLocation()
    }
    
    func locationManager(_ manager: CLLocationManager, didUpdateLocations locations: [CLLocation]) {
        if let location = locations.first {
            locationManager.stopUpdatingLocation()
            locationManager.delegate = nil
            robotRequestServer.requestRobotToLocation(gpsCoordinates: location)
            print("Longtitude: \(location.coordinate.longitude)")
            print("Latitude: \(location.coordinate.latitude)")
        }
    }
    
    func locationManager(_ manager: CLLocationManager, didFailWithError error: Error) {
        print("Failed to find user's location: \(error.localizedDescription)")
    }
    
    func didRequestSuccessfully() {
        print("Success")
    }
    
    func didFailToRequestRobot(erroMessage: String) {
        print("Eroorrr")
    }
    
    override func viewDidAppear(_ animated: Bool) {
        super.viewDidAppear(animated)
        robotRequestView.getGpsCoordinatesButton.makeButtonCircular()
        
        
        let gradiantLayer = CAGradientLayer()
        gradiantLayer.frame = self.view.bounds
        gradiantLayer.startPoint = CGPoint(x: 0, y: 0)
        gradiantLayer.endPoint = CGPoint(x: 0, y: 1)
        gradiantLayer.colors = [UIColor.white.cgColor,
                                UIColor.white.cgColor,
                                PinitColors.red.cgColor]
        self.view.layer.insertSublayer(gradiantLayer, at: 0)
    }
}
