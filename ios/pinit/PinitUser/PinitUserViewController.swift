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
        self.view.addSubview(buttonTest)
        
        locationManager.delegate = self
        locationManager.requestWhenInUseAuthorization()
        
        let profileMenuButton = UIButton(frame: CGRect.zero)
        profileMenuButton.setImage(UIImage(named: "menuIcon"), for: .normal)
        
        self.navigationController?.navigationBar.topItem?.leftBarButtonItems = [
            UIBarButtonItem(customView: profileMenuButton)
        ]
        
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
            //            print("Found user's location: \(location)")
            print("Longtitude: \(location.coordinate.longitude)")
            print("Latitude: \(location.coordinate.latitude)")
            locationManager.stopUpdatingLocation()
        }
    }
    
    func locationManager(_ manager: CLLocationManager, didFailWithError error: Error) {
        print("Failed to find user's location: \(error.localizedDescription)")
    }
    
    override func viewDidLayoutSubviews() {
        super.viewDidLayoutSubviews()
//        let gradiantLayer = CAGradientLayer()
//        gradiantLayer.frame = robotRequestView.bounds
//        gradiantLayer.colors = [PinitColors.yellow.cgColor,
//                                PinitColors.red.cgColor,
//                                PinitColors.blue.cgColor,
//                                PinitColors.green.cgColor]
//        robotRequestView.layer.insertSublayer(gradiantLayer, at: 0)
    }
}
