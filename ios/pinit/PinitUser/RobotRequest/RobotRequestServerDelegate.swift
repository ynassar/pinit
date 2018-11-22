import Foundation
import UIKit

protocol RobotRequestServerDelegate {

    func didRequestSuccessfully()
    
    func didFailToRequestRobot(erroMessage: String)
}
