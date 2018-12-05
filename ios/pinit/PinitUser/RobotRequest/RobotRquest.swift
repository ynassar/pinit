import Foundation

class RobotRequest {
    
    public private(set) var pickUpName: String
    
    public private(set) var destinationName: String
    
    init(pickUp: String, destination: String) {
        self.pickUpName = pickUp
        self.destinationName = destination
    }
    
}
