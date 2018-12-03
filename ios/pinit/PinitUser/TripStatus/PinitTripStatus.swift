import UIKit
import SwiftProtobuf

public enum PinitTripStatus: Int {
    case GoingToPickUp = 0
    case WaitingForConfrimation = 1
    case GoingToDestination = 2
    case TripCompleted = 3
    case NoStatus = 4
    
    init(grpcTripStatus: TripStatus.TripStatusType) {
        switch grpcTripStatus {
        case .routingToPickup:
            self = .GoingToPickUp
        case .awaitingConfirmation:
            self = .WaitingForConfrimation
        case .routingToDestination:
            self = .GoingToDestination
        case .completed:
            self = .TripCompleted
        default:
            self = .NoStatus
        }
    }
}


