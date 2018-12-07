import UIKit
import SwiftProtobuf

public class HomepageServer {
    
    public var delegate: HomepageServerDelegate?
    
    private var accountClient: RosServiceServiceClient!
    
    init() {
        accountClient = RosServiceServiceClient(
            address: PinitConstants.robotServerAddress,
            secure: false,
            arguments: [])
    }
    
    public func getTodayTrips() {
        var getTodayTripsRequest = GetTodaysTripsRequest()
        if let userProfile = UserDefaultsHelper.getUserProfile() {
            getTodayTripsRequest.token = userProfile.token
        }
        do {
            let tripLists = try accountClient.getTodaysTrips(getTodayTripsRequest) as TripList
            let trips = tripLists.trips.map{ TripInfo(trip: $0)}
            delegate?.didUpdateTodayTrips(trips: trips)
        } catch {
            print("Error in getting todays trips")
        }
    }
    
    public func getFrequentocations() {
        var getMostVisitedWaypointsRequest = GetMostVisitedWaypointsRequest()
        getMostVisitedWaypointsRequest.numResults = 100
        if let userProfile = UserDefaultsHelper.getUserProfile() {
            getMostVisitedWaypointsRequest.token = userProfile.token
        }
        do {
            let waypointsNameList = try accountClient
                .getMostVisitedWaypoints(getMostVisitedWaypointsRequest)
            let locations = zip(waypointsNameList.waypointNames,
                                waypointsNameList.frequencies)
                .map { LocationInfo(name: $0.0, frequency: Int($0.1))}
            delegate?.didGetFrequentLocations(locations: locations)
        } catch {
            print("Error in getting most frequent locations")
        }
        
        
        
    }
    
}
