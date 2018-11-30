import mongoengine

class Trip(mongoengine.Document):
    created_by = mongoengine.StringField(unique=True)
    start_waypoint = mongoengine.StringField()
    end_waypoint = mongoengine.StringField()
    status = mongoengine.StringField(regex='(RoutingToPickup)|(AwaitingConfirmation)|(RoutingToDestination)|(Completed)')
    robot_name = mongoengine.StringField()