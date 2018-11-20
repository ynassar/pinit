import mongoengine


class Robot(mongoengine.Document):
    robot_name = mongoengine.StringField(required=True, unique=True)
    row = mongoengine.IntField()
    column = mongoengine.IntField()
    angle = mongoengine.DecimalField()
    status = mongoengine.StringField(regex='(Mapping)|(Idle)|(RoutingToPickup)|(RoutingToDestination)|(Error)')
    position_timestamp = mongoengine.DateTimeField()