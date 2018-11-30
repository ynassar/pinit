import mongoengine


class Robot(mongoengine.Document):
    robot_name = mongoengine.StringField(required=True, unique=True)
    row = mongoengine.FloatField()
    column = mongoengine.FloatField()
    angle = mongoengine.FloatField()
    status = mongoengine.StringField(regex='(Mapping)|(Idle)|(RoutingToPickup)|(RoutingToDestination)|(Error)', default='Idle')
    position_timestamp = mongoengine.DateTimeField()
    trip = mongoengine.ReferenceField('Trip')