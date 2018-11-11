import mongoengine

class Waypoint(mongoengine.Document):
    waypoint_name = mongoengine.StringField(unique=True)
    description = mongoengine.StringField()
    robot_name = mongoengine.StringField()
    row = mongoengine.IntField()
    column = mongoengine.IntField()