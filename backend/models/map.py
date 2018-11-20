import mongoengine

class Map(mongoengine.Document):
    robot_name = mongoengine.StringField()
    resolution = mongoengine.DecimalField(min_value=0)
    height = mongoengine.IntField(min_value=0)
    width = mongoengine.IntField(min_value=0)
    b64_image = mongoengine.StringField()
    raw_map = mongoengine.StringField()
    origin = mongoengine.PointField()