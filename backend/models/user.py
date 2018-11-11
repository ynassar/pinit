import mongoengine

class User(mongoengine.Document):
    email = mongoengine.EmailField(required=True, unique=True)
    username = mongoengine.StringField(required=True, unique=True, max_length=35)
    password_hash = mongoengine.StringField(required=True)
    is_owner = mongoengine.BooleanField()
    owned_robot = mongoengine.StringField(required=False)