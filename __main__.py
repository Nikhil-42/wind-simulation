import numpy as np


wind_speed = 1
wind_direction = 0

wind = np.array([wind_speed * np.cos(wind_direction), wind_speed * np.sin(wind_direction)])

class PhysicsObject(object):