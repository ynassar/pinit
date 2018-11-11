
class GreyscaleMapRenderer(object):

    LOW_CONFIDENCE_VALUE = 1
    HIGH_CONFIDENCE_VALUE = 101
    UNKNOWN_VALUE = 0

    HIGH_CONFIDENCE_COLOR = 0
    LOW_CONFIDENCE_COLOR = 255
    UNKNOWN_COLOR = 127

    def __init__(self):
        pass

    def RenderToArray(self, occupancy_grid_array):
        """Returns a numpy array representing a rendering of an occupancy grid."""
        image = occupancy_grid_array.copy()
        low_confidence_mask = occupancy_grid_array == self.LOW_CONFIDENCE_VALUE
        high_confidence_mask = occupancy_grid_array == self.HIGH_CONFIDENCE_VALUE
        unknown_mask = occupancy_grid_array == self.UNKNOWN_VALUE
        image[low_confidence_mask] = self.LOW_CONFIDENCE_COLOR
        image[high_confidence_mask] = self.HIGH_CONFIDENCE_COLOR
        image[unknown_mask] = self.UNKNOWN_COLOR
        return image