# an abstract geometry object that can be made as parametric geometry class
# or a determinate mesh object

class UnitGeometry(object):
    def __init__(self):
        pass

    # --------------------------------------------------------------------------
    # attributes
    # --------------------------------------------------------------------------
    def centroid(self, frame):
        """Compute the centroid of the element.

        Returns
        -------
        point
            The XYZ location of the centroid.

        """
        self.centroid = frame[0]

        return self.centroid
