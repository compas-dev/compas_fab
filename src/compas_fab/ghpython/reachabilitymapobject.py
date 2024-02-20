from compas.colors import ColorMap
from compas.scene import SceneObject
from compas_ghpython.scene import GHSceneObject


class ReachabilityMapObject(GHSceneObject):
    """Scene object for drawing a reachability map.

    Parameters
    ----------
    reachability_map : :class:`compas_robots.ReachabilityMap`
        Robot model.
    **kwargs : dict, optional
        Additional keyword arguments.
    """

    def __init__(self, reachability_map, **kwargs):
        super(ReachabilityMapObject, self).__init__(**kwargs)
        self.reachability_map = reachability_map

    def draw_frames(self, ik_index=None):
        """Returns the frames of the reachability map.

        Parameters
        ----------
        ik_index : int, optional
            If passed, returns only the reachable frames at a given IK index. For a 6-axis industrial robot this
            index reaches from 0 to 7 (8 solutions).
        """
        from ghpythonlib.treehelpers import list_to_tree

        if ik_index is None:
            xframes = []
            for frames in self.reachability_map.frames:
                xframes.append([])
                for frame in frames:
                    xframes[-1].append(SceneObject(frame).draw())
            xframes = list_to_tree(xframes)
            return xframes
        else:
            frames, _ = self.reachability_map.reachable_frames_and_configurations_at_ik_index(ik_index)
            return [SceneObject(f).draw() for f in frames]

    def draw(self, colormap="viridis"):
        return self.draw_cloud(colormap)

    def draw_cloud(self, colormap="viridis", points=None):
        """Returns the points and colors to create a point cloud.

        The colors are calculated on the score at the respective frame. If the
        frames are a 2D list, the point of the first frame of the list is used.

        Parameters
        ----------
        colormap : str, optional
            The colormap for the point cloud.
        points : list of :class:`compas.geometry.Points`, optional
            Points to override the points from the reachability map.
        """

        from System.Drawing import Color

        points = points or self.reachability_map.points

        xpoints = []
        for pt in points:
            xpoints.extend(SceneObject(pt).draw())

        colors = []
        cmap = ColorMap.from_mpl(colormap)
        score = self.reachability_map.score
        minv, maxv = min(score), max(score)

        for num in score:
            color = cmap(num, minv, maxv)
            r, g, b, a = color.rgba255
            rcolor = Color.FromArgb(a, r, g, b)
            colors.append(rcolor)

        return xpoints, colors
