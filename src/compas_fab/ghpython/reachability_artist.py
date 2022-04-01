from compas.geometry import Frame
from compas.colors import ColorMap
from compas.artists import PrimitiveArtist
from ghpythonlib.treehelpers import list_to_tree


class ReachabilityMapArtist(object):  # how base on GHArtist without error?
    """Artist for drawing a reachability map.

    Parameters
    ----------
    reachability_map : :class:`compas.robots.ReachabilityMap`
        Robot model.
    **kwargs : dict, optional
        Additional keyword arguments.
    """

    def __init__(self, reachability_map, **kwargs):
        self.reachability_map = reachability_map

    def draw_frames(self, ik_index=None):
        """Returns the frames of the reachability map.

        Parameters
        ----------
        ik_index : int, optional
            If passed, returns only the reachable frames at a given IK index. For a 6-axis industrial robot this
            index reaches from 0 to 7 (8 solutions).
        """

        if ik_index == None:
            def convert_to_xframe(f, flist):
                if isinstance(f, Frame):
                    flist.append(PrimitiveArtist(f).draw())
                else:
                    flist.append([])
                    for subf in f:
                        convert_to_xframe(subf, flist[-1])
            frames = []
            [convert_to_xframe(f, frames) for f in self.reachability_map.frames]
            if len(self.reachability_map.shape) > 1:
                frames = list_to_tree(frames)  # does this cover lists which shape dim > 2
            return frames
        else:
            frames, _ = self.reachability_map.reachable_frames_and_configurations_at_ik_index(ik_index)
            return [PrimitiveArtist(f).draw() for f in frames]

    def draw(self, colormap='viridis'):
        return self.draw_cloud(colormap)

    def draw_cloud(self, colormap='viridis', points=None):
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

        xpoints = [PrimitiveArtist(pt).draw() for pt in points]

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
