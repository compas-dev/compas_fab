from compas.colors import ColorMap
from compas_rhino.artists import FrameArtist
from compas_rhino.artists import PointArtist
from compas_rhino.artists import RhinoArtist


class ReachabilityMapArtist(RhinoArtist):
    """Artist for drawing a reachability map.

    Parameters
    ----------
    reachability_map : :class:`compas.robots.ReachabilityMap`
        Robot model.
    scale: float, optional
        Scale factor that controls the length of the axes.
    layer : str, optional
        The layer that should contain the drawing.
    **kwargs : dict, optional
        Additional keyword arguments.
    """

    def __init__(self, reachability_map, layer=None, scale=1.0, **kwargs):
        super(ReachabilityMapArtist, self).__init__(layer=layer, **kwargs)
        self.reachability_map = reachability_map
        self.scale = scale or 1.0

    def draw_frames(self, ik_index=None):
        """Returns the frames of the reachability map.

        Parameters
        ----------
        ik_index : int, optional
            If passed, returns only the reachable frames at a given IK index. For a 6-axis industrial robot this
            index reaches from 0 to 7 (8 solutions).

        Returns
        -------
        list[System.Guid]
            The GUIDs of the created Rhino objects.
        """

        if ik_index is None:
            xframes = []
            for frames in self.reachability_map.frames:
                xframes.append([])
                for frame in frames:
                    xframes[-1].extend(FrameArtist(frame, layer=self.layer, scale=self.scale).draw())

            return xframes
        else:
            frames, _ = self.reachability_map.reachable_frames_and_configurations_at_ik_index(ik_index)
            return [FrameArtist(f, layer=self.layer, scale=self.scale).draw() for f in frames]

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

        Returns
        -------
        list[System.Guid]
            The GUIDs of the created Rhino objects.

        """

        points = points or self.reachability_map.points

        cmap = ColorMap.from_mpl(colormap)
        score = self.reachability_map.score
        minv, maxv = min(score), max(score)

        guids = []
        for num, pt in zip(score, points):
            color = cmap(num, minv, maxv)
            artist = PointArtist(pt, layer=self.layer)
            guids.extend(artist.draw(color))

        return guids
