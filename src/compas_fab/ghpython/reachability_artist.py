from compas.geometry import Frame
from compas.colors import ColorMap
from compas_ghpython.artists import FrameArtist
from compas_ghpython.artists import PointArtist
from ghpythonlib.treehelpers import list_to_tree


class ReachabilityMapArtist(object):  # how base on GHArtist without error?

    def __init__(self, reachability_map, **kwargs):
        self.reachability_map = reachability_map

    def draw_frames(self):

        if isinstance(self.reachability_map.frames[0], Frame):
            frames = [FrameArtist(f).draw() for f in self.reachability_map.frames]
        else:
            frames = []
            for frames_per_point in self.reachability_map.frames:
                frames.append([])
                for f in frames_per_point:
                    frames[-1].append(FrameArtist(f).draw())
            frames = list_to_tree(frames)
        return frames

    def draw_frames_at_ik_index(self, ik_index):
        frames, configurations = self.reachability_map.reachable_frames_and_configurations_at_ik_index(ik_index)
        return [FrameArtist(f).draw() for f in frames]

    def draw_cloud(self, colormap='viridis'):

        from System.Drawing import Color

        points = [PointArtist(pt).draw() for pt in self.reachability_map.points]

        colors = []
        cmap = ColorMap.from_mpl(colormap)
        score = self.reachability_map.score
        minv, maxv = min(score), max(score)

        for num in score:
            color = cmap(num, minv, maxv)
            r, g, b, a = color.rgba255
            rcolor = Color.FromArgb(a, r, g, b)
            colors.append(rcolor)

        return points, colors
