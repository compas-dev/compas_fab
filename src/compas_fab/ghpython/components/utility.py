from scriptcontext import sticky as st
from System.Drawing import Color
from compas_ghpython.utilities import update_component

from compas_ghpython.utilities import draw_frame


def create_id(component, name):
    return '%s_%s' % (name, component.InstanceGuid)


def draw_frames_component(frames, scale):

    red = Color.FromArgb(255, 0, 0)
    green = Color.FromArgb(0, 255, 0)
    blue = Color.FromArgb(0, 0, 255)

    P = []
    V = []
    C = []

    scale = float(scale)

    for frame in frames:
        if not frame:
            continue
        plane = draw_frame(frame)
        P += [plane.Origin] * 3
        V.append(plane.XAxis * scale)
        V.append(plane.YAxis * scale)
        V.append(plane.ZAxis * scale)
        C += [red, green, blue]

    W = 3
    return P, V, C, W


def counter(L, start=False, stop=False, step=False, back=False, reset=False, interval=None):
    """
    """
    interval = interval or 5
    counter_key = create_id('counter')
    running_key = create_id('running')

    if counter_key not in st or reset:
        st[counter_key] = 0

    if running_key not in st:
        st[running_key] = False

    if start:
        st[running_key] = True

    if stop:
        st[running_key] = False

    if step:
        if st[counter_key] < len(L) - 1:
            st[counter_key] += 1

    if back:
        if st[counter_key] > 1:
            st[counter_key] -= 1

    if st[running_key]:
        if st[counter_key] < len(L) - 1:
            st[counter_key] += 1
            update_component(ghenv, interval)  # noqa F821
        else:
            st[counter_key] = 0
            st[running_key] = False

    item = L[st[counter_key]] if len(L) else None

    return item, st[counter_key]
