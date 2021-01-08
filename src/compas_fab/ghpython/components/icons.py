import os
import base64
import System

HERE = os.path.dirname(__file__)
icon_path = os.path.join(HERE, "icons")


def bitmap_from_image_path(image_path):
    with open(image_path, "rb") as imageFile:
        img_string = base64.b64encode(imageFile.read())
    return System.Drawing.Bitmap(System.IO.MemoryStream(System.Convert.FromBase64String(img_string)))


ros_connect_icon = bitmap_from_image_path(os.path.join(icon_path, "ros_connect.png"))
default_icon = bitmap_from_image_path(os.path.join(icon_path, "compas_icon_white.png"))
