import os
import base64
import System

HERE = os.path.dirname(__file__)
icon_path = os.path.join(HERE, "icons")


def bitmap_from_image_path(image_path):
    with open(image_path, "rb") as imageFile:
        img_string = base64.b64encode(imageFile.read())
    return System.Drawing.Bitmap(System.IO.MemoryStream(System.Convert.FromBase64String(img_string)))


attached_collision_mesh_icon = bitmap_from_image_path(os.path.join(icon_path, 'attached_collision_mesh.png'))
collision_mesh_icon = bitmap_from_image_path(os.path.join(icon_path, 'collision_mesh.png'))
contraints_from_plane_icon = bitmap_from_image_path(os.path.join(icon_path, 'contraints_from_plane.png'))
inverse_kinematics_icon = bitmap_from_image_path(os.path.join(icon_path, 'inverse_kinematics.png'))
planning_scene_icon = bitmap_from_image_path(os.path.join(icon_path, 'planning_scene.png'))
plan_cartesian_motion_icon = bitmap_from_image_path(os.path.join(icon_path, 'plan_cartesian_motion.png'))
plan_motion_icon = bitmap_from_image_path(os.path.join(icon_path, 'plan_motion.png'))
robot_visualize_icon = bitmap_from_image_path(os.path.join(icon_path, 'robot_visualize.png'))
ros_connect_icon = bitmap_from_image_path(os.path.join(icon_path, 'ros_connect.png'))
ros_robot_icon = bitmap_from_image_path(os.path.join(icon_path, 'ros_robot.png'))
trajectory_visualize_icon = bitmap_from_image_path(os.path.join(icon_path, 'trajectory_visualize.png'))


# for img in os.listdir(icon_path):
#    basename = os.path.splitext(img)[0]
#    print("%s_icon = bitmap_from_image_path(os.path.join(icon_path, '%s.png'))" % (basename, basename))
