import bpy
import mathutils
import math
import os

def render_views():
    artifacts_dir = os.path.join(os.path.dirname(__file__), "..", "renders")
    os.makedirs(artifacts_dir, exist_ok=True)

    scene = bpy.context.scene
    scene.render.engine = 'CYCLES'
    scene.cycles.samples = 16
    scene.render.resolution_x = 1280
    scene.render.resolution_y = 960

    # Target focus point (center of lunabot)
    target = mathutils.Vector((0.0, 0.0, 0.45))

    # Ensure light source exists
    if "SunLight" not in bpy.data.objects:
        light_data = bpy.data.lights.new(name="SunLight", type='SUN')
        light_data.energy = 3.0
        light_object = bpy.data.objects.new(name="SunLight", object_data=light_data)
        bpy.context.collection.objects.link(light_object)
        light_object.rotation_euler = (math.radians(45), math.radians(15), math.radians(30))

    # Ensure camera exists
    if "RenderCam" not in bpy.data.objects:
        cam_data = bpy.data.cameras.new(name="RenderCam")
        cam_obj = bpy.data.objects.new(name="RenderCam", object_data=cam_data)
        bpy.context.collection.objects.link(cam_obj)
        scene.camera = cam_obj
    else:
        cam_obj = bpy.data.objects["RenderCam"]
        scene.camera = cam_obj

    def setup_camera(loc_tuple):
        cam_obj.location = mathutils.Vector(loc_tuple)
        direction = target - cam_obj.location
        rot_quat = direction.to_track_quat('-Z', 'Y')
        cam_obj.rotation_euler = rot_quat.to_euler()

    views = {
        "lunabot_mechanical_perspective": (2.2, -2.2, 1.6),
        "lunabot_mechanical_front": (2.5, 0.0, 0.45),
        "lunabot_mechanical_rear": (-2.5, 0.0, 0.45),
        "lunabot_mechanical_left": (0.0, 2.5, 0.45),
        "lunabot_mechanical_right": (0.0, -2.5, 0.45)
    }

    for vname, loc in views.items():
        setup_camera(loc)
        out_path = os.path.join(artifacts_dir, f"{vname}.png")
        scene.render.filepath = out_path
        bpy.ops.render.render(write_still=True)
        print(f"📸 RENDERED HIGH-RES SCREENSHOT -> {out_path}")

if __name__ == '__main__':
    render_views()
