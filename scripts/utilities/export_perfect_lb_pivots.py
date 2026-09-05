import bpy
import os
import mathutils

def export_perfect_pivots():
    print("=========================================================================")
    print(" EXPORTING PERFECT PIVOT-CENTERED MESHES FROM LB.BLEND")
    print("=========================================================================")

    mesh_dir = "/home/dk05/Desktop/SMART_HORIZON/LUNA_PRO/environment/models/lunabot/meshes"
    os.makedirs(mesh_dir, exist_ok=True)

    all_objs = {o.name: o for o in bpy.data.objects if o.type == 'MESH'}

    # Define exact mechanical pivot origins in LB.blend
    pivot_origins = {
        'chassis': mathutils.Vector((0.000, 0.000, 0.810)),
        'differential_bar': mathutils.Vector((0.000, 0.000, 0.700)),
        'left_rocker': mathutils.Vector((0.000, 0.380, 0.700)),
        'right_rocker': mathutils.Vector((0.000, -0.380, 0.700)),
        'left_bogie': mathutils.Vector((0.300, 0.420, 0.350)),
        'right_bogie': mathutils.Vector((0.300, -0.420, 0.350)),
        'left_front_wheel': mathutils.Vector((0.650, 0.470, 0.175)),
        'left_middle_wheel': mathutils.Vector((0.000, 0.470, 0.175)),
        'left_rear_wheel': mathutils.Vector((-0.650, 0.470, 0.175)),
        'right_front_wheel': mathutils.Vector((0.650, -0.470, 0.175)),
        'right_middle_wheel': mathutils.Vector((0.000, -0.470, 0.175)),
        'right_rear_wheel': mathutils.Vector((-0.650, -0.470, 0.175))
    }

    groups = {
        "chassis": [],
        "differential_bar": [],
        "left_rocker": [],
        "right_rocker": [],
        "left_bogie": [],
        "right_bogie": [],
        "left_front_wheel": [],
        "left_middle_wheel": [],
        "left_rear_wheel": [],
        "right_front_wheel": [],
        "right_middle_wheel": [],
        "right_rear_wheel": []
    }

    excluded_names = ["preview_ground", "ground", "plane", "floor"]

    for name, obj in all_objs.items():
        nl = name.lower()
        if any(ex in nl for ex in excluded_names):
            continue

        if "left_front" in nl or "lf_" in nl:
            groups["left_front_wheel"].append(obj)
        elif "left_middle" in nl or "lm_" in nl:
            groups["left_middle_wheel"].append(obj)
        elif "left_rear" in nl or "lr_" in nl:
            groups["left_rear_wheel"].append(obj)
        elif "right_front" in nl or "rf_" in nl:
            groups["right_front_wheel"].append(obj)
        elif "right_middle" in nl or "rm_" in nl:
            groups["right_middle_wheel"].append(obj)
        elif "right_rear" in nl or "rr_" in nl:
            groups["right_rear_wheel"].append(obj)
        elif "left_bogie" in nl or "bogie_left" in nl:
            groups["left_bogie"].append(obj)
        elif "right_bogie" in nl or "bogie_right" in nl:
            groups["right_bogie"].append(obj)
        elif "left_v_" in nl or "left_rocker" in nl:
            groups["left_rocker"].append(obj)
        elif "right_v_" in nl or "right_rocker" in nl:
            groups["right_rocker"].append(obj)
        elif "diff" in nl or "tiebar" in nl or "crossbar" in nl:
            groups["differential_bar"].append(obj)
        else:
            groups["chassis"].append(obj)

    # Export meshes zero-centered around pivot_origins
    for gname, gobjs in groups.items():
        if not gobjs:
            continue
            
        if bpy.context.object and bpy.context.object.mode != 'OBJECT':
            bpy.ops.object.mode_set(mode='OBJECT')
        bpy.ops.object.select_all(action='DESELECT')

        pivot = pivot_origins[gname]
        
        dup_objs = []
        for o in gobjs:
            o_copy = o.copy()
            o_copy.data = o.data.copy()
            bpy.context.collection.objects.link(o_copy)
            o_copy.select_set(True)
            dup_objs.append(o_copy)

        bpy.context.view_layer.objects.active = dup_objs[0]
        if len(dup_objs) > 1:
            bpy.ops.object.join()

        unified_obj = bpy.context.view_layer.objects.active
        unified_obj.name = f"LB_{gname}"
        
        bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
        unified_obj.location -= pivot
        bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)

        export_path = os.path.join(mesh_dir, f"{gname}.glb")
        bpy.ops.export_scene.gltf(
            filepath=export_path,
            export_format='GLB',
            use_selection=True,
            export_yup=False
        )

        bpy.ops.object.delete()
        print(f"✅ EXPORTED PIVOT-CENTERED GLB -> {export_path}")

    print("\n=========================================================================")
    print(" ALL 12 PIVOT-CENTERED GLBS EXPORTED FROM LB.BLEND SUCCESSFULLY!")
    print("=========================================================================")

if __name__ == '__main__':
    export_perfect_pivots()
