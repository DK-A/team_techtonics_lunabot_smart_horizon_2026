import bpy
import os

def export_lunabot_meshes(blend_path, output_dir):
    os.makedirs(output_dir, exist_ok=True)
    
    # Map of mesh group names to export target filenames
    mesh_export_groups = {
        'chassis.glb': ['chassis', 'solar_panel_array', 'battery_monitor', 'rgb_camera_housing', 'depth_camera_housing', 'rear_camera_housing', 'imu_housing', 'environmental_intake_cluster', 'fl_encoder', 'fr_encoder', 'rl_encoder', 'rr_encoder'],
        'differential_bar.glb': ['differential_bar'],
        'left_rocker.glb': ['left_rocker'],
        'right_rocker.glb': ['right_rocker'],
        'left_bogie.glb': ['left_bogie'],
        'right_bogie.glb': ['right_bogie'],
        'left_front_wheel.glb': ['left_front_wheel_rim', 'left_front_wheel_hub'] + [f'left_front_wheel_tread_{i}' for i in range(16)],
        'left_middle_wheel.glb': ['left_middle_wheel_rim', 'left_middle_wheel_hub'] + [f'left_middle_wheel_tread_{i}' for i in range(16)],
        'left_rear_wheel.glb': ['left_rear_wheel_rim', 'left_rear_wheel_hub'] + [f'left_rear_wheel_tread_{i}' for i in range(16)],
        'right_front_wheel.glb': ['right_front_wheel_rim', 'right_front_wheel_hub'] + [f'right_front_wheel_tread_{i}' for i in range(16)],
        'right_middle_wheel.glb': ['right_middle_wheel_rim', 'right_middle_wheel_hub'] + [f'right_middle_wheel_tread_{i}' for i in range(16)],
        'right_rear_wheel.glb': ['right_rear_wheel_rim', 'right_rear_wheel_hub'] + [f'right_rear_wheel_tread_{i}' for i in range(16)],
    }
    
    print("=========================================================================")
    print(f" EXPORTING 12 CLEAN GLB MESHES FROM: {blend_path}")
    print("=========================================================================")
    
    # Deselect all
    bpy.ops.object.select_all(action='DESELECT')
    
    for glb_filename, obj_names in mesh_export_groups.items():
        export_filepath = os.path.join(output_dir, glb_filename)
        
        # Select matching objects
        selected_count = 0
        for obj_name in obj_names:
            if obj_name in bpy.data.objects:
                obj = bpy.data.objects[obj_name]
                obj.select_set(True)
                selected_count += 1
                
        if selected_count > 0:
            print(f"Exporting {glb_filename} ({selected_count} objects) -> {export_filepath}")
            bpy.ops.export_scene.gltf(
                filepath=export_filepath,
                export_format='GLB',
                use_selection=True,
                export_apply=True
            )
            # Deselect after export
            bpy.ops.object.select_all(action='DESELECT')
        else:
            print(f"⚠️ WARNING: No matching objects found for {glb_filename}")

    print("\nGLB Mesh Export Complete.")

if __name__ == '__main__':
    export_lunabot_meshes(
        '/home/dk05/Desktop/SMART_HORIZON/LUNA_PRO/LunaBot_Sensor_Equipped_modified_new_final_123.blend',
        '/home/dk05/Desktop/SMART_HORIZON/LUNA_PRO/environment/models/lunabot/meshes'
    )
