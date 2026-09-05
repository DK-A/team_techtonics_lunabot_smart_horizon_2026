import bpy
import os
import mathutils

print("=========================================================================")
print(" AUDITING BLENDER OBJECTS & MESH BOUNDS FOR ACCIDENTAL PLANES/GROUNDS")
print(" Master Source: LunaBot_Sensor_Equipped_final.blend")
print("=========================================================================")

# 1. Audit all Blender objects for any ground, plane, floor, or oversized mesh
large_or_suspicious_objs = []

for obj in bpy.data.objects:
    nl = obj.name.lower()
    if any(k in nl for k in ["plane", "ground", "floor", "shadow", "reference", "terrain", "grid", "cube"]):
        matrix = obj.matrix_world
        corners = [matrix @ mathutils.Vector(c) for c in obj.bound_box] if obj.type == 'MESH' else []
        min_x = min(c.x for c in corners) if corners else 0
        max_x = max(c.x for c in corners) if corners else 0
        min_y = min(c.y for c in corners) if corners else 0
        max_y = max(c.y for c in corners) if corners else 0
        min_z = min(c.z for c in corners) if corners else 0
        max_z = max(c.z for c in corners) if corners else 0
        size_x = max_x - min_x
        size_y = max_y - min_y
        size_z = max_z - min_z
        
        print(f"SUSPICIOUS OBJECT: '{obj.name:35s}' | Type: {obj.type} | Size: ({size_x:.3f}m, {size_y:.3f}m, {size_z:.3f}m)")
        if size_x > 2.0 or size_y > 2.0 or size_z > 2.0:
            large_or_suspicious_objs.append(obj)

if not large_or_suspicious_objs:
    print("NO environment-sized or oversized plane objects found in Blender Master!")

# 2. Check GLB mesh files
mesh_dir = "/home/dk05/Desktop/SMART_HORIZON/LUNA_PRO/environment/models/lunabot/meshes"
print(f"\nChecking GLB mesh files in {mesh_dir}:")
for fname in sorted(os.listdir(mesh_dir)):
    if fname.endswith(".glb"):
        fpath = os.path.join(mesh_dir, fname)
        size_mb = os.path.getsize(fpath) / (1024 * 1024)
        print(f"  - Mesh: {fname:30s} | Size: {size_mb:.3f} MB")

print("\nAudit Complete.")
