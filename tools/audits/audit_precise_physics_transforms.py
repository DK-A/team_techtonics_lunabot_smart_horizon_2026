import bpy
import mathutils

print("=========================================================================")
print(" RIGOROUS GEOMETRY & PHYSICAL TRANSFORM AUDIT")
print(" Master Source: LunaBot_Sensor_Equipped_final.blend")
print("=========================================================================")

# Measure exact wheel radius and wheel width in Blender Master
wheel_objs = [o for o in bpy.data.objects if o.type == 'MESH' and ("left_front" in o.name or "lf_" in o.name)]
min_x = min_y = min_z = float('inf')
max_x = max_y = max_z = float('-inf')

for o in wheel_objs:
    matrix = o.matrix_world
    for corner in o.bound_box:
        wc = matrix @ mathutils.Vector(corner)
        min_x = min(min_x, wc.x)
        max_x = max(max_x, wc.x)
        min_y = min(min_y, wc.y)
        max_y = max(max_y, wc.y)
        min_z = min(min_z, wc.z)
        max_z = max(max_z, wc.z)

wheel_radius = (max_z - min_z) / 2.0
wheel_width = max_y - min_y
print(f"Measured Wheel Radius r = {wheel_radius:.4f} m (Diameter: {wheel_radius*2:.4f} m)")
print(f"Measured Wheel Width    l = {wheel_width:.4f} m")

# Calculate ideal angular velocities omega = v / r
for v in [0.02, 0.05, 0.08, 0.10]:
    omega = v / wheel_radius
    print(f"  Speed v = {v:.2f} m/s --> Ideal Wheel Angular Velocity omega = {omega:.4f} rad/s")

# Audit chassis belly bottom in Blender Master
chassis_objs = [o for o in bpy.data.objects if o.type == 'MESH' and any(k in o.name for k in ["chassis", "deck", "lidar", "cam", "imu", "battery", "sensor", "mount", "saddle"]) and not any(w in o.name for w in ["wheel", "rocker", "bogie"])]
c_min_z = min(min((o.matrix_world @ mathutils.Vector(c)).z for c in o.bound_box) for o in chassis_objs)
c_max_z = max(max((o.matrix_world @ mathutils.Vector(c)).z for c in o.bound_box) for o in chassis_objs)
print(f"\nChassis Bounds Z in Blender Master: [{c_min_z:.4f} m, {c_max_z:.4f} m]")
print(f"Chassis Belly Bottom Z: {c_min_z:.4f} m (Clearance above ground Z=0.0m: {c_min_z:.4f} m)")
