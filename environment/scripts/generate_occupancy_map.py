import os
import numpy as np
from PIL import Image

print("=========================================================================")
print(" GENERATING ROS 2 / NAV2 COMPATIBLE OCCUPANCY GRID MAP")
print("=========================================================================")

workspace_root = os.environ.get('LUNA_PRO_ROOT', None)
if not workspace_root or not os.path.exists(workspace_root):
    candidates = [
        os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")),
        "/home/dk05/Desktop/SMART_HORIZON/LUNA_PRO",
        os.getcwd()
    ]
    for c in candidates:
        if os.path.exists(os.path.join(c, "environment")):
            workspace_root = c
            break
if not workspace_root:
    workspace_root = "/home/dk05/Desktop/SMART_HORIZON/LUNA_PRO"

maps_dir = os.path.join(workspace_root, "environment", "maps")

# Map metadata
res = 0.05  # meters per pixel
width_m = 50.0
height_m = 50.0
width_px = int(width_m / res)    # 1000
height_px = int(height_m / res)  # 1000

origin_x = -25.0
origin_y = -25.0

# Create map array: 254 = free (white), 0 = occupied (black), 205 = unknown
grid = np.full((height_px, width_px), 254, dtype=np.uint8)

def world_to_pixel(wx, wy):
    px = int((wx - origin_x) / res)
    py = int((height_m - (wy - origin_y)) / res)  # Image Y is inverted
    return px, py

# 1. Burn NO-GO Zone 1 (Northern Boulder Field: X=6.0, Y=5.0, R=2.5m)
cx, cy = world_to_pixel(6.0, 5.0)
r_px = int(2.5 / res)
y_indices, x_indices = np.ogrid[:height_px, :width_px]
dist_from_c1 = np.sqrt((x_indices - cx)**2 + (y_indices - cy)**2)
grid[dist_from_c1 <= r_px] = 0

# 2. Burn NO-GO Zone 2 (Southern Crater Ridge: X=-5.0, Y=-6.0, R=3.0m)
cx, cy = world_to_pixel(-5.0, -6.0)
r_px = int(3.0 / res)
dist_from_c2 = np.sqrt((x_indices - cx)**2 + (y_indices - cy)**2)
grid[dist_from_c2 <= r_px] = 0

# 3. Burn NO-GO Zone 3 (Habitat Structure Perimeter: X=-8.0, Y=8.0, 4m x 4m)
px1, py1 = world_to_pixel(-10.0, 10.0)
px2, py2 = world_to_pixel(-6.0, 6.0)
grid[min(py1, py2):max(py1, py2), min(px1, px2):max(px1, px2)] = 0

# 4. Save PGM file
pgm_path = os.path.join(maps_dir, "lunar_habitat_map.pgm")
img = Image.fromarray(grid)
img.save(pgm_path)
print(f"Saved Occupancy Map Image: {pgm_path} ({width_px}x{height_px} px)")

# 5. Save YAML metadata
yaml_content = f"""image: lunar_habitat_map.pgm
mode: trinary
resolution: {res}
origin: [{origin_x}, {origin_y}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
"""

yaml_path = os.path.join(maps_dir, "lunar_habitat_map.yaml")
with open(yaml_path, "w") as f:
    f.write(yaml_content)

print(f"Saved Nav2 Map YAML: {yaml_path}")
print("Map Generation Complete.")
