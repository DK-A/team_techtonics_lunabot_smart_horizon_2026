import xml.etree.ElementTree as ET

def audit_symmetry(sdf_path):
    tree = ET.parse(sdf_path)
    root = tree.getroot()
    model = root.find('model')
    
    links = {}
    for link in model.findall('link'):
        name = link.attrib['name']
        pose = link.find('pose')
        pose_str = pose.text if pose is not None else "0 0 0 0 0 0"
        links[name] = [float(x) for x in pose_str.split()]
        
    joints = {}
    for joint in model.findall('joint'):
        name = joint.attrib['name']
        parent = joint.find('parent').text
        child = joint.find('child').text
        pose = joint.find('pose')
        pose_str = pose.text if pose is not None else "0 0 0 0 0 0"
        joints[name] = {
            'parent': parent,
            'child': child,
            'pose': [float(x) for x in pose_str.split()]
        }

    print("=========================================================================")
    print(" NUMERICAL SYMMETRY AUDIT (LEFT VS RIGHT)")
    print("=========================================================================")
    
    pairs = [
        ('left_rocker', 'right_rocker'),
        ('left_bogie', 'right_bogie'),
        ('left_front_wheel', 'right_front_wheel'),
        ('left_middle_wheel', 'right_middle_wheel'),
        ('left_rear_wheel', 'right_rear_wheel')
    ]
    
    for l_name, r_name in pairs:
        l_pose = links[l_name]
        r_pose = links[r_name]
        print(f"\n--- LINK PAIR: {l_name} vs {r_name} ---")
        print(f"   Left  Pose: {l_pose}")
        print(f"   Right Pose: {r_pose}")
        
        # Check symmetry: X_l == X_r, Y_l == -Y_r, Z_l == Z_r
        x_diff = abs(l_pose[0] - r_pose[0])
        y_diff = abs(l_pose[1] + r_pose[1])
        z_diff = abs(l_pose[2] - r_pose[2])
        print(f"   Symmetry Delta -> X_diff: {x_diff:.4f}, Y_sum: {y_diff:.4f}, Z_diff: {z_diff:.4f}")
        if x_diff > 1e-3 or y_diff > 1e-3 or z_diff > 1e-3:
            print(f"   ⚠️ WARNING: ASYMMETRY DETECTED IN {l_name} / {r_name}!")
        else:
            print(f"   ✅ 100% PERFECT MIRRORED SYMMETRY")

if __name__ == '__main__':
    audit_symmetry('/home/dk05/Desktop/SMART_HORIZON/LUNA_PRO/environment/models/lunabot/model.sdf')
