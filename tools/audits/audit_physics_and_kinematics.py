import xml.etree.ElementTree as ET
import math

print("=========================================================================")
print(" RIGOROUS LUNABOT 6-WHEEL KINEMATICS, GEOMETRY & PHYSICS AUDIT")
print("=========================================================================")

sdf_path = "/home/dk05/Desktop/SMART_HORIZON/LUNA_PRO/environment/models/lunabot/model.sdf"
urdf_path = "/home/dk05/Desktop/SMART_HORIZON/LUNA_PRO/environment/models/lunabot/lunabot.urdf"
world_path = "/home/dk05/Desktop/SMART_HORIZON/LUNA_PRO/environment/worlds/moon.sdf"

sdf_tree = ET.parse(sdf_path)
sdf_root = sdf_tree.getroot()
model = sdf_root.find('model')

print(f"Model Name: {model.attrib.get('name')}")
model_pose = model.find('pose')
print(f"Model Pose: {model_pose.text if model_pose is not None else '0 0 0 0 0 0'}")

links = {}
for link in model.findall('link'):
    lname = link.attrib['name']
    pose_el = link.find('pose')
    pose = pose_el.text if pose_el is not None else "0 0 0 0 0 0"
    
    inertial = link.find('inertial')
    mass = "0"
    com = "0 0 0 0 0 0"
    ixx, iyy, izz = "0", "0", "0"
    if inertial is not None:
        mass_el = inertial.find('mass')
        if mass_el is not None: mass = mass_el.text
        ipose = inertial.find('pose')
        if ipose is not None: com = ipose.text
        iner = inertial.find('inertia')
        if iner is not None:
            ixx = iner.findtext('ixx', '0')
            iyy = iner.findtext('iyy', '0')
            izz = iner.findtext('izz', '0')
            
    col = link.find('collision')
    col_pose = "0 0 0 0 0 0"
    col_geom = "None"
    if col is not None:
        cpose = col.find('pose')
        if cpose is not None: col_pose = cpose.text
        geom = col.find('geometry')
        if geom is not None:
            cyl = geom.find('cylinder')
            box = geom.find('box')
            if cyl is not None:
                col_geom = f"Cylinder(r={cyl.findtext('radius')}, l={cyl.findtext('length')})"
            elif box is not None:
                col_geom = f"Box(s={box.findtext('size')})"
                
    vis = link.find('visual')
    vis_geom = "None"
    if vis is not None:
        vgeom = vis.find('geometry')
        if vgeom is not None:
            mesh = vgeom.find('mesh')
            if mesh is not None:
                vis_geom = f"Mesh({mesh.findtext('uri')})"
                
    links[lname] = {
        'pose': pose,
        'mass': mass,
        'com': com,
        'inertia': (ixx, iyy, izz),
        'col_pose': col_pose,
        'col_geom': col_geom,
        'vis_geom': vis_geom
    }

joints = {}
for joint in model.findall('joint'):
    jname = joint.attrib['name']
    jtype = joint.attrib.get('type', 'fixed')
    parent = joint.findtext('parent')
    child = joint.findtext('child')
    rel_pose = joint.findtext('pose', '0 0 0 0 0 0')
    axis = "1 0 0"
    lower, upper = "None", "None"
    damping, friction = "0", "0"
    axis_el = joint.find('axis')
    if axis_el is not None:
        axis = axis_el.findtext('xyz', '1 0 0')
        lim = axis_el.find('limit')
        if lim is not None:
            lower = lim.findtext('lower', 'None')
            upper = lim.findtext('upper', 'None')
        dyn = axis_el.find('dynamics')
        if dyn is not None:
            damping = dyn.findtext('damping', '0')
            friction = dyn.findtext('friction', '0')
            
    joints[jname] = {
        'type': jtype,
        'parent': parent,
        'child': child,
        'rel_pose': rel_pose,
        'axis': axis,
        'limits': (lower, upper),
        'dynamics': (damping, friction)
    }

print("\n=========================================================================")
print(" LINK & INERTIAL AUDIT")
print("=========================================================================")
for lname, ldata in sorted(links.items()):
    print(f"Link: {lname:28s} | Mass: {ldata['mass']:>5s}kg | COM: {ldata['com']:20s} | Inertia: (Ixx={ldata['inertia'][0]}, Iyy={ldata['inertia'][1]}, Izz={ldata['inertia'][2]})")

print("\n=========================================================================")
print(" 6-WHEEL GEOMETRY AUDIT TABLE")
print("=========================================================================")
wheels = ['left_front_wheel', 'left_middle_wheel', 'left_rear_wheel', 'right_front_wheel', 'right_middle_wheel', 'right_rear_wheel']
print(f"{'Wheel Name':20s} | {'Parent':12s} | {'Link Pose (X Y Z R P Y)':32s} | {'Collision Shape':30s} | {'Collision Pose':22s}")
print("-" * 125)
for w in wheels:
    if w in links:
        parent = [j['parent'] for jname, j in joints.items() if j['child'] == w][0]
        l = links[w]
        print(f"{w:20s} | {parent:12s} | {l['pose']:32s} | {l['col_geom']:30s} | {l['col_pose']:22s}")

print("\n=========================================================================")
print(" JOINTS & DYNAMICS AUDIT TABLE")
print("=========================================================================")
print(f"{'Joint Name':26s} | {'Type':10s} | {'Parent':12s} | {'Child':20s} | {'Axis':8s} | {'Limits (lower, upper)':22s} | {'Damping/Friction':16s}")
print("-" * 125)
for jname, j in sorted(joints.items()):
    print(f"{jname:26s} | {j['type']:10s} | {j['parent']:12s} | {j['child']:20s} | {j['axis']:8s} | {str(j['limits']):22s} | Damp={j['dynamics'][0]}/Fric={j['dynamics'][1]}")

print("\nAudit Script Complete.")
