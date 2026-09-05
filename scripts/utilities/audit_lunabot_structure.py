import xml.etree.ElementTree as ET
import os

def audit_structure():
    print("=========================================================================")
    print(" AUTOMATED LUNABOT MECHANICAL STRUCTURE AUDIT (STEP 15 CHECKLIST)")
    print("=========================================================================")

    sdf_path = "/home/dk05/Desktop/SMART_HORIZON/LUNA_PRO/environment/models/lunabot/model.sdf"
    if not os.path.exists(sdf_path):
        print("❌ FAIL: model.sdf does not exist!")
        return False

    tree = ET.parse(sdf_path)
    root = tree.getroot()
    model = root.find("model")

    links = [l.attrib["name"] for l in model.findall("link")]
    joints = [j.attrib["name"] for j in model.findall("joint")]

    checks = []

    # 1. Links count
    wheels = [l for l in links if "wheel" in l]
    rockers = [l for l in links if "rocker" in l]
    bogies = [l for l in links if "bogie" in l]
    chassis = [l for l in links if l in ["base_link", "chassis"]]

    checks.append((len(wheels) == 6, "exactly 6 wheel links exist"))
    checks.append((len(rockers) == 2, "exactly 2 rocker links exist"))
    checks.append((len(bogies) == 2, "exactly 2 bogie links exist"))
    checks.append((len(chassis) == 1, "one chassis/base link exists"))

    # 2. Wheel joints
    wheel_joints = [j for j in model.findall("joint") if "wheel" in j.attrib["name"]]
    checks.append((len(wheel_joints) == 6, "every wheel has exactly one wheel joint"))

    for j in wheel_joints:
        jname = j.attrib["name"]
        parent = j.find("parent").text
        child = j.find("child").text
        axis = j.find("axis/xyz").text.strip()

        if "front" in jname:
            checks.append(("rocker" in parent, f"{jname} parent is rocker ({parent})"))
        else:
            checks.append(("bogie" in parent, f"{jname} parent is bogie ({parent})"))

        checks.append((child in wheels, f"{jname} child is wheel link ({child})"))
        checks.append((axis == "0 1 0", f"{jname} axis is (0 1 0)"))

    # 3. Rocker / Bogie joints
    rocker_joints = [j for j in model.findall("joint") if "rocker" in j.attrib["name"]]
    bogie_joints = [j for j in model.findall("joint") if "bogie" in j.attrib["name"]]

    for j in rocker_joints:
        parent = j.find("parent").text
        checks.append((parent in chassis, "rocker joints connect chassis to rocker"))

    for j in bogie_joints:
        parent = j.find("parent").text
        checks.append((parent in rockers, "bogie joints connect rocker to bogie"))

    # 4. No floating mechanical links
    joint_children = [j.find("child").text for j in model.findall("joint")]
    floating = [l for l in links if l not in chassis and l not in joint_children]
    checks.append((len(floating) == 0, "no floating mechanical links"))

    # 5. Mass & Inertia
    all_masses_valid = True
    for l in model.findall("link"):
        inertial = l.find("inertial")
        if inertial is not None:
            mass_elem = inertial.find("mass")
            if mass_elem is not None:
                m = float(mass_elem.text)
                if m <= 0:
                    all_masses_valid = False

    checks.append((all_masses_valid, "positive mass across all links"))
    checks.append((len(links) == len(set(links)), "no duplicate links"))
    checks.append((len(joints) == len(set(joints)), "no duplicate joints"))

    pass_count = 0
    for passed, msg in checks:
        status = "PASS" if passed else "FAIL"
        if passed:
            pass_count += 1
        print(f"[{'X' if passed else ' '}] {msg:64s} -> {status}")

    print("\n=========================================================================")
    print(f" AUDIT SUMMARY: {pass_count} / {len(checks)} CHECKS PASSED")
    print("=========================================================================")

    return pass_count == len(checks)

if __name__ == '__main__':
    audit_structure()
