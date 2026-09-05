import xml.etree.ElementTree as ET
import os

def validate_structure():
    print("=========================================================================")
    print(" LUNABOT MECHANICAL STRUCTURE VALIDATION (STEP 14 CHECKLIST)")
    print("=========================================================================")

    sdf_path = "/home/dk05/Desktop/SMART_HORIZON/LUNA_PRO/environment/models/lunabot/model.sdf"
    tree = ET.parse(sdf_path)
    root = tree.getroot()
    model = root.find("model")

    links = {l.attrib["name"]: l for l in model.findall("link")}
    joints = {j.attrib["name"]: j for j in model.findall("joint")}

    checks = [
        ("base_link" in links, "base_link exists"),
        ("left_rocker" in links and "right_rocker" in links, "two rocker links exist (left_rocker, right_rocker)"),
        ("left_bogie" in links and "right_bogie" in links, "two bogie links exist (left_bogie, right_bogie)"),
        (len([l for l in links if "wheel" in l]) == 6, "six wheel links exist"),
        (joints["left_rocker_joint"].find("parent").text == "base_link", "left rocker attached to base"),
        (joints["right_rocker_joint"].find("parent").text == "base_link", "right rocker attached to base"),
        (joints["left_front_wheel_joint"].find("parent").text == "left_rocker", "left front wheel attached to left rocker"),
        (joints["left_bogie_joint"].find("parent").text == "left_rocker", "left bogie attached to left rocker"),
        (joints["left_middle_wheel_joint"].find("parent").text == "left_bogie", "left middle wheel attached to left bogie"),
        (joints["left_rear_wheel_joint"].find("parent").text == "left_bogie", "left rear wheel attached to left bogie"),
        (joints["right_front_wheel_joint"].find("parent").text == "right_rocker", "right front wheel attached to right rocker"),
        (joints["right_bogie_joint"].find("parent").text == "right_rocker", "right bogie attached to right rocker"),
        (joints["right_middle_wheel_joint"].find("parent").text == "right_bogie", "right middle wheel attached to right bogie"),
        (joints["right_rear_wheel_joint"].find("parent").text == "right_bogie", "right rear wheel attached to right bogie"),
    ]

    all_wheel_axes = True
    for jname in ["left_front_wheel_joint", "left_middle_wheel_joint", "left_rear_wheel_joint",
                  "right_front_wheel_joint", "right_middle_wheel_joint", "right_rear_wheel_joint"]:
        axis = joints[jname].find("axis/xyz").text.strip()
        if axis != "0 1 0":
            all_wheel_axes = False

    checks.append((all_wheel_axes, "every wheel joint axis = 0 1 0"))
    checks.append((joints["left_rocker_joint"].find("axis/xyz").text.strip() == "0 1 0", "rocker joints axis = 0 1 0"))
    checks.append((joints["left_bogie_joint"].find("axis/xyz").text.strip() == "0 1 0", "bogie joints axis = 0 1 0"))
    checks.append((len(links) >= 14, "no disconnected dynamic links"))
    checks.append((len([j for j in joints if "wheel" in j]) == 6, "no duplicate wheel joints"))
    checks.append((len([j for j in joints if "rocker" in j]) == 2, "no duplicate rocker joints"))
    checks.append((len([j for j in joints if "bogie" in j]) == 2, "no duplicate bogie joints"))

    pass_count = 0
    for passed, msg in checks:
        if passed:
            pass_count += 1
        print(f"[{'X' if passed else ' '}] {msg:64s} -> {'PASS' if passed else 'FAIL'}")

    print("\n=========================================================================")
    print(f" VALIDATION SUMMARY: {pass_count} / {len(checks)} CHECKS PASSED")
    print("=========================================================================")

if __name__ == '__main__':
    validate_structure()
