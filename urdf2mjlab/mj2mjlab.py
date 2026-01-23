from pathlib import Path
import xml.etree.ElementTree as ET


# -------------------------------------------------
# CONFIG
# -------------------------------------------------

BASE_BODY_NAME = "base"

RIGHT_FOOT_BODY = "foot_subassembly"
LEFT_FOOT_BODY  = "foot_subassembly_2"

IMU_SITE_NAME = "imu_site"

RIGHT_FOOT_CONTACT = "foot_contact_right"
LEFT_FOOT_CONTACT  = "foot_contact_left"

FOOT_BOX_SIZE = "0.03 0.02 0.005"   # x y z half-sizes
FOOT_BOX_POS  = "0 0 -0.01"         # local to foot body

# -------------------------------------------------
# HELPERS
# -------------------------------------------------

def indent(elem, level=0):
    i = "\n" + level * "  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + "  "
        for e in elem:
            indent(e, level + 1)
        if not e.tail or not e.tail.strip():
            e.tail = i
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = i


def find_body(root, name):
    for b in root.findall(".//body"):
        if b.attrib.get("name") == name:
            return b
    raise RuntimeError(f"Body not found: {name}")


# -------------------------------------------------
# MAIN TRANSFORM
# -------------------------------------------------

def make_mjlab_xml(in_path: Path, out_path: Path):
    tree = ET.parse(in_path)
    root = tree.getroot()

    worldbody = root.find("worldbody")
    if worldbody is None:
        raise RuntimeError("No <worldbody> found")

    # -------------------------------------------------
    # 1) Add IMU site to base
    # -------------------------------------------------

    base = find_body(root, BASE_BODY_NAME)

    if base.find(f"./site[@name='{IMU_SITE_NAME}']") is None:
        ET.SubElement(
            base,
            "site",
            {
                "name": IMU_SITE_NAME,
                "pos": "0 0 0",
                "size": "0.01",
                "rgba": "0 1 0 1",
            },
        )

    # -------------------------------------------------
    # 2) Add foot contact geoms
    # -------------------------------------------------

    def add_foot_contact(body_name, geom_name):
        body = find_body(root, body_name)

        if body.find(f"./geom[@name='{geom_name}']") is not None:
            return

        ET.SubElement(
            body,
            "geom",
            {
                "name": geom_name,
                "type": "box",
                "size": FOOT_BOX_SIZE,
                "pos": FOOT_BOX_POS,
                "rgba": "0 0 1 0.3",
                "contype": "1",
                "conaffinity": "1",
            },
        )

    add_foot_contact(RIGHT_FOOT_BODY, RIGHT_FOOT_CONTACT)
    add_foot_contact(LEFT_FOOT_BODY,  LEFT_FOOT_CONTACT)

    # -------------------------------------------------
    # 3) Auto-generate sensors
    # -------------------------------------------------

    old_sensor = root.find("sensor")
    if old_sensor is not None:
        root.remove(old_sensor)

    sensor = ET.SubElement(root, "sensor")

    # ---- joints
    joints = root.findall(".//joint")
    actuators = root.find("actuator")

    actuator_names = {}
    if actuators is not None:
        for a in actuators:
            if "joint" in a.attrib:
                actuator_names[a.attrib["joint"]] = a.attrib["name"]

    for j in joints:
        jname = j.attrib.get("name")
        jtype = j.attrib.get("type", "hinge")

        if jname == "root" or jtype in {"free", "ball"}:
            continue

        ET.SubElement(sensor, "jointpos", {
            "name": f"q_{jname}",
            "joint": jname,
        })

        ET.SubElement(sensor, "jointvel", {
            "name": f"qd_{jname}",
            "joint": jname,
        })

        # actuator force if exists
        if jname in actuator_names:
            ET.SubElement(sensor, "actuatorfrc", {
                "name": f"tau_{jname}",
                "actuator": actuator_names[jname],
            })

    # ---- IMU
    ET.SubElement(sensor, "gyro", {
        "name": "gyro",
        "site": IMU_SITE_NAME,
    })

    ET.SubElement(sensor, "accelerometer", {
        "name": "accel",
        "site": IMU_SITE_NAME,
    })

    ET.SubElement(sensor, "framequat", {
        "name": "base_quat",
        "body": BASE_BODY_NAME,
    })

    # ---- foot contacts
    ET.SubElement(sensor, "touch", {
        "name": "touch_right",
        "geom": RIGHT_FOOT_CONTACT,
    })

    ET.SubElement(sensor, "touch", {
        "name": "touch_left",
        "geom": LEFT_FOOT_CONTACT,
    })

    # -------------------------------------------------
    # Save
    # -------------------------------------------------

    indent(root)
    tree.write(out_path, encoding="utf-8", xml_declaration=True)

    print(f"[OK] Wrote MJLab robot → {out_path}")


# -------------------------------------------------
# ENTRY POINT
# -------------------------------------------------

if __name__ == "__main__":
    IN  = Path("robot.xml")
    OUT = Path("robot_mjlab.xml")

    make_mjlab_xml(IN, OUT)
