from pathlib import Path
import xml.etree.ElementTree as ET


# -------------------------------
# CONFIG
# -------------------------------

WHITELIST_MESHES = {
    # torso / base
    "torso22", "torso12", "torso_meshe", "torso_pperbody_part",

    # hip / thigh
    "hipz12", "hipz22", "hipxy",
    "knee_rod22", "knee_rod12",
    "femur_22", "femur_12", "hat_femur",

    # shin / ankle
    "tibias22", "tibias12",
    "actuation_ankle",
    "part_3",

    # foot
    "foot", "hat_small",

    # motors
    "robstrideo0", "robstride02", "robstrideo3", "rmdx43",

    # joints
    "ujoint", "spacer_ujoint",

    # symmetry
    "hipz22_sym", "hipz12_sym", "hipxy_sym",
    "knee_rod22_sym", "knee_rod12_sym",
    "femur_22_sym", "femur_12_sym",
    "tibias22_sym", "tibias12_sym",

    # upper body
    "shoulder1", "shoulder2",
    "shoulder1_part", "shoulder2_part",
    "shoulder3_part", "forarm_part",
    "bearing_holder_upperbody",
}

ADD_CONTYPE_TO = {
    "foot",
    "femur_22", "femur_12",
    "femur_22_sym", "femur_12_sym",
    "forarm_part",
}


# -------------------------------
# ACTUATOR AUTOGEN CONFIG
# -------------------------------

DEFAULT_KP = 50
DEFAULT_KV = 2
DEFAULT_GEAR = 1
DEFAULT_CTRLRANGE = "-1 1"

# Optional per-joint gain overrides
JOINT_GAIN_OVERRIDES = {
    "hipy_right":  {"kp": 100, "kv": 2},
    "knee_right":  {"kp": 100, "kv": 2},
    "ankley_right":{"kp": 40,  "kv": 2},
    "anklex_right":{"kp": 10,  "kv": 2},

    "hipy_left":   {"kp": 100, "kv": 2},
    "knee_left":   {"kp": 100, "kv": 2},
    "ankley_left": {"kp": 40,  "kv": 2},
    "anklex_left": {"kp": 10,  "kv": 2},
}

EXCLUDED_JOINTS = {
    "root",   # free joint
}


def is_actuated_joint(joint: ET.Element) -> bool:
    name = joint.attrib.get("name", "")
    jtype = joint.attrib.get("type", "hinge")  # default in MJCF

    if name in EXCLUDED_JOINTS:
        return False

    if jtype in {"free", "ball"}:
        return False

    # hinge / slide are actuated
    return True

# -------------------------------
# HELPERS
# -------------------------------

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


# -------------------------------
# MAIN TRANSFORM
# -------------------------------

def simplify_mjcf(in_path: Path, out_path: Path):
    tree = ET.parse(in_path)
    root = tree.getroot()

    # ---- ASSET: keep only whitelisted meshes ----
    asset = root.find("asset")
    if asset is not None:
        for mesh in list(asset.findall("mesh")):
            name = mesh.attrib.get("name", "")
            if name not in WHITELIST_MESHES:
                asset.remove(mesh)

    # ---- WORLD: remove geoms using non-whitelisted meshes ----
    for geom in root.findall(".//geom"):
        mesh = geom.attrib.get("mesh")
        if mesh and mesh not in WHITELIST_MESHES:
            parent = geom.getparent() if hasattr(geom, "getparent") else None

            # xml.etree has no getparent(), so brute force:
            for p in root.iter():
                if geom in list(p):
                    p.remove(geom)
                    break

        # add contype / conaffinity if needed
        if mesh in ADD_CONTYPE_TO:
            geom.attrib.setdefault("contype", "1")
            geom.attrib.setdefault("conaffinity", "1")

    # ---- Wrap everything in a floating base ----
    worldbody = root.find("worldbody")

    base = ET.Element("body", {"name": "base", "pos": "0 0 0.72"})
    root_joint = ET.SubElement(base, "joint", {"name": "root", "type": "free"})

    # move all current worldbody children under base
    for child in list(worldbody):
        worldbody.remove(child)
        base.append(child)

    worldbody.append(base)

    # ---- ACTUATORS (one per joint, no passive DOF) ----
    old = root.find("actuator")
    if old is not None:
        root.remove(old)

    actuator = ET.SubElement(root, "actuator")

    joints = root.findall(".//joint")

    for j in joints:
        if not is_actuated_joint(j):
            continue

        jname = j.attrib["name"]
        aname = f"m_{jname}"

        gains = JOINT_GAIN_OVERRIDES.get(jname, {})
        kp = gains.get("kp", DEFAULT_KP)
        kv = gains.get("kv", DEFAULT_KV)

        ET.SubElement(
            actuator,
            "position",
            {
                "name": aname,
                "joint": jname,
                "kp": str(kp),
                "kv": str(kv),
                "gear": str(DEFAULT_GEAR),
                "ctrlrange": DEFAULT_CTRLRANGE,
            },
        )

    print(f"[OK] Created {len(actuator)} actuators (1 per joint)")

    # ---- Pretty print + save ----
    indent(root)
    tree.write(out_path, encoding="utf-8", xml_declaration=True)
    print(f"[OK] Wrote simplified MJCF → {out_path}")


# -------------------------------
# ENTRY POINT
# -------------------------------

if __name__ == "__main__":
    IN  = Path("robot.xml")
    OUT = Path("robot_simplified.xml")

    simplify_mjcf(IN, OUT)