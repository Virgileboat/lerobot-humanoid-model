from pathlib import Path
import re
import mujoco
from mj2mjlab import make_mjlab_xml
from remove_useless_meshe import simplify_mjcf

def load_with_assets(xml_path: str, meshes_root: str) -> mujoco.MjModel:
    xml_path = Path(xml_path).resolve()
    meshes_root = Path(meshes_root).resolve()

    assets = {}
    for p in meshes_root.rglob("*"):
        if p.suffix.lower() in {".stl", ".obj", ".dae", ".ply", ".png", ".jpg", ".jpeg"}:
            # Key must match file="..." in the XML
            assets[p.name] = p.read_bytes()

    return mujoco.MjModel.from_xml_path(str(xml_path), assets=assets)


# ---- paths you control ----
URDF_PATH   = "robot.urdf"
MESH_ROOT   = "assets"        # where your STL files live
# ---------------------------


# 1) Load URDF with VFS assets
m = load_with_assets(URDF_PATH, MESH_ROOT)

# 2) Export compiled MJCF
robot_xml = "robot.xml"
mujoco.mj_saveLastXML(str(robot_xml), m)

# 3) Patch mesh paths to assets/<basename>
txt = robot_xml.read_text()
def repl(m):
    f = m.group(1)
    if "/" in f or "\\" in f:   # already has a path → leave it
        return m.group(0)
    return f'file="assets/{f}"'

txt2 = re.sub(r'file="([^"]+)"', repl, txt)
robot_xml.write_text(txt2)

print(f"[OK] Wrote {robot_xml} with mesh paths rewritten to assets/<file>")


# 4) Simplify MJCF (remove useless meshes, add actuators, etc)
robot_simplified_xml =  "robot_simplified.xml"
simplify_mjcf(robot_xml, robot_simplified_xml)

# 5) Final MJLab conversion (add IMU, foot contacts, sensors, etc)
robot_mjlab_xml = "robot_mjlab.xml"
make_mjlab_xml(robot_simplified_xml, robot_mjlab_xml)