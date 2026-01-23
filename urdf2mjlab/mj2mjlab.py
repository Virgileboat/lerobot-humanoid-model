import xml.etree.ElementTree as ET
from copy import deepcopy


def index_robot_bodies_by_meshset(robot_root):
    """
    Build a dict:
      frozenset(mesh_names) -> body_element
    Only uses visual geoms (i.e. geoms with mesh=...).
    """
    meshmap = {}

    for body in robot_root.iter("body"):
        meshes = []
        for g in body.findall("geom"):
            mesh = g.get("mesh")
            if mesh:
                meshes.append(mesh)

        if not meshes:
            continue

        key = frozenset(meshes)
        if key in meshmap:
            raise RuntimeError(
                f"[ERR] Non-unique mesh signature for bodies:\n"
                f"  {meshmap[key].get('name')} and {body.get('name')}\n"
                f"  meshes = {sorted(key)}"
            )

        meshmap[key] = body

    return meshmap





def build_parent_map(root):
    parent_map = {}
    for parent in root.iter("body"):
        for child in parent.findall("body"):
            parent_map[child] = parent
    return parent_map


def index_robot_bodies_by_signature(robot_root):
    parent_map = build_parent_map(robot_root)
    sigmap = {}

    for body in robot_root.iter("body"):
        own = [g.get("mesh") for g in body.findall("geom") if g.get("mesh")]
        if not own:
            continue

        parent = parent_map.get(body)
        parent_meshes = []
        if parent is not None:
            parent_meshes = [g.get("mesh") for g in parent.findall("geom")
                             if g.get("mesh")]

        key = (frozenset(own), frozenset(parent_meshes))

        if key in sigmap:
            b0 = sigmap[key]
            raise RuntimeError(
                f"[ERR] Still non-unique body signature:\n"
                f"  {b0.get('name')} and {body.get('name')}\n"
                f"  own meshes    = {sorted(key[0])}\n"
                f"  parent meshes = {sorted(key[1])}"
            )

        sigmap[key] = body

    return sigmap


def mjlab_body_signature(mj_body, parent_map):
    own = [g.get("mesh") for g in mj_body.findall("geom") if g.get("mesh")]
    if not own:
        return None

    parent = parent_map.get(mj_body)
    parent_meshes = []
    if parent is not None:
        parent_meshes = [g.get("mesh") for g in parent.findall("geom")
                         if g.get("mesh")]

    return frozenset(own), frozenset(parent_meshes)
def extract_visual_geoms(body):
    geoms = []
    for g in body.findall("geom"):
        # robot.xml: no class attr
        # mjlab.xml: visual geoms have class="visual"
        if g.get("mesh") is not None:
            geoms.append(g)
    return geoms


def replace_assets(mjlab_root, robot_root):
    mjlab_asset = mjlab_root.find("asset")
    robot_asset = robot_root.find("asset")

    if robot_asset is None:
        print("[WARN] robot.xml has no <asset>, skipping asset replacement")
        return

    if mjlab_asset is not None:
        mjlab_root.remove(mjlab_asset)

    mjlab_root.insert(0, deepcopy(robot_asset))
    print("[OK] replaced <asset> block")


def sync_by_geometry(robot_xml, mjlab_xml, out_xml):
    robot_tree = ET.parse(robot_xml)
    robot_root = robot_tree.getroot()

    mjlab_tree = ET.parse(mjlab_xml)
    mjlab_root = mjlab_tree.getroot()

    robot_sigmap = index_robot_bodies_by_signature(robot_root)
    mjlab_parent_map = build_parent_map(mjlab_root)

    matched = 0
    unmatched = []

    for mj_body in mjlab_root.iter("body"):
        mj_name = mj_body.get("name")

        mj_geoms = extract_visual_geoms(mj_body)
        mj_meshes = [g.get("mesh") for g in mj_geoms if g.get("mesh")]

        if not mj_meshes:
            continue

        key = mjlab_body_signature(mj_body, mjlab_parent_map)
        if key is None:
            continue

        if key not in robot_sigmap:
            unmatched.append(mj_name)
            continue

        src_body = robot_sigmap[key]

        # --------------------------------
        # 1) body pose
        # --------------------------------
        for attr in ("pos", "quat"):
            if attr in src_body.attrib:
                mj_body.set(attr, src_body.get(attr))

        # --------------------------------
        # 2) inertial
        # --------------------------------
        src_inertial = src_body.find("inertial")
        if src_inertial is not None:
            old = mj_body.find("inertial")
            if old is not None:
                mj_body.remove(old)
            mj_body.insert(0, deepcopy(src_inertial))

        # --------------------------------
        # 3) visual geoms (order-based)
        # --------------------------------
        src_visuals = extract_visual_geoms(src_body)
        dst_visuals = [g for g in mj_geoms]

        if len(src_visuals) != len(dst_visuals):
            print(
                f"[WARN] geom count mismatch for body '{mj_name}': "
                f"{len(src_visuals)} (robot) vs {len(dst_visuals)} (mjlab)"
            )

        for s, d in zip(src_visuals, dst_visuals):
            for attr in ("mesh", "pos", "quat"):
                if attr in s.attrib:
                    d.set(attr, s.get(attr))

        matched += 1
        print(f"[OK] synced body '{mj_name}'  <-  '{src_body.get('name')}'")

    # --------------------------------
    # 4) replace assets
    # --------------------------------
    replace_assets(mjlab_root, robot_root)

    ET.indent(mjlab_tree, space="  ")
    mjlab_tree.write(out_xml, encoding="utf-8", xml_declaration=True)

    print(f"\n[OK] matched bodies: {matched}")
    if unmatched:
        print(f"[INFO] unmatched MJLab bodies: {unmatched}")
    print(f"[OK] wrote {out_xml}")


if __name__ == "__main__":
    sync_by_geometry(
        robot_xml="robot.xml",
        mjlab_xml="robot_mjlab.xml",
        out_xml="robot_mjlab_gen.xml",
    )