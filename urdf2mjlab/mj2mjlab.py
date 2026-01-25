#!/usr/bin/env python3
"""
mjlabify.py

Update a "known-good MJLab" MuJoCo XML (reference) using geometry + inertial data
from a "raw" MuJoCo/URDF-like XML (source), using an explicit body mapping.

Key design choice:
- NO mesh-signature matching (it breaks when parts repeat across left/right).
- We use an explicit mapping: reference_body_name -> source_body_name.

Typical usage:
  python mjlabify.py \
    --src robot.xml \
    --ref robot_mjlab.xml \
    --out robot_mjlab_out.xml
"""

from __future__ import annotations

import argparse
import copy
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict
from typing import Dict, List, Optional, Set, Tuple


# -----------------------------
# Utilities
# -----------------------------

def parse_xml(path: str) -> ET.ElementTree:
    return ET.parse(path)


def index_bodies_by_name(root: ET.Element) -> Dict[str, ET.Element]:
    out: Dict[str, ET.Element] = {}
    for b in root.iter("body"):
        name = b.attrib.get("name")
        if name:
            out[name] = b
    return out


def first_child(elem: ET.Element, tag: str) -> Optional[ET.Element]:
    for c in list(elem):
        if c.tag == tag:
            return c
    return None


def direct_children(elem: ET.Element, tag: str) -> List[ET.Element]:
    return [c for c in list(elem) if c.tag == tag]


def collect_used_meshes(root: ET.Element) -> Set[str]:
    used: Set[str] = set()
    for g in root.iter("geom"):
        m = g.attrib.get("mesh")
        if m:
            used.add(m)
    return used


def remove_all_children(elem: ET.Element, tag: str) -> None:
    for c in list(elem.findall(tag)):
        elem.remove(c)


def indent(elem: ET.Element, level: int = 0) -> None:
    # Deterministic pretty-print indentation (MJLab doesn't care about whitespace).
    i = "\n" + level * "  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + "  "
        for child in elem:
            indent(child, level + 1)
        if not child.tail or not child.tail.strip():
            child.tail = i
    if level and (not elem.tail or not elem.tail.strip()):
        elem.tail = i


# -----------------------------
# Core transformations
# -----------------------------

def strip_actuators_and_joint_ctrl_attrs(root: ET.Element, strip_joint_ctrl_attrs: bool = True) -> None:
    """
    Remove the <actuator> section entirely (MJLab training setups usually want no actuators),
    and optionally strip joint attributes that are control-related.
    """
    act = root.find("actuator")
    if act is not None:
        root.remove(act)

    if not strip_joint_ctrl_attrs:
        return

    ctrl_attrs = {
        "actuatorfrcrange",
        "armature",
        "damping",
        "frictionloss",
        "stiffness",
        "limited",
        "margin",
        "solreflimit",
        "solimplimit",
    }
    for j in root.iter("joint"):
        for a in list(j.attrib.keys()):
            if a in ctrl_attrs:
                del j.attrib[a]


def update_body_pose_and_inertial(ref_body: ET.Element, src_body: ET.Element) -> None:
    # Update body pose
    for k in ("pos", "quat"):
        if k in src_body.attrib:
            ref_body.attrib[k] = src_body.attrib[k]

    # Update inertial attributes (keep element location)
    src_in = first_child(src_body, "inertial")
    ref_in = first_child(ref_body, "inertial")
    if src_in is None:
        return

    if ref_in is None:
        # Insert inertial at top (before joint/geoms typically)
        ref_body.insert(0, copy.deepcopy(src_in))
    else:
        ref_in.attrib = dict(src_in.attrib)


def update_visual_geoms_by_mesh_name(ref_body: ET.Element, src_body: ET.Element) -> None:
    """
    Update only "visual" geoms in ref body using src geoms matched by mesh name.
    We *do not* add/remove geoms here; we only update pos/quat for existing ones.
    This keeps collisions/sensors/etc from the reference intact.

    Why this works:
    - Your ref MJLab file already encodes the MJLab-friendly structure.
    - Your src file encodes the 'fresh' placements (pos/quat) for those meshes.
    """
    src_geoms = [g for g in direct_children(src_body, "geom") if g.attrib.get("mesh")]
    src_by_mesh: Dict[str, List[ET.Element]] = defaultdict(list)
    for g in src_geoms:
        src_by_mesh[g.attrib["mesh"]].append(g)

    for rg in direct_children(ref_body, "geom"):
        # Only touch visual geoms in the reference.
        if rg.attrib.get("class") != "visual":
            continue
        mesh = rg.attrib.get("mesh")
        if not mesh:
            continue

        if mesh in src_by_mesh and src_by_mesh[mesh]:
            sg = src_by_mesh[mesh].pop(0)
            for k in ("pos", "quat"):
                if k in sg.attrib:
                    rg.attrib[k] = sg.attrib[k]


def rebuild_asset_meshes_from_src(
    ref_root: ET.Element,
    src_root: ET.Element,
    keep_only_used_by_ref: bool = True,
) -> None:
    """
    Rebuild <asset><mesh .../> in the reference using mesh filenames from the source,
    but (by default) only for meshes actually referenced by geoms in the reference worldbody.
    """
    ref_asset = ref_root.find("asset")
    if ref_asset is None:
        ref_asset = ET.SubElement(ref_root, "asset")

    src_asset = src_root.find("asset")
    src_mesh_map: Dict[str, Dict[str, str]] = {}
    if src_asset is not None:
        for m in src_asset.findall("mesh"):
            name = m.attrib.get("name")
            if name:
                src_mesh_map[name] = dict(m.attrib)

    used = collect_used_meshes(ref_root) if keep_only_used_by_ref else set(src_mesh_map.keys())

    # Preserve materials/textures in reference; only replace mesh children.
    remove_all_children(ref_asset, "mesh")

    # Keep deterministic ordering:
    # - If a mesh exists in source, emit it (name/file).
    # - Otherwise skip (or you can fall back to an existing file).
    for mesh_name in sorted(used):
        if mesh_name in src_mesh_map:
            file_ = src_mesh_map[mesh_name].get("file", "")
            ET.SubElement(ref_asset, "mesh", {"name": mesh_name, "file": file_})


def update_ref_from_src(
    src_root: ET.Element,
    ref_root: ET.Element,
    body_map: Dict[str, str],
) -> None:
    ref_bodies = index_bodies_by_name(ref_root)
    src_bodies = index_bodies_by_name(src_root)

    for ref_name, src_name in body_map.items():
        rb = ref_bodies.get(ref_name)
        sb = src_bodies.get(src_name)
        if rb is None:
            print(f"[WARN] ref body not found: {ref_name}", file=sys.stderr)
            continue
        if sb is None:
            print(f"[WARN] src body not found: {src_name} (for ref {ref_name})", file=sys.stderr)
            continue

        update_body_pose_and_inertial(rb, sb)
        update_visual_geoms_by_mesh_name(rb, sb)

    # Update asset meshes last (based on what reference worldbody uses)
    rebuild_asset_meshes_from_src(ref_root, src_root, keep_only_used_by_ref=True)


# -----------------------------
# Default mapping for your 12DOF case
# -----------------------------

DEFAULT_12DOF_BODY_MAP = {
    # base
    "base": "base",

    # RIGHT LEG
    "right_hip_z_link":   "hipx_subassembly",
    "right_hip_x_link":   "hipy_subassembly",
    "right_hip_y_link":   "tigh_subassembly",
    "right_knee_link":    "shin_subassembly",
    "right_ankle_y_link": "ankle_subassembly",
    "right_ankle_x_link": "foot_subassembly",

    # LEFT LEG
    "left_hip_z_link":    "hipx_subassemby_sym",   # note: your src typo is "subassemby"
    "left_hip_x_link":    "hipy_subassembly_sym",
    "left_hip_y_link":    "tigh_subassembly_sym",
    "left_knee_link":     "shin_subassembly_sym",
    "left_ankle_y_link":  "ankle_subassembly_2",
    "left_ankle_x_link":  "foot_subassembly_2",
}


# -----------------------------
# CLI
# -----------------------------

def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True, help="Source XML (raw urdf-like mujoco), e.g. robot.xml")
    ap.add_argument("--ref", required=True, help="Reference MJLab XML (known-good), e.g. robot_mjlab.xml")
    ap.add_argument("--out", required=True, help="Output MJLab XML path")
    ap.add_argument("--keep-joint-ctrl-attrs", action="store_true",
                    help="Do NOT strip joint control attrs like actuatorfrcrange/armature/etc.")
    args = ap.parse_args()

    src_tree = parse_xml(args.src)
    ref_tree = parse_xml(args.ref)

    src_root = src_tree.getroot()
    ref_root = ref_tree.getroot()

    # 1) Update reference from source (geometry + inertias + assets)
    update_ref_from_src(src_root, ref_root, DEFAULT_12DOF_BODY_MAP)

    # 2) Remove actuators to match MJLab expectations (and your target file)
    strip_actuators_and_joint_ctrl_attrs(ref_root, strip_joint_ctrl_attrs=(not args.keep_joint_ctrl_attrs))

    # 3) Pretty print and write
    indent(ref_root)
    ref_tree.write(args.out, encoding="utf-8", xml_declaration=False)
    print(f"[OK] Wrote MJLab XML: {args.out}")


if __name__ == "__main__":
    main()