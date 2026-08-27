#!/usr/bin/env python3
"""PinnZoo URDF converter v2.

Convert a URDF into the form expected by PinnZoo/Pinocchio code generation.

Only three semantic transforms are performed:
1. URDF `continuous` joints -> scalar `revolute` joints with wide limits.
2. Mesh paths -> `meshes/<basename>` (e.g. `../meshes/foo.stl` -> `meshes/foo.stl`).
3. Remove top-level `<mujoco>...</mujoco>` extension blocks.

Uses only the Python standard library.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

DEFAULT_WIDE_LIMIT = 1_000_000.0


def _mesh_basename(filename: str) -> str:
    """Return the final path component using URDF-style forward slashes."""
    normalized = filename.replace("\\", "/")
    return normalized.rsplit("/", 1)[-1]


def convert_urdf(source: Path, target: Path, wide_limit: float) -> tuple[int, int, int]:
    if wide_limit <= 0:
        raise ValueError("wide_limit must be > 0")

    # insert_comments=True keeps XML comments when possible.
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    tree = ET.parse(source, parser=parser)
    root = tree.getroot()

    if root.tag != "robot":
        raise ValueError(f"Expected URDF <robot> root, got <{root.tag}>")

    converted_joints = 0
    rewritten_meshes = 0
    removed_mujoco_blocks = 0

    # PinnZoo's symbolic generator accepts scalar joints (nq == 1) and the
    # 7/6 free-flyer block. Pinocchio represents URDF continuous joints with
    # nq == 2 (cos(theta), sin(theta)), so make them wide revolute joints.
    for joint in root.findall("joint"):
        if joint.get("type") != "continuous":
            continue

        joint.set("type", "revolute")
        limit = joint.find("limit")
        if limit is None:
            limit = ET.SubElement(joint, "limit")

        # Preserve any existing effort/velocity attributes; only add/replace
        # the lower/upper bounds required by a revolute joint.
        limit.set("lower", str(-wide_limit))
        limit.set("upper", str(wide_limit))
        converted_joints += 1

    # Match PinnZoo's model directory layout: URDF and meshes/ are siblings.
    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename")
        if not filename:
            continue

        new_filename = f"meshes/{_mesh_basename(filename)}"
        if filename != new_filename:
            mesh.set("filename", new_filename)
            rewritten_meshes += 1

    # PinnZoo does not need MuJoCo-specific URDF extension blocks. Remove only
    # top-level <mujoco> blocks; normal URDF content is otherwise preserved.
    for mujoco in list(root.findall("mujoco")):
        root.remove(mujoco)
        removed_mujoco_blocks += 1

    # Validation: no continuous joints, all mesh paths are under meshes/, and
    # no top-level MuJoCo extension block remains.
    remaining_continuous = [
        joint.get("name", "<unnamed>")
        for joint in root.findall("joint")
        if joint.get("type") == "continuous"
    ]
    if remaining_continuous:
        raise RuntimeError(
            "continuous joints remain after conversion: "
            + ", ".join(remaining_continuous)
        )

    invalid_meshes = [
        mesh.get("filename", "")
        for mesh in root.findall(".//mesh")
        if mesh.get("filename") and not mesh.get("filename", "").startswith("meshes/")
    ]
    if invalid_meshes:
        raise RuntimeError(
            "mesh paths outside meshes/ remain after conversion: "
            + ", ".join(invalid_meshes)
        )

    if root.findall("mujoco"):
        raise RuntimeError("top-level <mujoco> block remains after conversion")

    target.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(target, encoding="utf-8", xml_declaration=True)
    return converted_joints, rewritten_meshes, removed_mujoco_blocks


def default_output_path(source: Path) -> Path:
    return source.with_name(f"{source.stem}_pinnzoo{source.suffix}")


def main() -> int:
    argp = argparse.ArgumentParser(
        description="Convert a URDF to PinnZoo-compatible joint, mesh-path, and MuJoCo-extension conventions."
    )
    argp.add_argument("input", type=Path, help="source URDF")
    argp.add_argument(
        "output",
        nargs="?",
        type=Path,
        help="output URDF (default: <input_stem>_pinnzoo.urdf)",
    )
    argp.add_argument(
        "--wide-limit",
        type=float,
        default=DEFAULT_WIDE_LIMIT,
        metavar="RAD",
        help=f"lower/upper magnitude for converted continuous joints (default: {DEFAULT_WIDE_LIMIT:g})",
    )
    args = argp.parse_args()

    source = args.input.expanduser()
    target = (args.output or default_output_path(source)).expanduser()

    if not source.is_file():
        argp.error(f"input URDF does not exist: {source}")

    try:
        joint_count, mesh_count, mujoco_count = convert_urdf(source, target, args.wide_limit)
    except (ET.ParseError, OSError, ValueError, RuntimeError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    print(f"output: {target}")
    print(f"continuous -> revolute: {joint_count}")
    print(f"mesh paths rewritten: {mesh_count}")
    print(f"<mujoco> blocks removed: {mujoco_count}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
