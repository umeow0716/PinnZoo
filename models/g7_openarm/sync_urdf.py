"""Synchronize the PinnZoo G7 URDF from the canonical project URDF.

PinnZoo/Pinocchio intentionally represents the four wheel joints as very-wide
revolute joints instead of URDF ``continuous`` joints. Pinocchio otherwise uses
its cos/sin two-configuration representation for continuous joints, while this
code generator supports scalar or free-flyer configuration blocks.

All other robot semantics are copied from the source URDF. Mesh paths are made
relative to this PinnZoo model directory and the MuJoCo-only compiler extension
is removed.
"""

from __future__ import annotations

import argparse
import copy
import difflib
from pathlib import Path
import xml.etree.ElementTree as ET

WHEEL_JOINTS = frozenset(
    {
        "AMR_FLW_joint",
        "AMR_FRW_joint",
        "AMR_RLW_joint",
        "AMR_RRW_joint",
    }
)
WHEEL_LIMIT = 1_000_000.0
LOCAL_URDF = Path(__file__).with_name("g7_openarm.urdf")


def _prepare_tree(source: Path) -> ET.ElementTree:
    tree = ET.parse(source)
    root = tree.getroot()
    if root.tag != "robot":
        raise ValueError(f"Expected URDF <robot> root in {source}")

    for mujoco_tag in list(root.findall("mujoco")):
        root.remove(mujoco_tag)

    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename")
        if filename:
            mesh.set("filename", f"meshes/{Path(filename).name}")

    seen_wheels: set[str] = set()
    unsupported_continuous: list[str] = []
    for joint in root.findall("joint"):
        name = joint.get("name") or ""
        joint_type = joint.get("type")
        if name in WHEEL_JOINTS:
            seen_wheels.add(name)
            if joint_type not in {"continuous", "revolute"}:
                raise ValueError(
                    f"Wheel joint {name!r} must be continuous/revolute, got {joint_type!r}"
                )
            joint.set("type", "revolute")
            limit = joint.find("limit")
            if limit is None:
                limit = ET.SubElement(joint, "limit")
            limit.set("lower", str(-WHEEL_LIMIT))
            limit.set("upper", str(WHEEL_LIMIT))
        elif joint_type == "continuous":
            unsupported_continuous.append(name)

    missing = sorted(WHEEL_JOINTS - seen_wheels)
    if missing:
        raise ValueError(f"Canonical URDF is missing expected wheel joints: {missing}")
    if unsupported_continuous:
        raise ValueError(
            "PinnZoo generator does not support additional continuous joints: "
            f"{unsupported_continuous}"
        )

    ET.indent(tree, space="  ")
    return tree


def _semantic_lines(root: ET.Element) -> list[str]:
    """Stable structural representation that ignores XML attribute ordering."""
    lines: list[str] = []

    def visit(element: ET.Element, depth: int) -> None:
        attrs = " ".join(f"{key}={value!r}" for key, value in sorted(element.attrib.items()))
        text = (element.text or "").strip()
        lines.append(f"{'  ' * depth}<{element.tag} {attrs}> {text}".rstrip())
        for child in element:
            visit(child, depth + 1)

    visit(root, 0)
    return lines


def check(source: Path, target: Path = LOCAL_URDF) -> None:
    expected = _prepare_tree(source).getroot()
    actual = ET.parse(target).getroot()
    expected_lines = _semantic_lines(expected)
    actual_lines = _semantic_lines(actual)
    if expected_lines != actual_lines:
        diff = "\n".join(
            difflib.unified_diff(
                actual_lines,
                expected_lines,
                fromfile=str(target),
                tofile=f"normalized:{source}",
                lineterm="",
            )
        )
        raise RuntimeError(
            "PinnZoo G7 URDF has drifted from the canonical source outside the "
            f"approved wheel/mesh/MuJoCo transforms:\n{diff}"
        )


def write(source: Path, target: Path = LOCAL_URDF) -> None:
    tree = _prepare_tree(source)
    tree.write(target, encoding="utf-8", xml_declaration=True)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("source_urdf", type=Path, help="canonical G7 project URDF")
    parser.add_argument(
        "--write",
        action="store_true",
        help="overwrite PinnZoo's local URDF with the normalized canonical source",
    )
    args = parser.parse_args()

    source = args.source_urdf.expanduser().resolve()
    if not source.is_file():
        raise FileNotFoundError(source)
    if args.write:
        write(source)
    check(source)
    print("G7 PinnZoo URDF matches canonical source after approved transforms.")


if __name__ == "__main__":
    main()
