# G7 OpenArm code generation

The G7 generator uses robot joint names as its public contract. Do not encode
Pinocchio q/v offsets in `generate.py`.

## Canonical URDF

The canonical robot description lives in the G7 project. Before regenerating,
check that this local URDF has not drifted:

```bash
python models/g7_openarm/sync_urdf.py \
  <g7-repo>/modules/g7_openarm_mujoco/src/g7_openarm_mujoco/model/urdf/g7_openarm.urdf
```

Use `--write` to intentionally resync it. The sync script permits only these
PinnZoo-specific transforms:

- remove the MuJoCo-only `<mujoco>` extension
- rewrite mesh paths to this model's `meshes/` directory
- convert the four AMR wheel joints from `continuous` to very-wide scalar
  `revolute` joints (`+/-1e6`)

The wheel conversion is required because Pinocchio otherwise represents a
continuous joint with a two-value cos/sin configuration, while this generator
expects scalar joints (or a 7/6 free flyer).

## Name-based generated ABI

`ACTUATED_JOINTS` in `generate.py` contains semantic URDF joint names. The
`SymbolicGenerator` resolves each name to its Pinocchio velocity DoF after the
URDF is loaded and fails on missing, duplicate, non-scalar, or invalid joints.

Newly generated libraries expose vector-order API v2. In addition to the
existing null-terminated name arrays, `vector_orders.c` exports name lookup and
joint metadata functions for q, v, torque, joint `nq/nv`, and kinematics output
width. Consumers should resolve and validate names once at startup, then use
cached integer indices in realtime code.

## Regenerate and build

Generation requires Python Pinocchio and CasADi:

```bash
python models/g7_openarm/generate.py
cmake -S . -B build
cmake --build build --target g7_openarm_quat
```

Build each deployment architecture separately. Replacing an existing v1 `.so`
with the newly generated v2 library does not require changing the G7 Python
binding; it supports both and performs stronger cross-checks when v2 metadata
is present.
