# urdf-doctor 🔬

![Build and Test](https://github.com/iameijaz/urdf-doctor/actions/workflows/build-and-test.yml/badge.svg)

Static analysis tool for URDF robot description files. Catches errors, simulation pitfalls,
and inconsistencies before they crash your controller or waste hours of debugging.

No ROS2 installation required. Single C binary. Runs anywhere.

## What it checks

### Inertial properties
- Missing `<inertial>` block on dynamic links
- Inertial block present but incomplete (missing mass or inertia tensor)
- Zero or negative mass values
- Negative inertia principal values (`ixx`, `iyy`, `izz ≤ 0`)
- Non-positive-definite inertia matrix (Sylvester's criterion)
- Inertia tensor violating triangle inequality
- Cross terms (`ixy`, `ixz`, `iyz`) unrealistically large
- Center of mass far outside expected bounds
- Extremely small or large mass values (unit mismatch signal)

### Joint validation
- Invalid axis vector (`0 0 0`)
- Non-normalized joint axis vector
- Missing `<axis>` tag on revolute/prismatic joints
- Missing `<limit>` on revolute/prismatic joints
- Joint limits inverted (`lower > upper`)
- Equal joint limits (joint cannot move)
- Zero effort or velocity limits on actuated joints
- Unrealistically large joint ranges or limits
- Extremely high effort/velocity (unstable controllers)
- Missing `<safety_controller>` soft limits
- Soft limits outside hard limits
- Floating joint used unintentionally

### Kinematic tree
- Multiple root links detected
- No root link detected
- Cyclic kinematic chain (link appears as child twice)
- Orphan links not connected to any joint
- Parent or child link not found in URDF
- Parent and child are the same link

### Geometry and meshes
- Visual geometry with no collision geometry
- Collision geometry with no visual geometry
- Collision geometry much larger than visual geometry
- `file://` absolute paths that break portability
- Unsafe relative mesh paths (`../../../` traversal)
- Unsupported mesh formats (`.obj`, `.3ds`, `.fbx`)
- Mesh scale suggesting mm/m unit mismatch

### Names and references
- Duplicate link names
- Duplicate joint names
- Invalid link or joint names (spaces or illegal characters)
- Missing `<origin>` on joints (informational)
- References to undefined materials

## Installation

```bash
git clone https://github.com/iameijaz/urdf-doctor.git
cd urdf-doctor
make
sudo make install
```

Or one-liner:
```bash
curl -sL https://github.com/iameijaz/urdf-doctor/releases/latest/download/urdf-doctor \
  -o /usr/local/bin/urdf-doctor && chmod +x /usr/local/bin/urdf-doctor
```

## Usage

```bash
urdf-doctor robot.urdf              # check with errors + warnings
urdf-doctor robot.urdf --all        # include INFO diagnostics
urdf-doctor robot.urdf --errors     # errors only
urdf-doctor robot.urdf --quiet      # summary line only
urdf-doctor robot.urdf --no-color   # for CI logs or file output
```

## Example output

```
urdf-doctor v1.0.0
────────────────────────────────────
  my_robot.urdf

  [ERROR] <wheel_left>       zero mass — will crash controllers
  [ERROR] <wheel_right>      ixx = -0.001000 — must be positive
  [ERROR] <base_to_camera>   joint limits inverted: lower (1.5000) > upper (-1.5000)
  [ERROR] <bad_joint>        parent link 'nonexistent_link' not found in URDF
  [WARN ] <camera_mount>     missing <inertial> block — required for dynamic simulation
  [WARN ] <sensor_link>      absolute file:// URI breaks portability
  [WARN ] <base_to_panel>    effort limit is 0.0000 — actuated joint with zero effort

  links     : 8
  joints    : 6
  materials : 2

  ✗ 4 error(s)
  ⚠ 3 warning(s)
```

## Exit codes

| Code | Meaning |
|------|---------|
| `0` | No errors or warnings |
| `1` | Warnings found, no errors |
| `2` | Errors found |
| `3` | Could not read file |

Use exit codes in CI pipelines to gate deployments on clean URDFs.

## Use in CI

```yaml
- name: Validate URDF
  run: urdf-doctor src/my_robot/urdf/robot.urdf --errors
```

Exit 2 fails the pipeline. Exit 0 or 1 passes.

## Requirements

- Linux or Windows
- No ROS2 installation required
- Standard C library + `libm` only

## License

MIT — see [LICENSE](LICENSE)

---

*Part of the [Verbit](https://github.com/iameijaz) robotics utility toolkit.*
*Related: [ros-kill-all](https://github.com/iameijaz/ros-kill-all) · [netwatch](https://github.com/iameijaz/netwatch) · [dmitree](https://github.com/iameijaz/dmitree)*
