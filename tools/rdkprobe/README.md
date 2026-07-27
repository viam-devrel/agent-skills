# rdkprobe — the RDK oracle for armkit

A ~70-line Go program that asks the **real** Viam RDK what it thinks of a kinematics file.

## Why this exists

`armkit` is a Python reimplementation of behavior that lives in Go, in RDK's
`referenceframe` package. Two questions have to be answerable, and neither can be answered
from Python:

1. **Does armkit accept exactly what RDK accepts?** (parity)
2. **Does armkit compute the same poses RDK computes?** (forward kinematics)

This tool answers both by calling `referenceframe.KinematicModelFromFile` and
`Model.Transform` directly.

**It exists because reading RDK source repeatedly produced wrong conclusions during
development, and running it did not.** Among the errors caught by running rather than
reading:

- A claim that RDK accepts branching models. It does not — `model_json.go` requires
  exactly one leaf when no `output_frames` is declared, and URDF cannot declare one.
  The probe said: `need exactly one end effector, have [finger_l finger_r]`.
- A mimic-joint DoF discrepancy: RDK reports `DoF=2` on `test_mimic_serial.urdf` where
  armkit reported 3, because mimic frames consume no input slot.
- The order in which RDK composes transitive mimic multipliers and offsets. Both orderings
  produce plausible numbers; only the probe distinguishes them.

If you are about to assert something about RDK's behavior, run this first.

## Usage

```bash
cd tools/rdkprobe

# Accept/reject verdict and DoF
go run . path/to/arm.urdf path/to/other.json

# Forward kinematics at a joint configuration
go run . ../../skills/viam-arm-module/scripts/tests/fixtures/ur20.urdf \
  --at 0.1,-0.4,0.7,0.2,-0.3,0.5
```

Output lines are one of:

```
ACCEPT  <file>  DoF=6
REJECT  <file>  <RDK's error message, verbatim>
POSE    <file>  point_mm=[x y z]  quat=[w x y z]
```

Points are millimeters. **Quaternions print as `(w, x, y, z)`** — that is
`quat.Number{Real, Imag, Jmag, Kmag}` in RDK's ordering. Both are printed at 9 decimals,
which is enough to distinguish a correct implementation from a subtly wrong one.

`--at` takes a comma-separated list whose length must equal the model's DoF. Remember that
mimic joints consume no slot, so a file with four joints may take three values.

## Version pinning

Pinned to **RDK v1.0.0** (`go.mod`).
`skills/viam-arm-module/scripts/tests/test_parity.py` records this tool's output as literal
expected values, stamped with that version and the date they were captured.

**When RDK is upgraded, a changed verdict is a finding, not a test to update.** It means
RDK's behavior moved, and armkit must decide deliberately whether to follow. See the
"Parity drift" section of `docs/plans/2026-07-27-viam-arm-module-implementation.md` for the
procedure.

## Not part of the published skill

This is a development tool. It needs a Go toolchain and a populated module cache, and no
end user of the `viam-arm-module` skill ever runs it. It lives here so that anyone
maintaining `armkit` can re-verify parity instead of re-deriving it from source.

## Known divergences it has established

Two places where armkit deliberately differs from RDK, both recorded in `test_parity.py`:

- **Missing mesh files.** RDK hard-fails during parse (`failed to build mesh map`).
  armkit accepts and reports a finding — a validator that refuses to look at a file
  because an asset is missing is backwards.
- **A joint with no `<origin>`.** RDK panics (`SIGSEGV` at `model_urdf.go:196`, indexing
  an empty slice). armkit defaults to identity, per the URDF spec. armkit is correct here.

Both share a consequence worth stating in any user-facing output: **"armkit PASS" does not
imply "RDK will load this file."**
