# CAD agent instructions — hexapod prototype

These rules apply to any LLM coding agent editing the parametric CAD
in `hexapod_walker/prototype_sts3215/`.  Read them in full before touching
geometry; paste them into context for any helper / sub-agent you spawn
to make a change.

The pipeline they refer to is documented in `CAD_WORKFLOW.md`.

## Rules

1. **Before editing CAD, list every functional clearance and keep-out
   volume that the change might intrude into.**  The canonical
   inventory is `design_spec.yaml` (per-part `keep_out_volumes`,
   `wire_channels`, `clearances`) plus the `KEEP_OUT_VOLUMES`
   registry in `keepout_volumes.py`.  Skipping this step is how every
   recurring regression has gotten in; see the long comments in
   `_verify_prototype.py` for examples.

2. **Never add material inside a keep-out volume.**  This is a hard
   rule.  Common offenders:
   - `pad_sweep_clear` (a cylinder along Y inside the coxa link's
     pedestal + hub; the femur hip pad sweeps through it).
   - `yaw_horn_sweep` (cylinder above the chassis cradle's horn-bolt
     circle; the disc horn rotates inside it).  (`link_hub_horn_sweep`
     was retired Aug 2026: the coxa hub is bolted to and rotates WITH
     its horn, so a horn-sweep keep-out on the hub itself was
     meaningless.)
   - `*_servo_body` (rectangular pockets that hold each servo body —
     the body must drop in cleanly).
   - The L-shaped wire-exit corridor at the +X bottom-outboard
     corner of every cradle (see `_wire_exit_slot` in
     `hexapod_prototype.py`).

3. **Never remove or shrink a named hole or channel in
   `design_spec.yaml` unless the user explicitly requests it.**  Each
   entry in `parts.<name>.holes` and `parts.<name>.wire_channels`
   represents a functional invariant (mounting bolt, wire-exit path,
   bolt-circle hole) somebody downstream depends on.  If you think a
   hole is dead, ASK before deleting it.

4. **Every new support / gusset / pad must be checked against
   rotation and wire clearances.**  Anything that adds material near a
   joint axis can clip an actively rotating part (femur hip pad,
   disc horn) or block a wire-exit corridor.  Run
   `make -C hexapod_walker/prototype check-cad` after the edit and
   look specifically at the `check_workspace_self_collision` and
   `check_wire_slot` lines.

5. **Prefer parametric dimensions over hardcoded magic numbers.**  If
   the change needs a new length / radius / clearance, add a constant
   to the constants block in `hexapod_prototype.py` (or pull from one
   that exists — there are over 80 already) instead of typing the
   literal value into the body of a `make_*` function.  Mirror the
   constant in `design_spec.yaml` with a `# = hexapod_prototype.<NAME>`
   comment so the spec tracks the change.

6. **After every geometry edit, run
   `make -C hexapod_walker/prototype check-cad` and fix failures
   before committing.**  The check writes
   `artifacts/cad_report.md` — the "Failure details" section is
   designed to be pasted back into an LLM as a diagnostic prompt.  Do
   NOT commit a CAD edit while any check is failing; if a check is
   wrong (e.g. you've intentionally rerouted a wire), update the
   spec / check first and commit the spec change together with the
   CAD change.

7. **If a requested change is ambiguous, encode the assumption in
   `design_spec.yaml`, not in a code comment.**  Example: the user
   asks "make the foot bigger".  Add (or update) the `foot_boot`
   entry's `bounds_mm` / `features` to capture exactly how much bigger,
   in which axes, with what bore interference; the validator can then
   detect drift on future runs.  A comment in `make_foot_boot()` does
   not survive a refactor; a `design_spec.yaml` entry does, and it
   shows up in the report's "Discovered dimensions" table.

8. **Whenever you change `CHASSIS_GAP` in `hexapod_prototype.py`,
   re-check every part that lives between the two chassis plates.**
   At minimum: `battery_holder` (currently 28 mm tall -- needs >=
   28 mm gap with headroom), `electronics_tray` (~ 3 mm thick),
   and the brass standoffs in `BOM.md`
   (their physical length MUST equal `CHASSIS_GAP`).  The May 2026
   audit caught the 28 mm holder ramming through a 4 mm
   `chassis_top` deck with CHASSIS_GAP = 20 mm; the fix bumped
   CHASSIS_GAP to 32 mm and the standoffs from 25 to 32 mm to
   match.  If you bump CHASSIS_GAP without updating the standoff
   length the chassis will not actually assemble.

9. **When adding cable management features, NEVER place a feature
   inside `keepout_volumes.py` cable_keepouts or the joint sweep
   cylinders; always verify against `check_cable_clearance` and
   `check_workspace_self_collision` BEFORE committing.**  The May 2026
   cable-mgmt pass (commit `wire harness: cable posts on every link +
   chassis_bottom drop slots`) added 18 printed zip-tie posts and 6
   chassis-bottom anchor tabs; the placement search had to dodge the
   bracket flange, the electronics_tray footprint, the bracket bolt
   columns, the leg-sweep cylinders AND the cable_keepout boxes for
   every USB/HDMI/I2C connector airspace.  Most candidate positions
   FAILED one of these.  The workflow is: (a) sketch the new feature
   in bracket-local / well-local coordinates, (b) compute the leg-0
   chassis-frame position via `_leg_chassis_frames()` and check
   against every leg's transform (rotational symmetry doesn't help
   when the tray is asymmetric -- the May 2026 Pi cantilever shifted
   the tray's chassis-frame Y centre to -2.5), (c) hand-verify the
   z-range against the bracket flange's [+2, +17] mm slab and the
   tray's [+5, +8] mm slab, (d) ONLY THEN run
   `_verify_prototype.py --all`.  If a candidate fails any of
   `check_workspace_self_collision`, `check_cable_clearance`,
   `check_wire_slot`, or `check_leg_harness_drop`, MAKE THE FEATURE
   SMALLER (or reroute it); never shrink the keep-out volume.

10. **Any printed feature that takes a threaded fastener for repeated
    assembly MUST use a heat-set insert, not a self-tap pilot.**  The
   May 2026 cradle audit (commit history under "Design D") showed
   that self-tap pilots into Phi 2.5 mm printed holes are
   structurally inadequate when the surrounding wall material isn't
   explicitly verified (7 of 12 cradle sites had <= 1.5 mm of plastic
   radially -- see PROTOTYPE.md "Design D" for the audit table).
   The fix is to switch from "M3 SHCS into Phi 2.5 mm self-tap pilot"
   to "M3 SHCS into M3 brass heat-set insert (McMaster `94459A130`)
   in a Phi 4 mm pocket, surrounded by a Phi 8 mm boss".  Real metal
   threads instead of plastic + the boss enlargement supplies the
   missing radial material.  The exception is one-time-use joints
   (e.g. the foot pad, chassis hardware nyloc through-bolts) where
   no dis-assembly is expected; those can stay plain SHCS into
   self-tap or SHCS + captive nut.  Any new code that introduces a
   bolt-into-plastic pilot must add an analogous **radial-material
   check** to `_verify_prototype.py` -- probing 8 azimuths at
   `pilot_radius + min_wall` from the bolt axis at the top and
   bottom of the pilot.  See `check_cradle_insert_pockets` for the
   reference implementation.

## Workflow at a glance

```
$ make -C hexapod_walker/prototype check-cad-fast    # inner-loop ~30 s
$ make -C hexapod_walker/prototype check-cad         # full check ~2 min
$ open hexapod_walker/prototype_sts3215/artifacts/cad_report.md
```

If a single failure dominates: drop into the matching
`_verify_prototype.check_*` function and read the long comments — they
typically explain the constants involved and the prior failure history.

## Useful entry points

- `hexapod_prototype.py` constants block (lines ~80 to ~700): every
  geometric constant + docstring on what depends on it.
- `_verify_prototype.py` (~2.5 k LOC) — every validity check.
- `keepout_volumes.py` — code-driven registry of keep-out / clearance
  meshes; the validator + the render overlays both consume it.
- `design_spec.yaml` — bounding boxes, named holes / channels /
  keep-outs, print orientations.

Happy designing.
