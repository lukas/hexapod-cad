# Two-piece CNC chorn experiment

This is a separate, non-production full-robot version of
`cnc_chorn_overhead`.  It changes only the aluminum hip/knee clamps; the
existing overhead robot remains untouched.

## The assembly change

The one-piece clamp makes both independently rotating horn discs agree with
eight peripheral holes at once.  This version splits the clamp at the passive
blade:

1. `chorn_drive_frame_cnc` bolts to the driven horn by itself.
2. `chorn_passive_plate_cnc` bolts to the passive horn by itself.
3. A 2.8 x 24 x 2 mm tongue enters a matching receiver with 0.15 mm clearance
   per wall.
4. Two M2.5x6 low-head screws secure the split from the passive outer face.

The broad tongue is the locating/shear feature; the two screws clamp the
joint.  Peripheral horn screws become a one-time installation.  Routine
separation uses the two centre horn-retention screws plus the two accessible
split screws, so the eight-hole clocking operation is gone.

## Current result

| | one-piece | two-piece experiment |
|---|---:|---:|
| clamp mass | 48.0 g | 49.3 g |
| parts per clamp | 1 | 2 |
| recurring split screws | n/a | 2 x M2.5x6 |
| simultaneous horn patterns | 2 | 0 |

The assembled split clamp keeps the original disc faces, 38.04 mm nominal
span, 42 mm reach, printed-part Datum A, and workspace envelope.  The widened
join boss lives below the passive disc face; only the small passive screw-head
land extends 0.7 mm past Datum A, in a region where both inherited printed
parts are empty.

This remains an experiment.  The tongue root needs a generous production
fillet plus FEA or a physical proof-load test before a machining release.  The
STEP bundle is useful for inspection and iteration, not yet for ordering 12
production pairs.

## Build

From `hexapod_walker/prototype_sts3215`:

```sh
uv run python concepts/cnc_chorn_two_piece/make_two_piece_chorn_variant.py
```

Fast iteration:

```sh
uv run python concepts/cnc_chorn_two_piece/make_two_piece_chorn_variant.py \
  --skip-brep --skip-sweep
```

Canonical BREP sources are in `build_two_piece_chorn_step.py`.  Generated STEP
and tessellated files land under `step/`; the full self-contained BuildViz
asset set lands under `stl/`.

## BuildViz

Build id: `prototype_sts3215/cnc-chorn-two-piece`

```sh
npx buildviz push --project prototype_sts3215 --build cnc-chorn-two-piece \
  --upload-assets --scene concepts/cnc_chorn_two_piece/scene.json \
  -m "experimental keyed two-piece horn clamp"

uv run ../../tools/push_cloud_buildviz.py \
  --build-id prototype_sts3215/cnc-chorn-two-piece
```

Local view:
`http://127.0.0.1:5183/?project=prototype_sts3215&build=cnc-chorn-two-piece`

Cloud view:
`https://buildviz.cwd1f0-new-cluster.coreweave.app/?project=prototype_sts3215&build=cnc-chorn-two-piece`
