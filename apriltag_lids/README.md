# AprilTag motor-lid covers

This folder contains optional, two-colour `tag36h11` covers for the shared
STS3215 hip/knee `servo_clamp_cap`. The production cap is unchanged.

The ready-to-print set is `out/tag36h11_motor_lids_01-16_BambuStudio.3mf`.
It contains 16 covers arranged on one plate, with white assigned to
extruder/AMS slot 1 and black to slot 2. The portable standards-based 3MF and
aligned black/white STL pairs are emitted alongside it.

An additional 20-cover plate, IDs 100-119, is available as
`out/tag36h11_motor_lids_100-119_BambuStudio.3mf`. It uses the same portrait
geometry, 34 mm tags, mounting pattern, and two-colour assignments. These are
additional marker IDs rather than motor assignments in the current robot
pose configuration.

## Why 3MF instead of STL?

STL carries triangles only and has no dependable material or colour model.
The two colours can be represented as two aligned STL bodies, but a 3MF keeps
those bodies together and records their white/black material assignments.

Each cover is 61.8 x 34.3 x 1.4 mm, but its artwork is oriented as a portrait:
the 61.8 mm axis is vertical and the mounting screws appear above and below
the tag. The tag retains the maximum 34.0 mm size, including its required
white quiet zone (3.4 mm cells; 27.2 mm black square). A small black pixel-font
ID sits below the untouched quiet zone and above the lower screw. A 0.6 mm
white base makes the part continuous; the upper 0.8 mm contains coplanar
white and black regions, so the camera-facing surface has no raised pixels or
raised lettering.

IDs 1-6 match `L0_coxa` through `L5_coxa`; IDs 7-12 match `L0_femur` through
`L5_femur` in `hexapod-tracker/configs/housing_pose_config.example.json`.
IDs 13-16 are
extra numbered covers in the same clamp-cap geometry; they are not assigned
to motor caps by the current pose configuration. Tag 0 and the tibia/yaw
locations need different mount geometry.

## Fasteners

Use **two M3x10 90-degree flat-head/countersunk screws with head diameter no
larger than 6.0 mm per cover**. Do not reuse the M3x8 socket-head screws on top
of the accessory: their heads would stand proud in the moving-yoke envelope.

The screw axes come directly from
`hexapod_prototype.servo_clamp_bolt_centres()` at X = +/-27.2 mm and the cover
has a 3.4 mm through bore plus a 6.0 mm, 90-degree countersink.

## Build and verify

From the repository root:

```sh
uv run python hexapod_walker/prototype_sts3215/apriltag_lids/make_apriltag_lids.py
uv run python hexapod_walker/prototype_sts3215/apriltag_lids/make_apriltag_lids.py --ids 100-119
uv run python hexapod_walker/prototype_sts3215/apriltag_lids/check_clearance.py
```

The clearance check includes the flush screw heads and performs exact
mesh-intersection tests at 176 poses over the production yaw/hip/knee limits.
The closest audited approach is also required to remain at least 0.40 mm.

Print tag-side up, without supports, using matte black and matte white. After
installation, manually articulate the unpowered joint and confirm that the
yoke does not rub before operating the robot.
