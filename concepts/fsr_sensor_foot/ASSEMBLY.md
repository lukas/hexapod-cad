# Angle-tolerant RP-C10 foot assembly

## Print one test set

1. Print `fsr_foot_housing.stl` in PETG or PA. Use supports under the sensor
   roof and terminal lobe; remove every strand from the three guide bores.
2. Print `fsr_guided_carriage.stl` in PETG or PA. Favor guide-pin strength over
   cosmetics; a side orientation with organic supports is safer than putting
   the pin roots on weak layer lines.
3. Print `fsr_tpu_tread.stl` in TPU 95A at fine layers. The spherical outside
   must remain smooth.
4. Print `fsr_tpu_sensing_spring.stl` flat in TPU 95A. Confirm that all three
   spokes are fused to both the center platform and outer ring.
5. Prefer a smooth 7.5 × 0.25 mm PET shim for the spreader. The supplied STL is
   a sizing reference if a reliable 0.25 mm print is possible.

## Dry-fit the mechanical carriage first

1. With no sensor or spreader fitted, align the three carriage pins to the
   housing bores and push the 19.6 mm flange through the split 19.2 mm collar.
2. The flange should click past the lip with modest hand pressure. Do not use
   pliers; open the split or lightly deburr the lip if it takes real force.
3. Turn the housing upright. The carriage must fall back to the retention lip
   under its own weight or a very light tap. Polish the pins or ream the bores
   evenly until it does. Side-load sensing will not work if this test fails.
4. Push the carriage upward and feel a broad, definite stop after approximately
   0.25 mm. The guide pins must not bottom in their bores.
5. Pull the bare carriage back out of the housing. Do not fit the wide TPU
   tread yet; it is installed only after the carriage is snapped into the
   housing for the final time.
6. Seat the sensing spring's outer ring on the shelf in the carriage recess.
   Its center and spokes must float above the deeper 0.60 mm relief pocket.

## Install the sensor

1. Solder fine flexible wires before installation and strain-relieve the joint
   outside the film neck. Keep heat brief.
2. Feed the terminals through the radial lobe from below. Lay the 10 mm head
   flat in its pocket and retain only the inactive rim/terminal area.
3. Point the tail lobe toward the chassis/uphill side of the tibia. This is
   essential: the spherical tread accepts any azimuth, but the long flat tail
   protector must stay away from the downhill contact side.
4. Center the smooth 7.5 mm spreader on the TPU spring's center platform, then
   insert the carriage from below, guide pins first, and snap its rigid flange
   into the housing. The spreader must land under the 8 mm active circle.
5. Warm the TPU tread in hot tap water. From below the now-retained carriage,
   stretch its 18.04 mm return-lip throat over the 19.04 mm rigid rim. Work
   around the perimeter until the lip drops into the recessed neck with no
   rolled or pinched areas. The 0.50 mm radial undercut is the retention
   feature; do not glue the first prototype. Pull firmly around the full edge
   to confirm it cannot peel off.
6. Add smooth PET/Kapton shim only as needed to obtain a repeatable unloaded
   reading and early contact response. Never preload the FSR heavily.

## Fit the tibia

1. Remove the existing boot; do not cut the 142 mm tube.
2. Slide the housing on until the tube bottoms at the 8 mm station.
3. Rotate the sensor lobe toward the chassis and tighten one 2.5 mm zip tie in
   the sleeve groove.
4. Route the wire pair uphill with a service loop at the knee and keep it clear
   of the yoke and horn sweep.

## Bench gate before walking

1. Log raw ADC through a voltage divider; 10 kΩ is only a starting value.
2. Test the foot against a flat board at 0°, 40°, and 50°. At every angle,
   contact must trigger before the rigid housing or tail lobe touches the board.
3. Repeat at least 50 cycles at 40° while changing the contact azimuth. The
   unloaded value must recover and the trigger must remain repeatable.
4. Increase load only until the broad carriage stop is obvious. Do not push the
   full robot transient through the film; the recorded 44.7 N peak exceeds the
   sensor's listed 19.6 N ceiling.
5. Fit one instrumented leg for the first walking trace and compare it to video
   before printing five more.
