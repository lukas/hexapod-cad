# Assembly — TT-motor kid 4x4 classic pickup truck

Adult supervision is required for the first build and every powered test.
Assemble and wire with the battery disconnected.

## 1. Print the two small gauges first

1. Print `l298n_fit_gauge.stl`.  Place one red board over its four 2.5 mm
   pins.  All four board holes should slip over the pins without force.
2. Print `motor_fit_coupon.stl` and one `motor_clamp.stl`.  Put the yellow
   gearbox in the shallow pocket, place the clamp across it, and start two
   M3 × 12 screws.  Tighten alternately until the motor cannot slide.  Stop as
   soon as it is secure; the clamp is not meant to crush the gearbox.
3. If either gauge is wrong, measure the real part with calipers and edit the
   clearly named constants at the top of `build_tt_truck.py` before printing
   the large parts.

## 2. Install all four motors

1. Put the lower chassis flat, raised posts upward, with the bumper bars at
   front and rear.
2. Place a motor in each shallow side pocket.  The white wheel shaft points
   outward and the wire end points toward the open centre of the truck.
3. Lay one identical clamp across each gearbox.  The small centre pad faces
   down; the smooth broad face faces up.
4. Start the two M3 × 12 screws in each clamp by hand.  Alternate between the
   screws until the clamp just holds the motor.  Do not fully tighten one side
   first and do not use a power driver.

## 3. Add the wheels

Support the motor shaft with one hand and press each wheel straight onto the
white D-shaped shaft with the other.  Never use the chassis as the reaction
force; side-loading can damage the little plastic gearbox.  Spin every wheel
by hand and check that it misses the nearest clamp boss.

## 4. Install the upper deck and driver boards

1. Put the electronics deck on the four centre posts.  The 44 × 68 mm battery
   area with strap slots goes toward the rear.
2. Install four M3 × 12 screws through the deck into the chassis posts.
3. Place both L298N boards on the eight short standoffs.  Orient their blue
   motor terminals toward an accessible edge.
4. Install the eight M2.5 × 8 board screws.  Tighten only until the board stops
   moving; PCB material and printed threads are easy to strip.
5. Feed two hook-and-loop straps through the rear deck slots and secure the
   switched battery holder.  No cell or bare metal contact may touch a screw,
   board underside, or motor terminal.

## 5. Wiring and first test

The two red boards are power drivers, not a radio receiver or autonomous
controller.  An adult should wire the selected controller, common ground,
logic supply, battery switch, and four motor channels according to that
controller's instructions.

For the first powered test, put the truck on a block so all four wheels are in
the air.  Use a current-limited supply if available.  Briefly test one motor at
a time, then disconnect power and correct polarity so all four agree on
forward.  Stop immediately for hot wires, a hot driver, a stalled wheel,
smoke, odor, or a swelling/leaking battery.

## 6. Add the removable pickup body

1. Lower `truck_body_lower.stl` over the deck.  The tall grille and hood point
   forward; the open cargo bed points toward the battery zone.  Confirm that
   all four wheel arches clear the tyres.
2. Install four M3 × 10 screws through the corner tabs into the raised deck
   bosses.  Tighten only until the shell no longer rattles.
3. Put `truck_hood.stl` over the front electronics bay and install four
   M3 × 10 screws.  The shallow raised panel faces upward.
4. Put `truck_roof.stl` over the cab frame with its visor toward the hood and
   install the final four M3 × 10 screws.
5. The dark windows, grille, headlights, and tail lights in BuildViz are
   appearance suggestions rather than extra printed parts.  Leave the windows
   open and use paint, tape, or stickers for those details if desired.

## Child handoff check

- Every motor and board is snug, but no printed boss is cracked.
- Wheels spin without touching the chassis.
- Wheels also clear all four pickup-body arches.
- The battery is enclosed, strapped, switched, and has no exposed conductor.
- Wires cannot reach the tyres.
- A low-speed limit is configured before floor testing.
