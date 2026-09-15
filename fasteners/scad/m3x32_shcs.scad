// fasteners/scad/m3x32_shcs.scad
//
// ISO 4762 / DIN 912 M3 socket-head cap screw, nominal length 32 mm
// in the fastener_registry; rendered here at 30 mm because the BOM's
// McMaster part number (91290A123) is the 30 mm stock -- closest
// off-the-shelf to the budgeted 32 mm.  See fasteners/README.md
// section "Why 91290A123 for the M3 x 32 chassis bolts" for the
// length-substitution rationale.
include <NopSCADlib/utils/core/core.scad>
include <NopSCADlib/vitamins/screws.scad>

$fn = 32;
screw(M3_cap_screw, 30);
