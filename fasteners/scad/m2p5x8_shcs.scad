// fasteners/scad/m2p5x8_shcs.scad
//
// ISO 4762 / DIN 912 M2.5 socket-head cap screw, length 8 mm.
// McMaster-Carr 91290A104 (black-oxide steel).  Used as the servo
// spline centre screw (ships with the servo).  Mirrors
// SPEC_M25X8_SHCS in fastener_registry.py.
include <NopSCADlib/utils/core/core.scad>
include <NopSCADlib/vitamins/screws.scad>

$fn = 32;
screw(M2p5_cap_screw, 8);
