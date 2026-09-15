// fasteners/scad/m3x16_pan.scad
//
// ISO 7045 / DIN 7985 M3 pan-head Phillips screw, length 16 mm.
// McMaster-Carr 92010A130 (A2 stainless).  Used as the foot hinge
// pin (one per leg).  Mirrors SPEC_M3X16_PAN in fastener_registry.py.
//
// NopSCADlib's M3_pan_screw type has a flat round head (no Phillips
// cross slot in the rendered geometry, since NopSCADlib renders
// drivers separately).  Visually adequate for inspector zoom.
include <NopSCADlib/utils/core/core.scad>
include <NopSCADlib/vitamins/screws.scad>

$fn = 32;
screw(M3_pan_screw, 16);
