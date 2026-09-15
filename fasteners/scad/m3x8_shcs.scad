// fasteners/scad/m3x8_shcs.scad
//
// ISO 4762 / DIN 912 M3 socket-head cap screw, length 8 mm.
// McMaster-Carr 91290A113 (black-oxide steel).  Mirrors the
// SPEC_M3X8_SHCS entry in fastener_registry.py.  NopSCADlib emits
// the head at +Z and the shank at -Z; make_fastener_meshes.py's
// _orient_for_registry() reflects across the XY plane on cache
// write so the on-disk STL puts the head at z<0 and the shank at
// z>0, matching the FastenerInstance "+Z = into the material"
// convention (see fasteners/README.md).
include <NopSCADlib/utils/core/core.scad>
include <NopSCADlib/vitamins/screws.scad>

$fn = 32;
screw(M3_cap_screw, 8);
