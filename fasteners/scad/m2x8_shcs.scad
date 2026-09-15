// fasteners/scad/m2x8_shcs.scad
//
// ISO 4762 / DIN 912 M2 socket-head cap screw, length 8 mm.
// McMaster-Carr 91290A005 (black-oxide steel).  Used as the
// link-to-X-horn self-tap bolt (May 2026 fastener-spec fix; see
// hexapod_prototype.py XHORN_BOLT_* docstring + fasteners/README.md
// for the optional 99461A340 thread-forming upgrade path).
//
// Mirrors the SPEC_M2X8_SHCS entry in fastener_registry.py.
// NopSCADlib emits the head at +Z and the shank at -Z;
// make_fastener_meshes.py's _orient_for_registry() reflects across
// the XY plane on cache write so the on-disk STL puts the head at
// z<0 and the shank at z>0, matching the FastenerInstance "+Z = into
// the material" convention.
include <NopSCADlib/utils/core/core.scad>
include <NopSCADlib/vitamins/screws.scad>

$fn = 32;
screw(M2_cap_screw, 8);
