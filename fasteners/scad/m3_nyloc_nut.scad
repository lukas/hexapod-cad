// fasteners/scad/m3_nyloc_nut.scad
//
// DIN 985 M3 nylon-insert lock nut (nyloc).  McMaster-Carr 90576A102
// (A2 stainless).  Used as the captive nut for every M3 bolt joint
// in the build (cradle SHCS, chassis SHCS, foot hinge pin).  Mirrors
// SPEC_M3_NYLOC in fastener_registry.py.
//
// NopSCADlib output convention (raw):
//   * Steel body: z in [0, nut_thickness]    (~2.4 mm)
//   * Nylon collar: z in [nut_thickness, nyloc_thickness]  (~4.0 mm total)
// make_fastener_meshes.py mirrors across the XY plane so the cached
// STL matches the parametric convention:
//   * Visible outer face at z = 0
//   * Body extends into z < 0 (= INTO the printed hex pocket)
// See fasteners/README.md section "Axis convention" for why.
include <NopSCADlib/utils/core/core.scad>
include <NopSCADlib/vitamins/nuts.scad>

$fn = 32;
nut(M3_nut, nyloc=true);
