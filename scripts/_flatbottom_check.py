"""Read-only flat-bottom audit for chassis_bottom + yaw_bearing_cap.

Builds each part, applies the SAME print-orientation reorient used by
print_orientation.PART_REGISTRY, then measures whether the part rests
on a genuine flat face at min-Z or whether features protrude below the main
bottom plane (support-needing overhangs).
"""
from __future__ import annotations

import os
import sys

import numpy as np
import trimesh

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROTO_DIR = os.path.dirname(SCRIPT_DIR)
for _p in (PROTO_DIR, SCRIPT_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import hexapod_prototype as hp  # noqa: E402
import print_orientation as px  # noqa: E402


def _down_face_area_by_z(mesh, z_tol=0.20, nz_tol=-0.90):
    """Return list of (z_level, area) clusters for downward-facing faces."""
    fn = mesh.face_normals
    fa = mesh.area_faces
    tri = mesh.triangles  # (F,3,3)
    face_z = tri[:, :, 2].mean(axis=1)
    down = fn[:, 2] < nz_tol
    zlev = face_z[down]
    area = fa[down]
    if len(zlev) == 0:
        return []
    order = np.argsort(zlev)
    zlev = zlev[order]
    area = area[order]
    clusters = []
    cz = zlev[0]
    ca = 0.0
    members = []
    for z, a in zip(zlev, area):
        if z - cz <= z_tol:
            ca += a
            members.append(z)
            cz = np.mean(members)
        else:
            clusters.append((cz, ca))
            cz = z
            ca = a
            members = [z]
    clusters.append((cz, ca))
    return clusters


def _footprint_area(mesh):
    """XY convex-hull area of the projected vertices."""
    pts = mesh.vertices[:, :2]
    try:
        from scipy.spatial import ConvexHull
        hull = ConvexHull(pts)
        return hull.volume  # 2D hull "volume" == area
    except Exception:
        lo = pts.min(axis=0)
        hi = pts.max(axis=0)
        return float((hi[0] - lo[0]) * (hi[1] - lo[1]))


def analyze(name, mesh):
    lo, hi = mesh.bounds
    zmin = float(lo[2])
    zmax = float(hi[2])
    print(f"\n========== {name} ==========")
    print(f"  bounds (mm): X[{lo[0]:.1f},{hi[0]:.1f}] "
          f"Y[{lo[1]:.1f},{hi[1]:.1f}] Z[{zmin:.2f},{zmax:.2f}]")
    print(f"  Z height = {zmax - zmin:.2f} mm")
    foot = _footprint_area(mesh)
    print(f"  XY footprint (convex hull) = {foot:.0f} mm^2")

    clusters = _down_face_area_by_z(mesh)
    clusters.sort(key=lambda c: c[0])
    print("  Downward-facing planar face area, by Z level (bottom -> up):")
    for z, a in clusters:
        flag = ""
        if abs(z - zmin) < 0.5:
            flag = "  <-- ON BUILD PLATE (z_min)"
        print(f"      z = {z:8.2f} mm   area = {a:8.1f} mm^2{flag}")

    # bed-contact area = downward faces within 0.5 mm of z_min
    bed_area = sum(a for z, a in clusters if abs(z - zmin) < 0.5)
    print(f"  Bed-contact (z_min) downward face area = {bed_area:.0f} mm^2")
    print(f"  Bed-contact / footprint = {100.0 * bed_area / foot:.1f}%")

    # features that hang below the *main* (largest) downward plane
    if clusters:
        main_z, main_a = max(clusters, key=lambda c: c[1])
        print(f"  Largest downward plane: z = {main_z:.2f} mm "
              f"(area {main_a:.0f} mm^2)")
        below = [(z, a) for z, a in clusters if z < main_z - 0.5]
        if below:
            drop = main_z - min(z for z, _ in below)
            print(f"  !! Features BELOW the main plane (would need support):")
            for z, a in below:
                print(f"        z = {z:7.2f} mm  (={main_z - z:5.2f} mm below "
                      f"main plane)  area {a:.1f} mm^2")
            print(f"  !! Max protrusion below main plane = {drop:.2f} mm")
        else:
            print("  No downward faces below the main plane -> FLAT on plate.")
    return clusters


def flatness_metrics(mesh, *, z_tol=0.20, plate_tol=0.5):
    """Return print-orientation flat-bottom metrics for ``mesh`` (already in
    its print pose, resting on z_min).

    Keys:
      * ``footprint``            -- XY convex-hull area (mm^2)
      * ``bed_area``             -- downward face area within 0.5 mm of z_min
      * ``bed_fraction``         -- bed_area / footprint
      * ``main_z`` / ``main_area`` -- the largest downward plane
      * ``protrusion_below_main``  -- how far the LOWEST downward face sits
        below the largest downward plane (0.0 == flat: nothing hangs below
        the main bed plane).  This is the support-needing-overhang metric.

    Used by the verifier's flat-bottom printability guard
    (``check_flat_bottom``).
    """
    lo, hi = mesh.bounds
    zmin = float(lo[2])
    foot = _footprint_area(mesh)
    clusters = _down_face_area_by_z(mesh, z_tol=z_tol)
    bed_area = sum(a for z, a in clusters if abs(z - zmin) < plate_tol)
    main_z, main_area = (max(clusters, key=lambda c: c[1])
                         if clusters else (zmin, 0.0))
    below = [(z, a) for z, a in clusters if z < main_z - plate_tol]
    protrusion = (main_z - min(z for z, _ in below)) if below else 0.0
    return {
        "footprint": float(foot),
        "bed_area": float(bed_area),
        "bed_fraction": float(bed_area / foot) if foot > 0 else 0.0,
        "main_z": float(main_z),
        "main_area": float(main_area),
        "protrusion_below_main": float(protrusion),
    }


def reorient(name, make_fn):
    mesh = make_fn()
    for (fname, mk, ro, *_rest) in px.PART_REGISTRY:
        if mk is make_fn:
            return ro(mesh)
    return mesh


for nm, mk in (("chassis_bottom (print orient)", hp.make_chassis_bottom),
               ("yaw_bearing_cap (print orient)", hp.make_yaw_bearing_cap)):
    m = reorient(nm, mk)
    analyze(nm, m)

# Also show chassis_bottom if it were simply FLIPPED 180 about X (cradles up)
print("\n\n===== WHAT-IF: chassis_bottom flipped 180 about X (cradles UP) =====")
cb = hp.make_chassis_bottom()
cb.apply_transform(trimesh.transformations.rotation_matrix(np.pi, (1, 0, 0)))
cb.apply_translation([0, 0, -float(cb.bounds[0, 2])])
analyze("chassis_bottom FLIPPED", cb)
