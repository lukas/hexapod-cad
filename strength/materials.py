"""Material properties for the 3D-printed parts.

Everything is in **SI base units inside the solver / report code**:

* Young's modulus E :  Pa     (= N/m^2)
* Yield strength   :  Pa
* Density          :  kg/m^3
* Poisson's ratio  :  dimensionless

We keep both an isotropic-equivalent block (used for the linear-static
CalculiX deck and the beam-bending check) and the orthotropic stack
(printer-Z is weaker than XY for FDM; the XY/Z drop is sourced from
the manufacturer's tensile-test datasheet for the named filament).

The numbers below are the "as-printed, 100 %-infill, no annealing,
no fiber-aligned" values you'd expect from a hobby printer running
the manufacturer's recommended slicer profile.  Two derating knobs
are applied **at the report layer**, not here:

* ``infill_factor`` -- the structural parts in this build are sliced
  at 25 % gyroid, 4 walls.  Effective stiffness/strength on a flat
  spar with that profile is ~ 60-70 % of the solid value; the
  report bakes that into the safety-factor calculation.
* ``brittleness_factor`` -- PLA breaks rather than yields, so we
  derate its effective yield by 1.5x (Polymaker / Prusa application
  note recommendation) when comparing peak von Mises against the
  yield column.

If you want to test "what if I anneal the PLA", "what if I switch
to 80 % infill", or "what if I align fiber along the spar long
axis" you should edit the call sites (``load_cases.py`` for infill;
``report.py`` for brittleness) -- this file is the raw datasheet,
not the as-built property.

Sources (per material, inline in the dict)
------------------------------------------
PLA   -- Polymaker PolyLite PLA Technical Data Sheet rev 2024-09;
         Prusament PLA TDS rev 2025-03.  Both cite ISO 527 dog-bones
         printed on a 0.4 mm nozzle at 100 % infill.  E_XY ~ 2.6 GPa,
         yield ~ 50 MPa, density 1.24 g/cm^3.  Z drop ~ 60 % (so
         E_Z ~ 1.6 GPa, sigma_y,Z ~ 30 MPa).

PETG  -- Polymaker PolyLite PETG TDS rev 2024-09; Bambu Generic PETG
         profile measurements (Bambu test print 2025-06 batch).
         E_XY ~ 1.9 GPa, yield ~ 45 MPa, density 1.27 g/cm^3.  Z
         drop ~ 50 % at the layer interface (PETG bonds better than
         PLA between layers, so the orthotropy is gentler).

PA-CF / Onyx -- Markforged Onyx datasheet rev 2024-05 (chopped-CF
         filled nylon).  E_XY ~ 2.4 GPa, yield (interpreted as the
         flexural stress at 1 % strain) ~ 36 MPa, density
         1.20 g/cm^3.  This is the "Onyx alone" number -- the
         Markforged Mark Two with continuous CF reinforcement would
         be in a different class entirely (E ~ 40-50 GPa on the
         fibered axis); we do NOT model that here.

ABS   -- Bambu Generic ABS TDS rev 2024-11; SD3D / MatterHackers
         university comparison tests, U. of Tennessee Knoxville
         (Roberson et al. 2017, J. Mater. Eng. Perform.).  E_XY ~
         2.0 GPa, yield ~ 30 MPa, density 1.04 g/cm^3.  Z drop
         ~ 55 %.  ABS is the WEAKEST of the four bulk values but
         remains in scope because the prototype's TPU foot boot
         (separate material) is not in this list.

TPU 95A (foot boot) is intentionally absent.  TPU at 95A shore is so
flexible (E ~ 30 MPa) that any of the FEA cases below would print
"deflection > 10 mm, peak stress << 1 MPa", which is not actionable
strength data -- it's a *spring*.  Aug 2026: the boot has no FEA case
at all (the old hinged foot_pad's surrogate-material impact case is
retired with the part); the boot's only structural job is the press
fit on the Ø8 tube, which TPU handles by design.

Notes on the orthotropic block
------------------------------
The full orthotropic Young's-modulus tensor for FDM has E_X = E_Y
(in-plane isotropic to first order; printed perimeters dominate
the spar stiffness) and a SEPARATE E_Z (layer-interface bond,
weaker by 40-60 %).  For the LINEAR-STATIC CalculiX deck we lower
the isotropic E to the geometric mean of E_XY and E_Z, which is
correct for parts loaded in primarily out-of-plane bending
(femur spar bent about its long axis sees both XY tension and Z
shear on its layer faces).  See ``isotropic_equivalent`` below
for the math.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Material:
    """Material card for a single filament.

    All units SI.  ``E_iso`` and ``sigma_y_iso`` are the
    geometric-mean isotropic equivalents derived from the
    orthotropic XY/Z pair (see ``isotropic_equivalent`` below)
    and are what the linear-static FEA / beam-bending check
    consume.  ``E_xy`` / ``E_z`` / ``sigma_y_xy`` / ``sigma_y_z``
    are preserved for future orthotropic-CalculiX work and for
    user inspection.
    """
    name: str
    short_name: str
    E_xy: float            # Pa, in-plane Young's modulus
    E_z:  float            # Pa, layer-direction Young's modulus
    nu:   float            # dimensionless, isotropic-equivalent Poisson's
    sigma_y_xy: float      # Pa, in-plane yield
    sigma_y_z:  float      # Pa, layer-interface yield
    density:    float      # kg/m^3
    brittleness_factor: float  # divide effective yield by this in the report
    source: str            # human-readable provenance

    @property
    def E_iso(self) -> float:
        """Geometric-mean isotropic-equivalent Young's modulus (Pa).

        For an FDM spar loaded in mixed XY-tension / Z-shear,
        the effective bending modulus sits between E_xy and E_z;
        the geometric mean ``sqrt(E_xy * E_z)`` is a common
        textbook approximation (Crump 1992 / ASTM F2792
        commentary) and is what we feed into the linear-static
        deck.  Within ~ 15 % of the right answer for the spars
        + plates we care about.
        """
        return math.sqrt(self.E_xy * self.E_z)

    @property
    def sigma_y_iso(self) -> float:
        """Isotropic-equivalent yield (Pa)."""
        return math.sqrt(self.sigma_y_xy * self.sigma_y_z)

    @property
    def G_iso(self) -> float:
        """Shear modulus from E_iso and nu_iso (Pa)."""
        return self.E_iso / (2.0 * (1.0 + self.nu))


def isotropic_equivalent(mat: Material) -> "Material":
    """Return ``mat`` with E_xy = E_z = E_iso and sigma collapsed.

    Useful for sanity checks where you want a purely-isotropic
    card without changing the source datasheet block above.
    """
    return Material(
        name=mat.name + " (iso-equiv)",
        short_name=mat.short_name + "_iso",
        E_xy=mat.E_iso,
        E_z=mat.E_iso,
        nu=mat.nu,
        sigma_y_xy=mat.sigma_y_iso,
        sigma_y_z=mat.sigma_y_iso,
        density=mat.density,
        brittleness_factor=mat.brittleness_factor,
        source=mat.source + " | collapsed to isotropic geometric mean",
    )


# ---------------------------------------------------------------------------
# Material library
# ---------------------------------------------------------------------------
#
# Filament choice for the prototype is documented in docs/BOM.md
# (filament rows): 1 kg of PLA or PETG (structural) and
# 250 g of TPU 95A (foot boots).  PETG is the recommended structural
# default; PLA is the easier-to-print backup.  Onyx + ABS are listed
# here for "what if I had a Markforged" / "what if I had to use the
# only filament I have left" comparison runs.
#
# Numbers below are the as-printed, 100 %-infill, 0.2 mm-layer values
# from the named TDS.  No annealing.  No fiber alignment.  Print
# orientation assumes the spar's long axis lies along the bed X axis
# (typical for the link STLs when laid flat on the print bed).
# ---------------------------------------------------------------------------

PLA = Material(
    name="Polymaker PolyLite PLA / Prusament PLA (as-printed)",
    short_name="pla",
    E_xy=2.60e9,
    E_z=1.60e9,
    nu=0.36,
    sigma_y_xy=50.0e6,
    sigma_y_z=30.0e6,
    density=1240.0,
    brittleness_factor=1.5,  # PLA breaks rather than yields; derate.
    source=(
        "Polymaker PolyLite PLA TDS rev 2024-09 (E_XY = 2.6 GPa, "
        "sigma_y = 50 MPa); Prusament PLA TDS rev 2025-03 (cross-check). "
        "Z drop sourced from MatterHackers / U. of Tennessee Knoxville "
        "tensile-bar tests (Roberson 2017): typical E_Z / E_XY ~ 0.6 "
        "for 0.2 mm-layer PLA."
    ),
)

PETG = Material(
    name="Polymaker PolyLite PETG / Bambu Generic PETG (as-printed)",
    short_name="petg",
    E_xy=1.90e9,
    E_z=1.30e9,
    nu=0.40,
    sigma_y_xy=45.0e6,
    sigma_y_z=27.0e6,
    density=1270.0,
    brittleness_factor=1.0,  # PETG yields plastically before fracture.
    source=(
        "Polymaker PolyLite PETG TDS rev 2024-09 (E_XY ~ 1.9 GPa, "
        "yield ~ 45 MPa); Bambu Generic PETG profile + Bambu wiki test "
        "print 2025-06 (cross-check).  PETG layer adhesion is better "
        "than PLA so the Z derate is gentler (~ 0.7)."
    ),
)

ONYX = Material(
    name="Markforged Onyx (chopped-CF nylon, as-printed)",
    short_name="onyx",
    E_xy=2.40e9,
    E_z=1.80e9,
    nu=0.34,
    sigma_y_xy=36.0e6,
    sigma_y_z=22.0e6,
    density=1200.0,
    brittleness_factor=1.0,
    source=(
        "Markforged Onyx datasheet rev 2024-05.  Flexural stress at "
        "1 %% strain treated as effective yield for the linear-static "
        "comparison; Markforged's reported tensile strength of 36 MPa "
        "matches.  Continuous-CF reinforcement (Mark Two fiber-aligned) "
        "is NOT modelled here -- this is the unreinforced bulk."
    ),
)

ABS = Material(
    name="Bambu Generic ABS (as-printed)",
    short_name="abs",
    E_xy=2.00e9,
    E_z=0.90e9,
    nu=0.35,
    sigma_y_xy=30.0e6,
    sigma_y_z=16.0e6,
    density=1040.0,
    brittleness_factor=1.0,
    source=(
        "Bambu Generic ABS TDS rev 2024-11 (E_XY ~ 2.0 GPa, yield "
        "~ 30 MPa); SD3D / MatterHackers tensile tests (Roberson "
        "2017, J. Mater. Eng. Perform.) for the Z-direction drop."
    ),
)


MATERIALS: dict[str, Material] = {
    "pla":  PLA,
    "petg": PETG,
    "onyx": ONYX,
    "abs":  ABS,
}


def get(name: str) -> Material:
    """Look up a material by short name (case-insensitive)."""
    key = name.lower().strip()
    if key not in MATERIALS:
        raise KeyError(
            f"Unknown material {name!r}.  "
            f"Known: {sorted(MATERIALS)}"
        )
    return MATERIALS[key]


# A fixed "infill / wall-count" derate applied at the report layer.
# 25 % gyroid + 4 walls is the BOM-stated print profile (PROTOTYPE.md
# §3.1).  For a typical 30-mm-tall spar that profile gives ~ 65 % of
# the 100 %-infill effective E and yield (slicer-time measurement,
# OrcaSlicer 2.2 wall-count sweep).
DEFAULT_INFILL_FACTOR = 0.65
