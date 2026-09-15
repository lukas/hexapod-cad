#!/usr/bin/env python3
"""Build two-colour AprilTag covers for the STS3215 servo clamp cap.

The printable is a 1.4 mm face plate whose two M3 holes inherit their
coordinates from ``hexapod_prototype.servo_clamp_bolt_centres``.  A white
0.6 mm structural base supports a coplanar 0.8 mm black/white tag skin.
The resulting 3MF contains separate white and black components with material
metadata; the aligned STL pair is also emitted for slicers that prefer a
multi-part import.

Default IDs 1..16 include the 12 configured motor-housing IDs plus four
additional numbered covers:
  1..6  -> L0..L5 coxa / hip-servo housings
  7..12 -> L0..L5 femur / knee-servo housings
  13..16 -> extra covers; not assigned to clamp caps in the current pose config

The official tag36h11 codes and bit coordinates come from AprilRobotics:
https://github.com/AprilRobotics/apriltag/blob/master/tag36h11.c
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
import subprocess
import tempfile
import zipfile
from pathlib import Path
from xml.etree import ElementTree as ET
from xml.sax.saxutils import escape

import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyBboxPatch, Rectangle
import numpy as np
from shapely.affinity import affine_transform
from shapely.geometry import MultiPolygon, Point, Polygon, box
from shapely.ops import unary_union
import trimesh


HERE = Path(__file__).resolve().parent

import hexapod_prototype as hp


# Official tag36h11 codedata[0..18].
TAG36H11_CODES = [
    0x0000000D7E00984B,
    0x0000000DDA664CA7,
    0x0000000DC4A1C821,
    0x0000000E17B470E9,
    0x0000000EF91D01B1,
    0x0000000F429CDD73,
    0x000000005DA29225,
    0x00000001106CBA43,
    0x0000000223BED79D,
    0x000000021F51213C,
    0x000000033EB19CA6,
    0x00000003F76EB0F8,
    0x0000000469A97414,
    0x000000045DCFE0B0,
    0x00000004A6465F72,
    0x000000051801DB96,
    0x00000005EB946B4E,
    0x000000068A7CC2EC,
    0x00000006F0BA2652,
]

# Official codedata[100..119], used by the second 20-cover plate.  Keeping the
# high-ID block separate avoids embedding unused IDs 19..99 in this generator.
TAG36H11_EXTRA_CODES = {
    100: 0x0000000533FE2404,
    101: 0x000000063DE7D35E,
    102: 0x0000000925EDDC72,
    103: 0x000000099B8B3896,
    104: 0x0000000AACE4C708,
    105: 0x0000000C22994AF0,
    106: 0x00000008F1EAE41B,
    107: 0x0000000D95FB486C,
    108: 0x000000013FB77857,
    109: 0x00000004FE0983A3,
    110: 0x0000000D559BF8A9,
    111: 0x0000000E1855D78D,
    112: 0x0000000FEC8DAAAD,
    113: 0x000000071ECB6D95,
    114: 0x0000000DC9E50E4C,
    115: 0x0000000CA3A4C259,
    116: 0x0000000740D12BBF,
    117: 0x0000000AEEDD18E0,
    118: 0x0000000B509B9C8E,
    119: 0x00000005232FEA1C,
}

BIT_X = [
    1, 2, 3, 4, 5, 2, 3, 4, 3, 6, 6, 6, 6, 6, 5, 5, 5, 4,
    6, 5, 4, 3, 2, 5, 4, 3, 4, 1, 1, 1, 1, 1, 2, 2, 2, 3,
]
BIT_Y = [
    1, 1, 1, 1, 1, 2, 2, 2, 3, 1, 2, 3, 4, 5, 2, 3, 4, 3,
    6, 6, 6, 6, 6, 5, 5, 5, 4, 6, 5, 4, 3, 2, 5, 4, 3, 4,
]

NBITS = 36
WIDTH_AT_BORDER = 8
TOTAL_WIDTH = 10  # one-cell white quiet zone around the 8x8 bordered code

# Accessory geometry, millimetres.  Its local XY is the clamp-cap XZ face:
# accessory X == cap X; accessory Y + CLAMP_BOLT_Z == cap Z.
PLATE_W = 61.8                               # inset 0.8/side from 63.4 cap
PLATE_H = float(hp.SERVO_BODY_H)             # 34.3
CORNER_R = 2.0
BASE_T = 0.6
SKIN_T = 0.8
TOTAL_T = BASE_T + SKIN_T                    # 1.4
TAG_SIZE = 34.0
CELL = TAG_SIZE / TOTAL_WIDTH                 # 3.4
LABEL_PIXEL = 0.6
LABEL_PIXEL_GAP = 0.12
LABEL_DIGIT_W = 3 * LABEL_PIXEL + 2 * LABEL_PIXEL_GAP
LABEL_DIGIT_H = 5 * LABEL_PIXEL + 4 * LABEL_PIXEL_GAP
LABEL_GAP = 0.6
LABEL_CENTER_V = -20.5
# The moving yoke's exact-BREP sweep clips the last ~0.9 mm of a full-height
# skin at extreme negative hip pitch.  The structural base is already inset
# 0.8 mm/side from the production cap.  Stop the broad colour skin at |x|=29.4
# and retain only a small circular skin pad at each screw; the pad reaches the
# flat-head rim but does not recreate a full-height yoke-catching edge.
SKIN_BODY_HALF_W = 29.4
SCREW_SKIN_R = 3.0

THROUGH_OD = float(hp.CLAMP_BOLT_CLEAR_OD)    # 3.4
CSK_TOP_OD = 6.0                              # ISO M3 90-degree flat head
CSK_DEPTH = (CSK_TOP_OD - THROUGH_OD) / 2.0   # 1.3 at 90 degrees
CSK_BOTTOM_Z = TOTAL_T - CSK_DEPTH            # 0.1 mm first-layer land
CSK_SECTIONS = 64

# The production cap's screw coordinates are the single source of truth.
BOLT_CENTRES = tuple(
    (float(x), float(z) - float(hp.CLAMP_BOLT_Z))
    for x, z in hp.servo_clamp_bolt_centres()
)

WHITE = "#F8F8F8"
BLACK = "#101010"

PIXEL_DIGITS = {
    "0": ("111", "101", "101", "101", "111"),
    "1": ("010", "110", "010", "010", "111"),
    "2": ("111", "001", "111", "100", "111"),
    "3": ("111", "001", "111", "001", "111"),
    "4": ("101", "101", "111", "001", "001"),
    "5": ("111", "100", "111", "001", "111"),
    "6": ("111", "100", "111", "101", "111"),
    "7": ("111", "001", "010", "010", "010"),
    "8": ("111", "101", "111", "101", "111"),
    "9": ("111", "101", "111", "001", "111"),
}

BAMBU_STUDIO = Path("/Applications/BambuStudio.app/Contents/MacOS/BambuStudio")
BAMBU_PLA_PROFILE = Path(
    "/Applications/BambuStudio.app/Contents/Resources/profiles/BBL/filament/"
    "Generic PLA @BBL A1.json"
)
BAMBU_MACHINE_PROFILE = Path(
    "/Applications/BambuStudio.app/Contents/Resources/profiles/BBL/machine/"
    "Bambu Lab A1 0.4 nozzle.json"
)
BAMBU_PROCESS_PROFILE = Path(
    "/Applications/BambuStudio.app/Contents/Resources/profiles/BBL/process/"
    "0.20mm Standard @BBL A1.json"
)

BAMBU_FILAMENT_KEY_ALIASES = {
    "type": "filament_type",
    "filament_deretraction_speed": "deretraction_speed",
    "filament_long_retractions_when_cut": "long_retractions_when_cut",
    "filament_long_retractions_when_ec": "long_retractions_when_ec",
    "filament_retract_before_wipe": "retract_before_wipe",
    "filament_retract_restart_extra": "retract_restart_extra",
    "filament_retract_when_changing_layer": "retract_when_changing_layer",
    "filament_retraction_distances_when_cut": "retraction_distances_when_cut",
    "filament_retraction_distances_when_ec": "retraction_distances_when_ec",
    "filament_retraction_length": "retraction_length",
    "filament_retraction_minimum_travel": "retraction_minimum_travel",
    "filament_retraction_speed": "retraction_speed",
    "filament_wipe": "wipe",
    "filament_wipe_distance": "wipe_distance",
    "filament_z_hop": "z_hop",
    "filament_z_hop_types": "z_hop_types",
}


def resolved_bambu_profile(path: Path) -> dict:
    """Resolve Bambu JSON inheritance for reliable command-line loading."""
    data = json.loads(path.read_text(encoding="utf-8"))
    inherited = data.get("inherits")
    if not inherited:
        return data
    parent = path.parent / f"{inherited}.json"
    if not parent.is_file():
        raise FileNotFoundError(
            f"Bambu profile {path.name} inherits missing {parent.name}"
        )
    merged = resolved_bambu_profile(parent)
    merged.update(data)
    merged.pop("inherits", None)
    return merged


def tag_grid(tag_id: int) -> list[list[int]]:
    """Return a 10x10 tag image; 0 is black and 1 is white."""
    if 0 <= tag_id < len(TAG36H11_CODES):
        code = TAG36H11_CODES[tag_id]
    elif tag_id in TAG36H11_EXTRA_CODES:
        code = TAG36H11_EXTRA_CODES[tag_id]
    else:
        raise ValueError(
            f"tag ID {tag_id} is not embedded; available ranges are "
            f"0..{len(TAG36H11_CODES)-1} and 100..119"
        )
    image = [[0] * TOTAL_WIDTH for _ in range(TOTAL_WIDTH)]
    for i in range(TOTAL_WIDTH):
        image[0][i] = 1
        image[-1][i] = 1
        image[i][0] = 1
        image[i][-1] = 1
    border_start = (TOTAL_WIDTH - WIDTH_AT_BORDER) // 2
    for i in range(NBITS):
        if code & (1 << (NBITS - i - 1)):
            x = BIT_X[i] + border_start
            y = BIT_Y[i] + border_start
            image[y][x] = 1
    return image


def rounded_outline() -> Polygon:
    raw = box(-PLATE_W / 2.0, -PLATE_H / 2.0,
              PLATE_W / 2.0, PLATE_H / 2.0)
    return raw.buffer(-CORNER_R, join_style="round").buffer(
        CORNER_R, join_style="round", quad_segs=12
    )


def portrait_to_print(geometry):
    """Rotate portrait artwork into the production cap's print XY frame.

    Portrait +V maps to print +X and portrait +U maps to print -Y.  Thus the
    production screw pair is visually above/below the tag while retaining its
    exact physical coordinates.
    """
    return affine_transform(geometry, [0.0, 1.0, -1.0, 0.0, 0.0, 0.0])


def black_cells_2d(tag_id: int):
    cells = []
    grid = tag_grid(tag_id)
    for row in range(TOTAL_WIDTH):
        for col in range(TOTAL_WIDTH):
            if grid[row][col] != 0:
                continue
            u0 = -TAG_SIZE / 2.0 + col * CELL
            # Image row zero is portrait +V / tag-top.
            v1 = TAG_SIZE / 2.0 - row * CELL
            cells.append(box(u0, v1 - CELL, u0 + CELL, v1))
    return portrait_to_print(unary_union(cells))


def id_label_portrait_2d(tag_id: int):
    """Return a compact pixel ID below the tag in portrait coordinates."""
    text = str(tag_id)
    total_w = len(text) * LABEL_DIGIT_W + (len(text) - 1) * LABEL_GAP
    origin_u = -total_w / 2.0
    origin_v = LABEL_CENTER_V - LABEL_DIGIT_H / 2.0
    pixels = []
    for index, digit in enumerate(text):
        du = origin_u + index * (LABEL_DIGIT_W + LABEL_GAP)
        for row, pattern in enumerate(PIXEL_DIGITS[digit]):
            for col, value in enumerate(pattern):
                if value == "0":
                    continue
                u0 = du + col * (LABEL_PIXEL + LABEL_PIXEL_GAP)
                v1 = (
                    origin_v + LABEL_DIGIT_H
                    - row * (LABEL_PIXEL + LABEL_PIXEL_GAP)
                )
                pixels.append(box(u0, v1 - LABEL_PIXEL,
                                  u0 + LABEL_PIXEL, v1))
    return unary_union(pixels)


def id_label_2d(tag_id: int):
    """Return the portrait ID label rotated into print coordinates."""
    return portrait_to_print(id_label_portrait_2d(tag_id))


def colour_skin_outline_2d():
    outline = rounded_outline()
    skin_regions = [
        box(-SKIN_BODY_HALF_W, -PLATE_H / 2.0,
            SKIN_BODY_HALF_W, PLATE_H / 2.0),
        *[Point(x, y).buffer(SCREW_SKIN_R, quad_segs=24)
          for x, y in BOLT_CENTRES],
    ]
    return outline.intersection(unary_union(skin_regions))


def _extrude_geometry(geometry, height: float, z0: float = 0.0) -> trimesh.Trimesh:
    polygons: list[Polygon]
    if isinstance(geometry, Polygon):
        polygons = [geometry]
    elif isinstance(geometry, MultiPolygon):
        polygons = list(geometry.geoms)
    else:
        polygons = [g for g in geometry.geoms if isinstance(g, Polygon)]
    meshes = []
    for polygon in polygons:
        if polygon.area <= 1e-9:
            continue
        mesh = trimesh.creation.extrude_polygon(polygon, height=height)
        if z0:
            mesh.apply_translation((0.0, 0.0, z0))
        meshes.append(mesh)
    if not meshes:
        raise ValueError("cannot extrude an empty polygon set")
    return trimesh.util.concatenate(meshes)


def _frustum(radius_bottom: float, radius_top: float,
             z_bottom: float, z_top: float,
             sections: int = CSK_SECTIONS) -> trimesh.Trimesh:
    """Closed conical frustum on +Z for a boolean countersink cutter."""
    angles = np.linspace(0.0, 2.0 * math.pi, sections, endpoint=False)
    bottom = np.column_stack((
        radius_bottom * np.cos(angles),
        radius_bottom * np.sin(angles),
        np.full(sections, z_bottom),
    ))
    top = np.column_stack((
        radius_top * np.cos(angles),
        radius_top * np.sin(angles),
        np.full(sections, z_top),
    ))
    vertices = np.vstack((bottom, top, [[0.0, 0.0, z_bottom],
                                        [0.0, 0.0, z_top]]))
    bottom_center = 2 * sections
    top_center = bottom_center + 1
    faces = []
    for i in range(sections):
        j = (i + 1) % sections
        faces.extend((
            [i, j, sections + j],
            [i, sections + j, sections + i],
            [bottom_center, j, i],
            [top_center, sections + i, sections + j],
        ))
    return trimesh.Trimesh(vertices=vertices, faces=np.asarray(faces), process=True)


def countersink_cutters() -> list[trimesh.Trimesh]:
    cutters = []
    for x, y in BOLT_CENTRES:
        through = trimesh.creation.cylinder(
            radius=THROUGH_OD / 2.0,
            height=TOTAL_T + 0.4,
            sections=CSK_SECTIONS,
        )
        through.apply_translation((x, y, TOTAL_T / 2.0))
        frustum = _frustum(
            THROUGH_OD / 2.0,
            CSK_TOP_OD / 2.0 + 0.01,
            CSK_BOTTOM_Z,
            TOTAL_T + 0.01,
        )
        frustum.apply_translation((x, y, 0.0))
        cutters.extend((through, frustum))
    return cutters


def flush_flat_head_envelopes() -> list[trimesh.Trimesh]:
    """Nominal M3 flat-head volumes, flush with the accessory face."""
    heads = []
    for x, y in BOLT_CENTRES:
        head = _frustum(
            THROUGH_OD / 2.0,
            CSK_TOP_OD / 2.0,
            CSK_BOTTOM_Z,
            TOTAL_T,
        )
        head.apply_translation((x, y, 0.0))
        heads.append(head)
    return heads


def _difference(mesh: trimesh.Trimesh,
                cutters: list[trimesh.Trimesh]) -> trimesh.Trimesh:
    # Manifold treats a disconnected input mesh as one operand; when most
    # islands do not touch a cutter it can discard untouched islands.  Cut
    # each watertight body independently and keep non-overlapping bodies as-is.
    results = []
    for body in mesh.split(only_watertight=False):
        relevant = []
        for cutter in cutters:
            lo = np.maximum(body.bounds[0], cutter.bounds[0])
            hi = np.minimum(body.bounds[1], cutter.bounds[1])
            if np.all(hi > lo):
                relevant.append(cutter)
        if not relevant:
            results.append(body)
            continue
        cut = trimesh.boolean.difference(
            [body, *relevant], engine="manifold", check_volume=False
        )
        if cut is None:
            raise RuntimeError("manifold boolean difference returned no mesh")
        results.append(cut)
    if not results:
        raise RuntimeError("boolean difference discarded every mesh body")
    return trimesh.util.concatenate(results)


def build_tag_meshes(tag_id: int) -> tuple[trimesh.Trimesh, trimesh.Trimesh]:
    """Return ``(white, black)`` material meshes in print coordinates."""
    outline = rounded_outline()
    skin_outline = colour_skin_outline_2d()
    tag_square = box(-TAG_SIZE / 2.0, -TAG_SIZE / 2.0,
                     TAG_SIZE / 2.0, TAG_SIZE / 2.0)
    label = id_label_2d(tag_id)
    outside_tag = skin_outline.difference(unary_union((tag_square, label)))
    cutters = countersink_cutters()
    white_parts = [
        _difference(_extrude_geometry(outline, BASE_T), cutters),
        _difference(_extrude_geometry(outside_tag, SKIN_T, BASE_T), cutters),
    ]
    black_parts = []
    grid = tag_grid(tag_id)
    for row in range(TOTAL_WIDTH):
        for col in range(TOTAL_WIDTH):
            u0 = -TAG_SIZE / 2.0 + col * CELL
            v1 = TAG_SIZE / 2.0 - row * CELL
            cell = portrait_to_print(
                box(u0, v1 - CELL, u0 + CELL, v1)
            )
            cell_mesh = _extrude_geometry(
                cell, SKIN_T, BASE_T
            )
            if grid[row][col] == 0:
                black_parts.append(cell_mesh)
            else:
                white_parts.append(cell_mesh)
    black_parts.append(_extrude_geometry(label, SKIN_T, BASE_T))
    white = trimesh.util.concatenate(white_parts)
    black = trimesh.util.concatenate(black_parts)
    return white, black


def combined_mesh(white: trimesh.Trimesh,
                  black: trimesh.Trimesh) -> trimesh.Trimesh:
    """Return the colour-independent, fully filled accessory solid.

    Building it directly avoids asking a boolean kernel to classify the many
    exactly coplanar black/white pixel boundaries.
    """
    del white, black
    base = _extrude_geometry(rounded_outline(), BASE_T)
    skin = _extrude_geometry(colour_skin_outline_2d(), SKIN_T, BASE_T)
    return trimesh.util.concatenate((
        _difference(base, countersink_cutters()),
        _difference(skin, countersink_cutters()),
    ))


def clearance_envelope(white: trimesh.Trimesh,
                       black: trimesh.Trimesh) -> trimesh.Trimesh:
    """Accessory plus the two installed flush screw-head envelopes."""
    result = trimesh.boolean.union(
        [combined_mesh(white, black), *flush_flat_head_envelopes()],
        engine="manifold",
        check_volume=False,
    )
    if result is None:
        raise RuntimeError("clearance-envelope union returned no mesh")
    return result


def _mesh_xml(object_id: int, name: str, mesh: trimesh.Trimesh,
              material_id: int, material_index: int) -> str:
    vertices = "\n".join(
        f'      <vertex x="{x:.6f}" y="{y:.6f}" z="{z:.6f}"/>'
        for x, y, z in mesh.vertices
    )
    triangles = "\n".join(
        f'      <triangle v1="{a}" v2="{b}" v3="{c}"/>'
        for a, b, c in mesh.faces
    )
    return (
        f'  <object id="{object_id}" name="{escape(name)}" type="model" '
        f'pid="{material_id}" pindex="{material_index}">\n'
        f"   <mesh>\n    <vertices>\n{vertices}\n    </vertices>\n"
        f"    <triangles>\n{triangles}\n    </triangles>\n   </mesh>\n"
        f"  </object>"
    )


def write_3mf(path: Path, tag_meshes: list[tuple[int, trimesh.Trimesh,
                                                  trimesh.Trimesh]],
              *, arrange: bool) -> None:
    """Write a standards-based multi-material 3MF assembly."""
    material_id = 1
    next_id = 2
    object_xml = []
    assemblies = []
    build_items = []

    if arrange:
        cols = 3
        # Twenty horizontal covers fit an A1 as a tight 3x7 grid.  Smaller
        # sets keep the more generous visual spacing used by the first tray.
        gap_x = 2.0 if len(tag_meshes) >= 20 else 5.0
        gap_y = 1.5 if len(tag_meshes) >= 20 else 5.0
        rows = math.ceil(len(tag_meshes) / cols)
        set_w = cols * PLATE_W + (cols - 1) * gap_x
        set_h = rows * PLATE_H + (rows - 1) * gap_y

    for index, (tag_id, white, black) in enumerate(tag_meshes):
        white_id = next_id
        black_id = next_id + 1
        assembly_id = next_id + 2
        next_id += 3
        object_xml.append(_mesh_xml(
            white_id, f"tag36h11_{tag_id:02d}_WHITE", white,
            material_id, 0,
        ))
        object_xml.append(_mesh_xml(
            black_id, f"tag36h11_{tag_id:02d}_BLACK", black,
            material_id, 1,
        ))
        assemblies.append(
            f'  <object id="{assembly_id}" name="tag36h11_{tag_id:02d}" type="model">\n'
            f"   <components>\n"
            f'    <component objectid="{white_id}"/>\n'
            f'    <component objectid="{black_id}"/>\n'
            f"   </components>\n  </object>"
        )
        if arrange:
            row, col = divmod(index, cols)
            tx = -set_w / 2.0 + PLATE_W / 2.0 + col * (PLATE_W + gap_x)
            ty = set_h / 2.0 - PLATE_H / 2.0 - row * (PLATE_H + gap_y)
        else:
            tx = ty = 0.0
        build_items.append(
            f'  <item objectid="{assembly_id}" '
            f'transform="1 0 0 0 1 0 0 0 1 {tx:.6f} {ty:.6f} 0"/>'
        )

    model = f'''<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xml:lang="en-US"
 xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">
 <metadata name="Title">STS3215 AprilTag motor-lid covers</metadata>
 <metadata name="Designer">hexapod parametric CAD</metadata>
 <metadata name="Description">tag36h11; white material first, black material second</metadata>
 <resources>
  <basematerials id="{material_id}">
   <base name="White" displaycolor="{WHITE}FF"/>
   <base name="Black" displaycolor="{BLACK}FF"/>
  </basematerials>
{chr(10).join(object_xml)}
{chr(10).join(assemblies)}
 </resources>
 <build>
{chr(10).join(build_items)}
 </build>
</model>
'''
    content_types = '''<?xml version="1.0" encoding="UTF-8"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
 <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
 <Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>
</Types>
'''
    relationships = '''<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
 <Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>
</Relationships>
'''
    path.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(path, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        archive.writestr("[Content_Types].xml", content_types)
        archive.writestr("_rels/.rels", relationships)
        archive.writestr("3D/3dmodel.model", model)


def _patch_bambu_project(path: Path, thumbnail: bytes | None = None) -> None:
    """Set Bambu filament assignments and optionally replace plate previews."""
    with zipfile.ZipFile(path, "r") as source:
        entries = {name: source.read(name) for name in source.namelist()}
    settings_key = "Metadata/project_settings.config"
    if settings_key in entries:
        settings = json.loads(entries[settings_key].decode("utf-8"))
        # The Bambu CLI duplicates leaf-profile values when loading the same
        # PLA preset twice, but inherited base-profile vectors can remain
        # one element long.  Expand those filament-owned vectors explicitly;
        # otherwise the project looks two-colour but slices as one filament.
        filament_keys: set[str] = set()
        profile = BAMBU_PLA_PROFILE
        seen_profiles: set[Path] = set()
        while profile.is_file() and profile not in seen_profiles:
            seen_profiles.add(profile)
            profile_data = json.loads(profile.read_text(encoding="utf-8"))
            filament_keys.update(profile_data)
            inherited = profile_data.get("inherits")
            if not inherited:
                break
            profile = BAMBU_PLA_PROFILE.parent / f"{inherited}.json"
        for profile_key in filament_keys:
            setting_key = BAMBU_FILAMENT_KEY_ALIASES.get(
                profile_key, profile_key
            )
            value = settings.get(setting_key)
            if isinstance(value, list) and len(value) == 1:
                settings[setting_key] = value * 2
        settings["filament_colour"] = ["#FFFFFF", "#000000"]
        settings["default_filament_colour"] = ["#FFFFFF", "#000000"]
        settings["extruder_colour"] = ["#FFFFFF", "#000000"]
        # Bambu's CLI may retain its four-spool default flush table even when
        # this project loads two filaments.  A 2x2 table and two-value vector
        # are required for the project to slice without manual repair.
        settings["flush_volumes_matrix"] = ["0", "280", "280", "0"]
        settings["flush_volumes_vector"] = ["140", "140", "140", "140"]
        # The 16-object A1 arrangement occupies the default upper-left tower
        # keepout.  The lower-left grid slot is intentionally empty, so place
        # the prime tower there instead.
        settings["wipe_tower_x"] = ["15"]
        settings["wipe_tower_y"] = ["15"]
        entries[settings_key] = (
            json.dumps(settings, indent=4) + "\n"
        ).encode("utf-8")

    # Bambu Studio preserves the two component volumes from the standard 3MF,
    # but does not carry its base-material assignment into slicer extruders.
    # Explicitly map source volume 0 -> white/extruder 1 and volume 1 ->
    # black/extruder 2 for every grouped lid.
    model_key = "Metadata/model_settings.config"
    if model_key in entries:
        root = ET.fromstring(entries[model_key])
        for part in root.findall(".//part"):
            source_volume = next(
                (
                    metadata.get("value")
                    for metadata in part.findall("metadata")
                    if metadata.get("key") == "source_volume_id"
                ),
                None,
            )
            if source_volume is None:
                continue
            desired = str(int(source_volume) + 1)
            extruder = next(
                (
                    metadata
                    for metadata in part.findall("metadata")
                    if metadata.get("key") == "extruder"
                ),
                None,
            )
            if extruder is None:
                extruder = ET.SubElement(part, "metadata", {"key": "extruder"})
            extruder.set("value", desired)
        entries[model_key] = (
            b'<?xml version="1.0" encoding="UTF-8"?>\n'
            + ET.tostring(root, encoding="utf-8")
        )

    if thumbnail is not None:
        for image_key in (
            "Metadata/plate_1.png",
            "Metadata/plate_no_light_1.png",
            "Metadata/top_1.png",
            "Metadata/pick_1.png",
        ):
            if image_key in entries:
                entries[image_key] = thumbnail

    temp = path.with_suffix(".tmp.3mf")
    with zipfile.ZipFile(temp, "w", compression=zipfile.ZIP_DEFLATED) as target:
        for name, data in entries.items():
            target.writestr(name, data)
    temp.replace(path)


def write_bambu_project(
    path: Path,
    portable_3mf: Path,
) -> bool:
    """Write a Bambu Studio project with white=extruder 1, black=extruder 2.

    Returns false when Bambu Studio or its bundled Generic PLA profile is not
    present.  The standards-based 3MF remains the portable fallback.
    """
    required_bambu_files = (
        BAMBU_STUDIO,
        BAMBU_PLA_PROFILE,
        BAMBU_MACHINE_PROFILE,
        BAMBU_PROCESS_PROFILE,
    )
    if not all(candidate.is_file() for candidate in required_bambu_files):
        return False
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="apriltag-lids-") as temp_name:
        temp_dir = Path(temp_name)
        staged_path = temp_dir / path.name
        resolved_machine = temp_dir / "bambu_a1_machine_resolved.json"
        resolved_process = temp_dir / "bambu_a1_process_resolved.json"
        resolved_filament = temp_dir / "generic_pla_a1_resolved.json"
        for source, target in (
            (BAMBU_MACHINE_PROFILE, resolved_machine),
            (BAMBU_PROCESS_PROFILE, resolved_process),
            (BAMBU_PLA_PROFILE, resolved_filament),
        ):
            target.write_text(
                json.dumps(resolved_bambu_profile(source), indent=2) + "\n",
                encoding="utf-8",
            )
        cmd = [
            str(BAMBU_STUDIO),
            "--outputdir", str(temp_dir),
            "--allow-multicolor-oneplate",
            "--arrange", "1",
            "--load-settings",
            f"{resolved_machine};{resolved_process}",
            "--load-filaments", f"{resolved_filament};{resolved_filament}",
            "--export-3mf", staged_path.name,
            str(portable_3mf),
        ]
        completed = subprocess.run(
            cmd, check=False, capture_output=True, text=True, timeout=180
        )
        if completed.returncode != 0 or not staged_path.is_file():
            message = (completed.stderr or completed.stdout).strip()
            raise RuntimeError(f"Bambu Studio 3MF export failed: {message}")
        _patch_bambu_project(staged_path)

        # Generate a trustworthy black/white plate preview from the patched
        # extruder metadata, then embed it in the project thumbnail slots.
        preview_dir = temp_dir / "preview"
        preview_dir.mkdir()
        preview_cmd = [
            str(BAMBU_STUDIO),
            "--outputdir", str(preview_dir),
            "--export-png", "1",
            "--camera-view", "1",
            str(staged_path),
        ]
        preview_run = subprocess.run(
            preview_cmd, check=False, capture_output=True, text=True, timeout=180
        )
        preview_files = sorted(preview_dir.glob("plate_*.png"))
        if preview_run.returncode != 0 or not preview_files:
            message = (preview_run.stderr or preview_run.stdout).strip()
            raise RuntimeError(f"Bambu Studio preview export failed: {message}")
        _patch_bambu_project(staged_path, preview_files[0].read_bytes())
        shutil.copy2(staged_path, path)
    return True


def write_svg(path: Path, tag_id: int) -> None:
    grid = tag_grid(tag_id)
    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{TAG_SIZE}mm" '
        f'height="{TAG_SIZE}mm" viewBox="0 0 {TAG_SIZE} {TAG_SIZE}">',
        f'<title>tag36h11 ID {tag_id}</title>',
        f'<rect width="{TAG_SIZE}" height="{TAG_SIZE}" fill="#fff"/>',
    ]
    for row in range(TOTAL_WIDTH):
        col = 0
        while col < TOTAL_WIDTH:
            if grid[row][col] == 1:
                col += 1
                continue
            start = col
            while col < TOTAL_WIDTH and grid[row][col] == 0:
                col += 1
            lines.append(
                f'<rect x="{start * CELL:.4f}" y="{row * CELL:.4f}" '
                f'width="{(col - start) * CELL:.4f}" height="{CELL:.4f}" fill="#000"/>'
            )
    lines.append("</svg>")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def render_preview(path: Path, ids: list[int]) -> None:
    cols = 4
    rows = math.ceil(len(ids) / cols)
    fig, axes = plt.subplots(rows, cols, figsize=(11, 4.9 * rows), squeeze=False)
    for ax in axes.flat:
        ax.set_aspect("equal")
        ax.axis("off")
    for ax, tag_id in zip(axes.flat, ids):
        # Show the installed portrait orientation.  Plot U=-print Y and
        # V=print X, so the production screw stations appear top and bottom.
        ax.add_patch(FancyBboxPatch(
            (-PLATE_H / 2.0, -PLATE_W / 2.0), PLATE_H, PLATE_W,
            boxstyle=f"round,pad=0,rounding_size={CORNER_R}",
            facecolor=WHITE, edgecolor="#666", linewidth=0.8,
        ))
        grid = tag_grid(tag_id)
        for row in range(TOTAL_WIDTH):
            for col in range(TOTAL_WIDTH):
                colour = BLACK if grid[row][col] == 0 else WHITE
                u0 = -TAG_SIZE / 2.0 + col * CELL
                v1 = TAG_SIZE / 2.0 - row * CELL
                ax.add_patch(Rectangle(
                    (u0, v1 - CELL), CELL, CELL,
                    facecolor=colour, edgecolor=colour, linewidth=0,
                ))
        label = id_label_portrait_2d(tag_id)
        label_polygons = [label] if isinstance(label, Polygon) else label.geoms
        for polygon in label_polygons:
            x, y = polygon.exterior.xy
            ax.fill(x, y, color=BLACK, linewidth=0)
        for x, y in BOLT_CENTRES:
            portrait_xy = (-y, x)
            ax.add_patch(Circle(portrait_xy, CSK_TOP_OD / 2.0,
                                facecolor="#c8c8c8", edgecolor="#555", linewidth=0.6))
            ax.add_patch(Circle(portrait_xy, THROUGH_OD / 2.0,
                                facecolor="white", edgecolor="#555", linewidth=0.5))
        ax.set_xlim(-PLATE_H / 2.0 - 2, PLATE_H / 2.0 + 2)
        ax.set_ylim(-PLATE_W / 2.0 - 4, PLATE_W / 2.0 + 2)
        ax.text(0, -PLATE_W / 2.0 - 2.2, f"tag36h11 ID {tag_id}",
                ha="center", va="center", fontsize=9)
    fig.suptitle(
        f"STS3215 AprilTag motor-lid covers — portrait view, {TAG_SIZE:g} mm tags",
        fontsize=13,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(path, dpi=180, facecolor="white")
    plt.close(fig)


def validate_meshes(tag_id: int, white: trimesh.Trimesh,
                    black: trimesh.Trimesh) -> dict:
    if not white.is_volume:
        raise AssertionError(f"tag {tag_id}: white mesh is not a closed volume")
    if not black.is_volume:
        raise AssertionError(f"tag {tag_id}: black mesh is not a closed volume")
    merged = combined_mesh(white, black)
    if not merged.is_volume:
        raise AssertionError(f"tag {tag_id}: combined mesh is not a closed volume")
    expected = np.array([PLATE_W, PLATE_H, TOTAL_T])
    if not np.allclose(merged.extents, expected, atol=0.03):
        raise AssertionError(
            f"tag {tag_id}: bbox {merged.extents.tolist()} != {expected.tolist()}"
        )
    # The 2D source is a strict partition: white_skin = skin - black_cells.
    # Comparing material-body volume against the independently built full
    # solid catches either a gap or overlap without numerically intersecting
    # hundreds of exactly coplanar pixel walls.
    colour_partition_error = abs(
        float(white.volume) + float(black.volume) - float(merged.volume)
    )
    if colour_partition_error > 1e-3:
        raise AssertionError(
            f"tag {tag_id}: colour partition differs from full solid by "
            f"{colour_partition_error:.6f} mm^3"
        )
    return {
        "white_triangles": int(len(white.faces)),
        "black_triangles": int(len(black.faces)),
        "combined_triangles": int(len(merged.faces)),
        "combined_volume_mm3": round(float(merged.volume), 3),
        "bbox_mm": [round(float(v), 3) for v in merged.extents],
        "colour_partition_error_mm3": round(colour_partition_error, 8),
    }


def parse_ids(text: str) -> list[int]:
    ids: list[int] = []
    for token in text.split(","):
        token = token.strip()
        if not token:
            continue
        if "-" in token:
            start, end = (int(value) for value in token.split("-", 1))
            ids.extend(range(start, end + 1))
        else:
            ids.append(int(token))
    ids = list(dict.fromkeys(ids))
    if not ids:
        raise ValueError("at least one tag ID is required")
    for tag_id in ids:
        tag_grid(tag_id)
    return ids


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ids", default="1-16",
                        help="comma/range list within 0..18 (default: 1-16)")
    parser.add_argument("--output", type=Path, default=HERE / "out")
    args = parser.parse_args()
    ids = parse_ids(args.ids)
    output = args.output.resolve()
    individual = output / "individual"
    individual.mkdir(parents=True, exist_ok=True)

    generated: list[tuple[int, trimesh.Trimesh, trimesh.Trimesh]] = []
    manifest_rows = []
    for tag_id in ids:
        print(f"building tag36h11 ID {tag_id} ...", flush=True)
        white, black = build_tag_meshes(tag_id)
        stats = validate_meshes(tag_id, white, black)
        merged = combined_mesh(white, black)
        stem = f"tag36h11_{tag_id:02d}_motor_lid"
        white_path = individual / f"{stem}_WHITE.stl"
        black_path = individual / f"{stem}_BLACK.stl"
        combined_path = individual / f"{stem}_combined_colorless.stl"
        three_mf_path = individual / f"{stem}.3mf"
        svg_path = individual / f"tag36h11_{tag_id:02d}_{TAG_SIZE:g}mm.svg"
        white.export(white_path)
        black.export(black_path)
        merged.export(combined_path)
        write_svg(svg_path, tag_id)
        write_3mf(three_mf_path, [(tag_id, white, black)], arrange=False)
        generated.append((tag_id, white, black))
        manifest_rows.append({
            "tag_id": tag_id,
            "white_stl": str(white_path.relative_to(output)),
            "black_stl": str(black_path.relative_to(output)),
            "combined_colorless_stl": str(combined_path.relative_to(output)),
            "three_mf": str(three_mf_path.relative_to(output)),
            "svg": str(svg_path.relative_to(output)),
            **stats,
        })

    first, last = min(ids), max(ids)
    set_path = output / f"tag36h11_motor_lids_{first:02d}-{last:02d}.3mf"
    write_3mf(set_path, generated, arrange=True)
    bambu_path = output / (
        f"tag36h11_motor_lids_{first:02d}-{last:02d}_BambuStudio.3mf"
    )
    bambu_written = write_bambu_project(bambu_path, set_path)
    preview_path = output / f"preview_tag36h11_{first:02d}-{last:02d}.png"
    render_preview(preview_path, ids)

    # Reference full-solid accessory used by the ROM clearance audit.  Tag ID
    # does not change the external envelope, so one representative is enough.
    reference = combined_mesh(generated[0][1], generated[0][2])
    reference_path = output / "motor_lid_accessory_reference.stl"
    reference.export(reference_path)

    manifest = {
        "schema_version": 1,
        "family": "tag36h11",
        "ids": ids,
        "dimensions_mm": {
            "plate": [PLATE_W, PLATE_H, TOTAL_T],
            "tag_outer_with_quiet_zone": TAG_SIZE,
            "black_square": TAG_SIZE * WIDTH_AT_BORDER / TOTAL_WIDTH,
            "cell": CELL,
            "portrait_centre_v": 0.0,
            "white_base": BASE_T,
            "coplanar_colour_skin": SKIN_T,
            "id_label_digit_height": LABEL_DIGIT_H,
            "id_label_portrait_centre_v": LABEL_CENTER_V,
        },
        "fasteners": {
            "centres_mm": [list(pair) for pair in BOLT_CENTRES],
            "through_diameter_mm": THROUGH_OD,
            "countersink_top_diameter_mm": CSK_TOP_OD,
            "countersink_angle_deg": 90,
            "replacement": "2 x M3x10 90-degree flat-head/countersunk screws per lid",
        },
        "set_3mf": str(set_path.relative_to(output)),
        "bambu_studio_3mf": (
            str(bambu_path.relative_to(output)) if bambu_written else None
        ),
        "preview": str(preview_path.relative_to(output)),
        "clearance_reference": str(reference_path.relative_to(output)),
        "parts": manifest_rows,
    }
    manifest_text = json.dumps(manifest, indent=2) + "\n"
    manifest_path = output / f"manifest_{first:02d}-{last:02d}.json"
    manifest_path.write_text(manifest_text, encoding="utf-8")
    # Preserve the historical default manifest name for the primary 1..16
    # robot set while allowing additional plates to coexist in the same out/.
    if ids == list(range(1, 17)):
        (output / "manifest.json").write_text(manifest_text, encoding="utf-8")
    print(f"wrote {set_path}")
    if bambu_written:
        print(f"wrote {bambu_path}")
    print(f"wrote {manifest_path}")
    print(f"wrote {preview_path}")


if __name__ == "__main__":
    main()
