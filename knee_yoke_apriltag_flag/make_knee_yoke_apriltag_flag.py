#!/usr/bin/env python3
"""Generate a removable AprilTag flag for the knee-yoke driven face.

The holder presses over the four exposed M3 socket-head cap screws on the
driven/front face of the production tibia knee yoke.  It does not enter the
clamped yoke/horn stack and requires no longer fasteners.  The camera face is
a 38 mm rounded square carrying a full 34 mm tag36h11 marker.

Print orientation is intentional: the tag face is at Z=0 and lies on the
build plate; the four split grip cups grow upward without supports.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import zipfile
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
from shapely.geometry import MultiPolygon, Polygon, box
from shapely.ops import unary_union
import trimesh


HERE = Path(__file__).resolve().parent
PROTO_DIR = HERE.parent
TAG_TOOL_DIR = PROTO_DIR / "apriltag_lids"
for import_dir in (PROTO_DIR, TAG_TOOL_DIR):
    if str(import_dir) not in sys.path:
        sys.path.insert(0, str(import_dir))

import hexapod_prototype as hp  # noqa: E402
import make_apriltag_lids as tags  # noqa: E402


DEFAULT_TAG_ID = 16
PLATE_SIZE = 38.0
PLATE_CORNER_R = 2.5
TAG_SIZE = 34.0
TAG_SKIN_T = 0.6
PLATE_T = 1.6

# ISO 4762 M3 socket-head nominal geometry.  The generated design remains
# parameterized because printed-hole shrink and off-brand heads vary.
DEFAULT_HEAD_D = 5.5
DEFAULT_HEAD_H = 3.0
HEAD_TOP_CLEARANCE = 0.25
CUP_OUTER_D = 8.2
CUP_PLATE_OVERLAP = 0.15
CUP_SLOT_W = 0.9
CUP_SLOT_START = 1.0
DEFAULT_THROAT_CLEARANCE = 0.0
INNER_BACK_CLEARANCE = 0.25

BACK_LABEL_PIXEL = 0.40
BACK_LABEL_GAP = 0.08
BACK_LABEL_DIGIT_GAP = 0.28
BACK_LABEL_EDGE_INSET = 1.0
BACK_LABEL_H = 0.50

WHITE = "#F8F8F8"
BLACK = "#101010"


def rounded_square(size: float, radius: float) -> Polygon:
    raw = box(-size / 2.0, -size / 2.0, size / 2.0, size / 2.0)
    return raw.buffer(-radius, join_style="round").buffer(
        radius, join_style="round", quad_segs=16
    )


def extrude_2d(geometry, height: float, z0: float = 0.0) -> trimesh.Trimesh:
    if isinstance(geometry, Polygon):
        polygons = [geometry]
    elif isinstance(geometry, MultiPolygon):
        polygons = list(geometry.geoms)
    else:
        polygons = [part for part in geometry.geoms if isinstance(part, Polygon)]
    meshes: list[trimesh.Trimesh] = []
    for polygon in polygons:
        if polygon.area <= 1e-9:
            continue
        mesh = trimesh.creation.extrude_polygon(polygon, height=height)
        if z0:
            mesh.apply_translation((0.0, 0.0, z0))
        meshes.append(mesh)
    if not meshes:
        raise ValueError("cannot extrude empty geometry")
    return trimesh.util.concatenate(meshes)


def frustum(
    radius_bottom: float,
    radius_top: float,
    z_bottom: float,
    z_top: float,
    *,
    sections: int = 72,
) -> trimesh.Trimesh:
    angles = np.linspace(0.0, 2.0 * math.pi, sections, endpoint=False)
    bottom = np.column_stack(
        (
            radius_bottom * np.cos(angles),
            radius_bottom * np.sin(angles),
            np.full(sections, z_bottom),
        )
    )
    top = np.column_stack(
        (
            radius_top * np.cos(angles),
            radius_top * np.sin(angles),
            np.full(sections, z_top),
        )
    )
    vertices = np.vstack((bottom, top, [[0.0, 0.0, z_bottom], [0.0, 0.0, z_top]]))
    bottom_center = 2 * sections
    top_center = bottom_center + 1
    faces: list[list[int]] = []
    for index in range(sections):
        nxt = (index + 1) % sections
        faces.extend(
            (
                [index, nxt, sections + nxt],
                [index, sections + nxt, sections + index],
                [bottom_center, nxt, index],
                [top_center, sections + index, sections + nxt],
            )
        )
    return trimesh.Trimesh(vertices=vertices, faces=np.asarray(faces), process=True)


def bolt_centres_about_output() -> tuple[tuple[float, float], ...]:
    """Read the production horn pattern, recentered on the output axis."""
    return tuple(
        (float(x) - float(hp.SERVO_OUTPUT_X), float(y))
        for x, y in hp._disc_horn_bolt_centres()
    )


def tag_grid(tag_id: int) -> np.ndarray:
    """Return a verified 10x10 tag36h11 grid; 0=black and 1=white.

    OpenCV is the same detector family used by the robot.  The 8x8 marker
    includes its one-cell black border; this function adds a one-cell white
    quiet zone to match the existing 34 mm motor-lid tags.  A 180-degree
    rotation keeps the artwork convention used by those existing lids.
    """
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    marker_count = int(dictionary.bytesList.shape[0])
    if not 0 <= tag_id < marker_count:
        raise ValueError(f"tag ID {tag_id} is outside tag36h11 0..{marker_count - 1}")
    rendered = cv2.aruco.generateImageMarker(
        dictionary, tag_id, 80, borderBits=1
    )
    marker = cv2.resize(rendered, (8, 8), interpolation=cv2.INTER_AREA)
    marker = np.rot90((marker > 127).astype(np.uint8), 2)
    grid = np.ones((10, 10), dtype=np.uint8)
    grid[1:9, 1:9] = marker
    return grid


def back_label_2d(tag_id: int):
    """Human-readable ID at the lower-right of the non-camera face."""
    text = str(tag_id)
    pitch = BACK_LABEL_PIXEL + BACK_LABEL_GAP
    digit_w = 3 * BACK_LABEL_PIXEL + 2 * BACK_LABEL_GAP
    digit_h = 5 * BACK_LABEL_PIXEL + 4 * BACK_LABEL_GAP
    total_w = len(text) * digit_w + (len(text) - 1) * BACK_LABEL_DIGIT_GAP
    x0 = PLATE_SIZE / 2.0 - BACK_LABEL_EDGE_INSET - total_w
    y0 = -PLATE_SIZE / 2.0 + BACK_LABEL_EDGE_INSET
    pixels = []
    for digit_index, digit in enumerate(text):
        dx = x0 + digit_index * (digit_w + BACK_LABEL_DIGIT_GAP)
        for row, pattern in enumerate(tags.PIXEL_DIGITS[digit]):
            for col, value in enumerate(pattern):
                if value == "0":
                    continue
                px0 = dx + col * pitch
                py1 = y0 + digit_h - row * pitch
                pixels.append(
                    box(px0, py1 - BACK_LABEL_PIXEL,
                        px0 + BACK_LABEL_PIXEL, py1)
                )
    return unary_union(pixels)


def black_back_label(tag_id: int) -> trimesh.Trimesh:
    """Raised rear ID volume assigned to the black material component.

    The 0.05 mm embed into the white plate matches the original fused label
    geometry, giving the two material parts a positive mechanical key rather
    than relying only on a coplanar contact surface.
    """
    return extrude_2d(
        back_label_2d(tag_id),
        BACK_LABEL_H + 0.05,
        z0=PLATE_T - 0.05,
    )


def boolean_union(parts: list[trimesh.Trimesh]) -> trimesh.Trimesh:
    result = trimesh.boolean.union(parts, engine="manifold", check_volume=False)
    if result is None:
        raise RuntimeError("manifold union returned no mesh")
    result.remove_unreferenced_vertices()
    return result


def boolean_difference(
    body: trimesh.Trimesh, cutters: list[trimesh.Trimesh]
) -> trimesh.Trimesh:
    result = trimesh.boolean.difference(
        [body, *cutters], engine="manifold", check_volume=False
    )
    if result is None:
        raise RuntimeError("manifold difference returned no mesh")
    result.remove_unreferenced_vertices()
    return result


def build_holder(
    *,
    tag_id: int = DEFAULT_TAG_ID,
    head_diameter: float,
    head_height: float,
    throat_clearance: float,
) -> tuple[trimesh.Trimesh, dict[str, float]]:
    """Build the one-piece blank holder before the tag colour inlay."""
    cup_height = head_height + HEAD_TOP_CLEARANCE
    total_height = PLATE_T + cup_height
    outer_radius = CUP_OUTER_D / 2.0
    throat_radius = (head_diameter + throat_clearance) / 2.0
    back_radius = (head_diameter + INNER_BACK_CLEARANCE) / 2.0

    plate = extrude_2d(rounded_square(PLATE_SIZE, PLATE_CORNER_R), PLATE_T)
    cup_parts: list[trimesh.Trimesh] = []
    cavities: list[trimesh.Trimesh] = []
    slots: list[trimesh.Trimesh] = []
    for x, y in bolt_centres_about_output():
        cup = trimesh.creation.cylinder(
            radius=outer_radius,
            height=cup_height + CUP_PLATE_OVERLAP,
            sections=96,
        )
        cup.apply_translation(
            (x, y, PLATE_T - CUP_PLATE_OVERLAP + (cup_height + CUP_PLATE_OVERLAP) / 2.0)
        )
        cup_parts.append(cup)

        # Wider at the closed back, gently narrowing at the mouth.  The slot
        # lets the two wall halves spring over a nominal head instead of
        # requiring a brittle dimensional interference fit.
        cavity = frustum(
            back_radius,
            throat_radius,
            PLATE_T - 0.03,
            total_height + 0.20,
        )
        cavity.apply_translation((x, y, 0.0))
        cavities.append(cavity)

        slot_height = cup_height - CUP_SLOT_START + 0.25
        slot = trimesh.creation.box(
            extents=(CUP_OUTER_D + 1.0, CUP_SLOT_W, slot_height)
        )
        slot.apply_translation(
            (x, y, PLATE_T + CUP_SLOT_START + slot_height / 2.0)
        )
        slots.append(slot)

    label = black_back_label(tag_id)
    solid = boolean_union([plate, *cup_parts, label])
    holder = boolean_difference(solid, [*cavities, *slots])
    dimensions = {
        "plate_size_mm": PLATE_SIZE,
        "tag_size_mm": TAG_SIZE,
        "plate_thickness_mm": PLATE_T,
        "cup_height_mm": cup_height,
        "total_height_mm": total_height,
        "head_diameter_mm": head_diameter,
        "head_height_mm": head_height,
        "cup_throat_diameter_mm": 2.0 * throat_radius,
        "cup_back_diameter_mm": 2.0 * back_radius,
        "bolt_circle_diameter_mm": float(hp.DISC_HORN_BOLT_PCD),
        "back_label_height_mm": BACK_LABEL_H,
    }
    return holder, dimensions


def black_tag_cells(tag_id: int) -> trimesh.Trimesh:
    grid = tag_grid(tag_id)
    cell = TAG_SIZE / tags.TOTAL_WIDTH
    polygons = []
    for row in range(tags.TOTAL_WIDTH):
        for col in range(tags.TOTAL_WIDTH):
            if grid[row][col] != 0:
                continue
            # The colour inlay is modeled on the build-plate side (Z=0), so
            # the camera observes it from -Z. Mirror X in the model to avoid
            # presenting a reflected, undecodable AprilTag after printing.
            face_col = tags.TOTAL_WIDTH - 1 - col
            x0 = -TAG_SIZE / 2.0 + face_col * cell
            y1 = TAG_SIZE / 2.0 - row * cell
            polygons.append(box(x0, y1 - cell, x0 + cell, y1))
    return extrude_2d(unary_union(polygons), TAG_SKIN_T + 0.01, z0=-0.005)


def write_tag_svg(path: Path, tag_id: int) -> None:
    """Write a camera-facing, correctly oriented 34 mm paper-tag fallback."""
    grid = tag_grid(tag_id)
    cell = TAG_SIZE / 10.0
    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{TAG_SIZE}mm" '
        f'height="{TAG_SIZE}mm" viewBox="0 0 {TAG_SIZE} {TAG_SIZE}">',
        f'  <title>tag36h11 ID {tag_id}</title>',
        f'  <rect width="{TAG_SIZE}" height="{TAG_SIZE}" fill="white"/>',
    ]
    for row in range(10):
        for col in range(10):
            if grid[row, col] != 0:
                continue
            lines.append(
                f'  <rect x="{col * cell:.6f}" y="{row * cell:.6f}" '
                f'width="{cell:.6f}" height="{cell:.6f}" fill="black"/>'
            )
    lines.append("</svg>")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_multimaterial_3mf(
    path: Path,
    tag_meshes: list[tuple[int, trimesh.Trimesh, trimesh.Trimesh]],
    *,
    columns: int | None = None,
    gap_mm: float = 4.0,
) -> None:
    """Write independent white/black holder objects arranged as a tray."""
    if columns is None:
        # Six 38 mm flags with five 4 mm gaps occupy 248 mm, fitting the
        # 256 x 256 mm A1 plate. Smaller sets stay in the roomier 5-column
        # layout used by the original 16-32 tray.
        columns = 6 if len(tag_meshes) > 25 else 5
    material_id = 1
    next_id = 2
    objects: list[str] = []
    assemblies: list[str] = []
    build_items: list[str] = []
    rows = math.ceil(len(tag_meshes) / columns)
    set_w = columns * PLATE_SIZE + (columns - 1) * gap_mm
    set_h = rows * PLATE_SIZE + (rows - 1) * gap_mm
    for index, (tag_id, white, black) in enumerate(tag_meshes):
        white_id, black_id, assembly_id = next_id, next_id + 1, next_id + 2
        next_id += 3
        objects.append(
            tags._mesh_xml(
                white_id, f"tag36h11_{tag_id:02d}_WHITE", white,
                material_id, 0,
            )
        )
        objects.append(
            tags._mesh_xml(
                black_id, f"tag36h11_{tag_id:02d}_BLACK", black,
                material_id, 1,
            )
        )
        assemblies.append(
            f'  <object id="{assembly_id}" name="tag36h11_{tag_id:02d}" type="model">\n'
            f"   <components>\n"
            f'    <component objectid="{white_id}"/>\n'
            f'    <component objectid="{black_id}"/>\n'
            f"   </components>\n  </object>"
        )
        row, col = divmod(index, columns)
        tx = -set_w / 2.0 + PLATE_SIZE / 2.0 + col * (PLATE_SIZE + gap_mm)
        ty = set_h / 2.0 - PLATE_SIZE / 2.0 - row * (PLATE_SIZE + gap_mm)
        build_items.append(
            f'  <item objectid="{assembly_id}" '
            f'transform="1 0 0 0 1 0 0 0 1 {tx:.6f} {ty:.6f} 0"/>'
        )

    model = f'''<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xml:lang="en-US"
 xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">
 <metadata name="Title">STS3215 knee-yoke AprilTag flags {tag_meshes[0][0]}-{tag_meshes[-1][0]}</metadata>
 <metadata name="Designer">hexapod parametric CAD</metadata>
 <metadata name="Description">tag36h11; black raised ID on back; tighter M3 head cups</metadata>
 <resources>
  <basematerials id="{material_id}">
   <base name="White" displaycolor="{WHITE}FF"/>
   <base name="Black" displaycolor="{BLACK}FF"/>
  </basematerials>
{chr(10).join(objects)}
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


def build_colour_parts(
    holder: trimesh.Trimesh, tag_id: int
) -> tuple[trimesh.Trimesh, trimesh.Trimesh]:
    tag_cells = black_tag_cells(tag_id)
    rear_label = black_back_label(tag_id)
    white = boolean_difference(holder, [tag_cells, rear_label])
    # Return an exact-height black insert after using a slightly overshooting
    # camera-face copy as the cutter. The disconnected rear label is kept in
    # the same black object so Bambu Studio assigns both features to slot 2.
    exact_tag_cells = black_tag_cells(tag_id)
    exact_tag_cells.apply_translation((0.0, 0.0, 0.005))
    black = trimesh.util.concatenate([exact_tag_cells, rear_label])
    return white, black


def installed_mesh(print_mesh: trimesh.Trimesh, total_height: float) -> trimesh.Trimesh:
    """Map print coordinates onto the yoke's driven-face joint coordinates."""
    result = print_mesh.copy()
    result.apply_transform(
        np.array(
            [
                [1.0, 0.0, 0.0, hp.SERVO_OUTPUT_X],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, -1.0, hp._YOKE_TOP_Z1 + total_height],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
    )
    return result


def write_preview(
    path: Path,
    white: trimesh.Trimesh,
    black: trimesh.Trimesh,
    holder: trimesh.Trimesh,
    tag_id: int,
) -> None:
    figure = plt.figure(figsize=(15, 5.5), dpi=180)
    total_height = float(holder.bounds[1][2])
    installed_white = installed_mesh(white, total_height)
    installed_black = installed_mesh(black, total_height)
    yoke = hp.make_tibia_knee_yoke()
    views = (
        (
            figure.add_subplot(1, 3, 1, projection="3d"),
            "camera face",
            -90,
            -90,
            ((white, WHITE, 1.0), (black, BLACK, 1.0)),
            ((-21, 21), (-21, 21), (-0.5, 6.0)),
            (42, 42, 8),
        ),
        (
            figure.add_subplot(1, 3, 2, projection="3d"),
            "rear: black raised ID and four split cups",
            62,
            -55,
            ((white, WHITE, 1.0), (black, BLACK, 1.0)),
            ((-21, 21), (-21, 21), (0.0, 6.0)),
            (42, 42, 10),
        ),
        (
            figure.add_subplot(1, 3, 3, projection="3d"),
            "installed on driven yoke face",
            34,
            -62,
            (
                (yoke, "#4385C6", 0.85),
                (installed_white, WHITE, 1.0),
                (installed_black, BLACK, 1.0),
            ),
            ((-8, 64), (-27, 27), (-14, 55)),
            (72, 54, 69),
        ),
    )
    for axis, title, elevation, azimuth, meshes, limits, aspect in views:
        edge_colour = "#8f8f8f" if title.startswith("rear:") else "none"
        edge_width = 0.08 if title.startswith("rear:") else 0.0
        for mesh, colour, alpha in meshes:
            triangles = mesh.vertices[mesh.faces]
            axis.add_collection3d(
                Poly3DCollection(
                    triangles,
                    facecolor=colour,
                    edgecolor=edge_colour,
                    linewidth=edge_width,
                    alpha=alpha,
                )
            )
        axis.set_xlim(*limits[0])
        axis.set_ylim(*limits[1])
        axis.set_zlim(*limits[2])
        axis.set_box_aspect(aspect)
        axis.view_init(elev=elevation, azim=azimuth)
        axis.set_title(title)
        axis.set_axis_off()
    figure.suptitle(f"Knee-yoke AprilTag {tag_id} push-on flag — 38 mm plate")
    figure.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, transparent=False, facecolor="white")
    plt.close(figure)


def parse_tray_range(value: str) -> list[int]:
    """Parse an inclusive ID range such as 16-32."""
    try:
        start_text, end_text = value.split("-", maxsplit=1)
        start, end = int(start_text), int(end_text)
    except ValueError as error:
        raise argparse.ArgumentTypeError(
            "tray must be an inclusive range like 16-32"
        ) from error
    if start > end:
        raise argparse.ArgumentTypeError(
            "tray range start must not exceed its end"
        )
    tag_grid(start)
    tag_grid(end)
    return list(range(start, end + 1))


def decoded_tag_id(tag_id: int) -> int | None:
    """Render an ideal camera view and return the OpenCV-decoded ID."""
    canvas = np.ones((12, 12), dtype=np.uint8) * 255
    canvas[1:11, 1:11] = tag_grid(tag_id) * 255
    image = cv2.resize(canvas, (720, 720), interpolation=cv2.INTER_NEAREST)
    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),
        cv2.aruco.DetectorParameters(),
    )
    _corners, ids, _rejected = detector.detectMarkers(image)
    return None if ids is None else int(ids.flatten()[0])


def write_tray_preview(path: Path, tag_ids: list[int]) -> None:
    """Render the unobstructed camera faces as a compact verification sheet."""
    columns = 5
    rows = math.ceil(len(tag_ids) / columns)
    figure, axes = plt.subplots(
        rows, columns, figsize=(2.2 * columns, 2.4 * rows), squeeze=False
    )
    for axis in axes.flat:
        axis.axis("off")
    for axis, tag_id in zip(axes.flat, tag_ids):
        axis.imshow(
            tag_grid(tag_id), cmap="gray", vmin=0, vmax=1,
            interpolation="nearest"
        )
        axis.set_title(f"ID {tag_id}", fontsize=9)
    figure.suptitle(
        "Knee-yoke flags: clean camera faces (human-readable IDs are on backs)",
        fontsize=12,
    )
    figure.tight_layout(rect=(0, 0, 1, 0.96))
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=180, facecolor="white")
    plt.close(figure)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tag-id", type=int, default=DEFAULT_TAG_ID)
    parser.add_argument(
        "--tray",
        type=parse_tray_range,
        metavar="START-END",
        help="generate an inclusive multi-part print tray, for example 16-32",
    )
    parser.add_argument("--head-diameter", type=float, default=DEFAULT_HEAD_D)
    parser.add_argument("--head-height", type=float, default=DEFAULT_HEAD_H)
    parser.add_argument(
        "--throat-clearance",
        type=float,
        default=DEFAULT_THROAT_CLEARANCE,
        help="diametral clearance at cup mouth; reduce if the fit is loose",
    )
    parser.add_argument("--out", type=Path, default=HERE / "out")
    parser.add_argument("--skip-bambu", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    if args.tray:
        tag_meshes: list[tuple[int, trimesh.Trimesh, trimesh.Trimesh]] = []
        holder_reports: list[dict] = []
        decoded: list[int | None] = []
        paper_dir = out / "paper_tags"
        for tag_id in args.tray:
            holder, dimensions = build_holder(
                tag_id=tag_id,
                head_diameter=args.head_diameter,
                head_height=args.head_height,
                throat_clearance=args.throat_clearance,
            )
            white, black = build_colour_parts(holder, tag_id)
            tag_meshes.append((tag_id, white, black))
            components = holder.split(only_watertight=False)
            holder_reports.append(
                {
                    "tag_id": tag_id,
                    "watertight": bool(holder.is_watertight),
                    "connected_components": len(components),
                    "volume_mm3": round(abs(float(holder.volume)), 3),
                }
            )
            decoded.append(decoded_tag_id(tag_id))
            write_tag_svg(
                paper_dir / f"tag36h11_{tag_id:02d}_{TAG_SIZE:g}mm.svg",
                tag_id,
            )

        first_id, last_id = args.tray[0], args.tray[-1]
        stem = f"tag36h11_{first_id:02d}-{last_id:02d}_knee_yoke_snap_flags"
        portable_3mf = out / f"{stem}.3mf"
        write_multimaterial_3mf(portable_3mf, tag_meshes)
        preview = out / f"{stem}_preview.png"
        write_tray_preview(preview, args.tray)
        bambu_created = False
        bambu_path = out / f"{stem}_BambuStudio.3mf"
        bambu_plate_preview: Path | None = None
        if not args.skip_bambu:
            bambu_created = tags.write_bambu_project(
                bambu_path, portable_3mf
            )
            if bambu_created:
                bambu_plate_preview = out / f"{stem}_rear_plate_preview.png"
                with zipfile.ZipFile(bambu_path) as project:
                    bambu_plate_preview.write_bytes(
                        project.read("Metadata/plate_1.png")
                    )

        passed = bool(
            all(
                report["watertight"] and report["connected_components"] == 1
                for report in holder_reports
            )
            and decoded == args.tray
        )
        manifest = {
            "schema_version": 1,
            "name": "STS3215 knee-yoke AprilTag snap-flag tray",
            "tag_family": "tag36h11",
            "tag_ids": args.tray,
            "part_count": len(args.tray),
            "attachment": "four split friction cups over M3 SHCS heads",
            "camera_face": (
                "tag and full white quiet zone only; black human-readable ID is on back"
            ),
            "print_orientation": (
                "tag faces on build plate; cups and black raised IDs upward; no supports"
            ),
            "colour_assignment": {
                "white": "holder plate and four grip cups",
                "black": "AprilTag cells and raised rear IDs",
            },
            "dimensions": dimensions,
            "opencv_decoded_tag_ids": decoded,
            "holders": holder_reports,
            "outputs": {
                "portable_3mf": portable_3mf.name,
                "bambu_project": (
                    f"{stem}_BambuStudio.3mf" if bambu_created else None
                ),
                "bambu_rear_plate_preview": (
                    bambu_plate_preview.name if bambu_plate_preview else None
                ),
                "paper_tag_directory": paper_dir.name,
                "preview": preview.name,
            },
            "pass": passed,
        }
        (out / f"{stem}_manifest.json").write_text(
            json.dumps(manifest, indent=2) + "\n"
        )
        print(json.dumps(manifest, indent=2))
        if not passed:
            raise SystemExit("generated tray failed mesh or AprilTag verification")
        return

    holder, dimensions = build_holder(
        tag_id=args.tag_id,
        head_diameter=args.head_diameter,
        head_height=args.head_height,
        throat_clearance=args.throat_clearance,
    )
    white, black = build_colour_parts(holder, args.tag_id)

    stem = f"tag36h11_{args.tag_id:02d}_knee_yoke_snap_flag"
    holder.export(out / f"{stem}_plain.stl")
    white.export(out / f"{stem}_WHITE.stl")
    black.export(out / f"{stem}_BLACK.stl")
    portable_3mf = out / f"{stem}.3mf"
    tags.write_3mf(portable_3mf, [(args.tag_id, white, black)], arrange=False)
    write_tag_svg(
        out / f"tag36h11_{args.tag_id:02d}_{TAG_SIZE:g}mm.svg", args.tag_id
    )
    write_preview(out / "preview.png", white, black, holder, args.tag_id)

    bambu_created = False
    if not args.skip_bambu:
        bambu_created = tags.write_bambu_project(
            out / f"{stem}_BambuStudio.3mf", portable_3mf
        )

    components = holder.split(only_watertight=False)
    manifest = {
        "schema_version": 1,
        "name": "STS3215 knee-yoke AprilTag snap flag",
        "tag_family": "tag36h11",
        "tag_id": args.tag_id,
        "attachment": "four split friction cups over M3 SHCS heads",
        "camera_face": (
            "tag and full white quiet zone only; black human-readable ID is on back"
        ),
        "print_orientation": (
            "tag face on build plate; cups and black raised ID upward; no supports"
        ),
        "colour_assignment": {
            "white": "holder plate and four grip cups",
            "black": "AprilTag cells and raised rear ID",
        },
        "dimensions": dimensions,
        "bolt_centres_mm": bolt_centres_about_output(),
        "holder": {
            "watertight": bool(holder.is_watertight),
            "connected_components": len(components),
            "volume_mm3": round(abs(float(holder.volume)), 3),
            "mass_g_at_1_24_g_cm3": round(abs(float(holder.volume)) * 0.00124, 3),
        },
        "outputs": {
            "portable_3mf": portable_3mf.name,
            "bambu_project": f"{stem}_BambuStudio.3mf" if bambu_created else None,
            "plain_stl": f"{stem}_plain.stl",
            "aligned_colour_stls": [f"{stem}_WHITE.stl", f"{stem}_BLACK.stl"],
            "paper_tag_svg": f"tag36h11_{args.tag_id:02d}_{TAG_SIZE:g}mm.svg",
            "preview": "preview.png",
        },
    }
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")

    print(json.dumps(manifest, indent=2))
    if not holder.is_watertight or len(components) != 1:
        raise SystemExit("generated holder must be one watertight connected body")


if __name__ == "__main__":
    main()
