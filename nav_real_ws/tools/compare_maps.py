#!/usr/bin/env python3
"""Align two ROS occupancy maps (PGM + YAML) and estimate map-frame transform.

Typical use in this repo:
    python3 tools/compare_maps.py \\
        --official-yaml ../../../map/compitation.yaml \\
        --ours-yaml ../src/car_sim/map/map_sim.yaml

Assumes both maps cover the same physical layout with the same orientation
(different SLAM / scan origins only). Estimates 2D translation (and optionally
a small rotation) from occupied-cell overlap in a common metric grid.

Output:
  - Transform: point_ours = R @ point_official + t  (map frame, metres)
  - Inverse for converting official coordinates into the team map frame
  - Match score and suggested use (map alignment only, not GOAL_LIST tuning)

Dependencies: Python 3 stdlib only (overlay written as PPM; PNG if Pillow installed).
"""

from __future__ import annotations

import argparse
import math
import re
import sys
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

# Overlay colors (RGB)
COLOR_BG = (245, 245, 245)
COLOR_TEAM = (30, 144, 255)       # team map walls (blue)
COLOR_OFFICIAL = (220, 60, 60)    # official walls after transform (red)
COLOR_OVERLAP = (255, 200, 0)     # both (gold)
COLOR_UNKNOWN = (200, 200, 200)

def load_map_yaml(path: Path) -> Dict:
    text = path.read_text(encoding="utf-8", errors="replace")
    data: Dict = {}
    for line in text.splitlines():
        line = line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, val = line.split(":", 1)
        key = key.strip()
        val = val.strip()
        if key == "origin":
            nums = [float(x) for x in re.findall(r"-?\d+\.?\d*", val)]
            data["origin"] = (nums[0], nums[1], nums[2] if len(nums) > 2 else 0.0)
        elif key in ("resolution", "occupied_thresh", "free_thresh", "negate"):
            data[key] = float(val)
        else:
            data[key] = val.strip('"').strip("'")
    if "resolution" not in data:
        raise ValueError(f"Missing resolution in {path}")
    if "origin" not in data:
        raise ValueError(f"Missing origin in {path}")
    return data


def resolve_image_path(yaml_path: Path, image_name: str) -> Path:
    # Fix common typo compitation..pgm -> compitation.pgm
    name = image_name.replace("..pgm", ".pgm")
    candidate = yaml_path.parent / name
    if candidate.is_file():
        return candidate
    alt = yaml_path.parent / name.replace("..", ".")
    if alt.is_file():
        return alt
    raise FileNotFoundError(f"PGM not found for '{image_name}' next to {yaml_path}")


def read_pgm(path: Path) -> Tuple[int, int, List[int]]:
    with path.open("rb") as f:
        magic = f.readline().strip()
        if magic != b"P5":
            raise ValueError(f"{path}: expected P5, got {magic!r}")

        tokens: List[bytes] = []
        while len(tokens) < 3:
            line = f.readline()
            if not line:
                raise ValueError(f"{path}: unexpected EOF in header")
            if line.startswith(b"#"):
                continue
            tokens.extend(part for part in line.split() if part)

        width, height, maxval = int(tokens[0]), int(tokens[1]), int(tokens[2])
        if maxval > 255:
            raise ValueError(f"{path}: 16-bit PGM not supported")
        raw = f.read(width * height)
        if len(raw) != width * height:
            raise ValueError(f"{path}: expected {width * height} bytes, got {len(raw)}")
    return width, height, list(raw)


def occupancy_probability(pixel: int, negate: int) -> float:
    """Same trinary convention as ROS map_server (negate flag from YAML)."""
    if negate:
        return pixel / 255.0
    return (255.0 - pixel) / 255.0


def occupied_cells(
    width: int,
    height: int,
    pixels: List[int],
    origin: Tuple[float, float, float],
    resolution: float,
    negate: int = 0,
    occupied_thresh: float = 0.65,
) -> List[Tuple[float, float]]:
    ox, oy, _ = origin
    cells: List[Tuple[float, float]] = []
    for row in range(height):
        for col in range(width):
            v = pixels[row * width + col]
            if occupancy_probability(v, negate) > occupied_thresh:
                x = ox + (col + 0.5) * resolution
                y = oy + (row + 0.5) * resolution
                cells.append((x, y))
    return cells


def world_bounds(points: List[Tuple[float, float]]) -> Tuple[float, float, float, float]:
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return min(xs), max(xs), min(ys), max(ys)


def quantize(points: List[Tuple[float, float]], step: float) -> Set[Tuple[int, int]]:
    inv = 1.0 / step
    return { (int(round(x * inv)), int(round(y * inv))) for x, y in points }


def score_translation(
    occ_official: Set[Tuple[int, int]],
    occ_ours: Set[Tuple[int, int]],
    tx: float,
    ty: float,
    step: float,
) -> Tuple[int, int, int]:
    inv = 1.0 / step
    stx = int(round(tx * inv))
    sty = int(round(ty * inv))
    hits = 0
    for ix, iy in occ_official:
        if (ix + stx, iy + sty) in occ_ours:
            hits += 1
    return hits, len(occ_official), len(occ_ours)


def search_translation(
    occ_official: Set[Tuple[int, int]],
    occ_ours: Set[Tuple[int, int]],
    step: float,
    tx_range: Tuple[float, float],
    ty_range: Tuple[float, float],
) -> Tuple[float, float, int, float]:
    best_tx, best_ty = 0.0, 0.0
    best_hits = -1
    best_score = -1.0

    tx0, tx1 = tx_range
    ty0, ty1 = ty_range
    n_tx = int(round((tx1 - tx0) / step)) + 1
    n_ty = int(round((ty1 - ty0) / step)) + 1

    for itx in range(n_tx):
        tx = tx0 + itx * step
        for ity in range(n_ty):
            ty = ty0 + ity * step
            hits, n_off, n_our = score_translation(occ_official, occ_ours, tx, ty, step)
            # Jaccard-style score on occupied overlap
            union = n_off + n_our - hits
            jaccard = hits / union if union > 0 else 0.0
            if hits > best_hits or (hits == best_hits and jaccard > best_score):
                best_hits = hits
                best_score = jaccard
                best_tx, best_ty = tx, ty

    return best_tx, best_ty, best_hits, best_score


def rotation_matrix(theta: float) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    c, s = math.cos(theta), math.sin(theta)
    return ((c, -s), (s, c))


def apply_rotation(points: List[Tuple[float, float]], theta: float) -> List[Tuple[float, float]]:
    c, s = math.cos(theta), math.sin(theta)
    return [(c * x - s * y, s * x + c * y) for x, y in points]


def is_occupied_at(
    x: float,
    y: float,
    width: int,
    height: int,
    pixels: List[int],
    origin: Tuple[float, float, float],
    resolution: float,
    negate: int,
    occupied_thresh: float,
) -> bool:
    ox, oy, _ = origin
    col = int(math.floor((x - ox) / resolution))
    row = int(math.floor((y - oy) / resolution))
    if col < 0 or row < 0 or col >= width or row >= height:
        return False
    v = pixels[row * width + col]
    return occupancy_probability(v, negate) > occupied_thresh


def official_to_team(
    x_off: float,
    y_off: float,
    theta: float,
    tx: float,
    ty: float,
) -> Tuple[float, float]:
    c, s = math.cos(theta), math.sin(theta)
    x_rot = c * x_off - s * y_off
    y_rot = s * x_off + c * y_off
    return x_rot + tx, y_rot + ty


def team_to_official(
    x_team: float,
    y_team: float,
    theta: float,
    tx: float,
    ty: float,
) -> Tuple[float, float]:
    c, s = math.cos(theta), math.sin(theta)
    dx = x_team - tx
    dy = y_team - ty
    # R^T * (p_team - t)  since p_team = R * p_off + t
    return c * dx + s * dy, -s * dx + c * dy


def write_ppm(path: Path, width: int, height: int, rgb_flat: List[int]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    header = f"P6\n{width} {height}\n255\n".encode("ascii")
    with path.open("wb") as f:
        f.write(header)
        f.write(bytes(rgb_flat))


def try_write_png(path: Path, width: int, height: int, rgb_flat: List[int]) -> bool:
    try:
        from PIL import Image  # type: ignore
    except ImportError:
        return False
    img = Image.new("RGB", (width, height))
    img.putdata(list(zip(rgb_flat[0::3], rgb_flat[1::3], rgb_flat[2::3])))
    path.parent.mkdir(parents=True, exist_ok=True)
    img.save(path)
    return True


def render_overlay(
    out_base: Path,
    off_w: int,
    off_h: int,
    off_px: List[int],
    off_meta: Dict,
    our_w: int,
    our_h: int,
    our_px: List[int],
    our_meta: Dict,
    theta: float,
    tx: float,
    ty: float,
    margin_m: float = 0.35,
) -> Tuple[Path, Optional[Path]]:
    """Rasterize both maps in team map frame; official warped by (theta, t)."""
    res = float(our_meta["resolution"])
    negate_off = int(off_meta.get("negate", 0))
    negate_our = int(our_meta.get("negate", 0))
    occ_th_off = float(off_meta.get("occupied_thresh", 0.65))
    occ_th_our = float(our_meta.get("occupied_thresh", 0.65))

    our_pts = occupied_cells(
        our_w, our_h, our_px, our_meta["origin"], res, negate_our, occ_th_our
    )
    off_pts = occupied_cells(
        off_w, off_h, off_px, off_meta["origin"], res, negate_off, occ_th_off
    )
    off_in_team = [official_to_team(x, y, theta, tx, ty) for x, y in off_pts]

    all_pts = our_pts + off_in_team
    if not all_pts:
        raise ValueError("No occupied cells to render")

    xmin, xmax, ymin, ymax = world_bounds(all_pts)
    xmin -= margin_m
    xmax += margin_m
    ymin -= margin_m
    ymax += margin_m

    cols = max(1, int(math.ceil((xmax - xmin) / res)))
    rows = max(1, int(math.ceil((ymax - ymin) / res)))

    # Image row 0 = top (ymax) for intuitive viewing
    pixels_rgb: List[int] = []
    for row in range(rows - 1, -1, -1):
        y = ymin + (row + 0.5) * res
        for col in range(cols):
            x = xmin + (col + 0.5) * res
            team_occ = is_occupied_at(
                x, y, our_w, our_h, our_px, our_meta["origin"],
                res, negate_our, occ_th_our,
            )
            x_off, y_off = team_to_official(x, y, theta, tx, ty)
            off_occ = is_occupied_at(
                x_off, y_off, off_w, off_h, off_px, off_meta["origin"],
                res, negate_off, occ_th_off,
            )
            if team_occ and off_occ:
                r, g, b = COLOR_OVERLAP
            elif team_occ:
                r, g, b = COLOR_TEAM
            elif off_occ:
                r, g, b = COLOR_OFFICIAL
            else:
                r, g, b = COLOR_BG
            pixels_rgb.extend((r, g, b))

    ppm_path = out_base.with_suffix(".ppm")
    write_ppm(ppm_path, cols, rows, pixels_rgb)

    png_path: Optional[Path] = None
    png_candidate = out_base.with_suffix(".png")
    if try_write_png(png_candidate, cols, rows, pixels_rgb):
        png_path = png_candidate

    legend_path = out_base.with_suffix(".txt")
    legend_path.write_text(
        "Map overlay (team map frame)\n"
        f"Transform: p_team = R({math.degrees(theta):.2f} deg) * p_off + ({tx:+.4f}, {ty:+.4f})\n"
        f"Image size: {cols} x {rows} px, {res} m/px\n"
        f"World bounds: x=[{xmin:.3f}, {xmax:.3f}]  y=[{ymin:.3f}, {ymax:.3f}]\n"
        "\nColors:\n"
        "  Gold  = overlap (walls match after transform)\n"
        "  Blue  = team map only\n"
        "  Red   = official map only (after transform)\n"
        "  Gray  = free in both\n",
        encoding="utf-8",
    )
    return ppm_path, png_path


def main() -> int:
    script_dir = Path(__file__).resolve().parent
    repo_map = script_dir.parent.parent.parent / "map"

    parser = argparse.ArgumentParser(
        description="Estimate 2D transform between official and team ROS maps."
    )
    parser.add_argument(
        "--official-yaml",
        type=Path,
        default=repo_map / "compitation.yaml",
        help="Official map YAML (default: CAIR/map/compitation.yaml)",
    )
    parser.add_argument(
        "--ours-yaml",
        type=Path,
        default=script_dir.parent / "src/car_sim/map/map_sim.yaml",
        help="Team map YAML (default: nav_real_ws/.../map_sim.yaml)",
    )
    parser.add_argument(
        "--search-radius",
        type=float,
        default=8.0,
        help="Search +/- this many metres for translation (default: 8)",
    )
    parser.add_argument(
        "--coarse-step",
        type=float,
        default=0.25,
        help="Coarse grid step in metres (default: 0.25)",
    )
    parser.add_argument(
        "--fine-step",
        type=float,
        default=0.05,
        help="Fine grid step in metres (default: 0.05)",
    )
    parser.add_argument(
        "--try-rotation",
        action="store_true",
        help="Also search small yaw offsets (degrees step 1, +/- 5 deg)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Overlay image base path without extension (default: tools/map_compare_overlay)",
    )
    parser.add_argument(
        "--no-image",
        action="store_true",
        help="Skip writing overlay image",
    )
    args = parser.parse_args()

    off_yaml = args.official_yaml.resolve()
    our_yaml = args.ours_yaml.resolve()

    off_meta = load_map_yaml(off_yaml)
    our_meta = load_map_yaml(our_yaml)

    off_pgm = resolve_image_path(off_yaml, str(off_meta.get("image", "map.pgm")))
    our_pgm = resolve_image_path(our_yaml, str(our_meta.get("image", "map.pgm")))

    off_w, off_h, off_px = read_pgm(off_pgm)
    our_w, our_h, our_px = read_pgm(our_pgm)

    res_off = float(off_meta["resolution"])
    res_our = float(our_meta["resolution"])
    if abs(res_off - res_our) > 1e-6:
        print(f"WARNING: resolution differs: official={res_off}, ours={res_our}", file=sys.stderr)

    negate_off = int(off_meta.get("negate", 0))
    negate_our = int(our_meta.get("negate", 0))

    occ_th_off = float(off_meta.get("occupied_thresh", 0.65))
    occ_th_our = float(our_meta.get("occupied_thresh", 0.65))

    off_occ_pts = occupied_cells(
        off_w, off_h, off_px, off_meta["origin"], res_off, negate_off, occ_th_off
    )
    our_occ_pts = occupied_cells(
        our_w, our_h, our_px, our_meta["origin"], res_our, negate_our, occ_th_our
    )

    print("=" * 60)
    print("Map metadata")
    print("=" * 60)
    print(f"Official: {off_yaml}")
    print(f"  PGM: {off_pgm.name}  size={off_w}x{off_h}  resolution={res_off} m")
    print(f"  origin (map frame): {off_meta['origin']}")
    print(f"  occupied cells (p > {occ_th_off}): {len(off_occ_pts)}")
    if off_occ_pts:
        x0, x1, y0, y1 = world_bounds(off_occ_pts)
        print(f"  occupied bounds: x=[{x0:.2f}, {x1:.2f}]  y=[{y0:.2f}, {y1:.2f}]")
    print(f"Team:     {our_yaml}")
    print(f"  PGM: {our_pgm.name}  size={our_w}x{our_h}  resolution={res_our} m")
    print(f"  origin (map frame): {our_meta['origin']}")
    print(f"  occupied cells (p > {occ_th_our}): {len(our_occ_pts)}")
    if our_occ_pts:
        x0, x1, y0, y1 = world_bounds(our_occ_pts)
        print(f"  occupied bounds: x=[{x0:.2f}, {x1:.2f}]  y=[{y0:.2f}, {y1:.2f}]")
    print()
    print("Note: origin difference alone does NOT equal waypoint shift.")
    print(f"  origin_ours - origin_official = "
          f"({our_meta['origin'][0] - off_meta['origin'][0]:.3f}, "
          f"{our_meta['origin'][1] - off_meta['origin'][1]:.3f}) m")
    print()

    step = args.coarse_step
    r = args.search_radius
    occ_off = quantize(off_occ_pts, step)
    occ_our = quantize(our_occ_pts, step)

    best_theta = 0.0
    best_tx, best_ty = 0.0, 0.0
    best_hits = -1
    best_jaccard = -1.0

    thetas = [0.0]
    if args.try_rotation:
        thetas = [math.radians(d) for d in range(-5, 6)]

    for theta in thetas:
        pts = apply_rotation(off_occ_pts, theta) if theta != 0.0 else off_occ_pts
        q_off = quantize(pts, step)
        tx, ty, hits, jac = search_translation(
            q_off, occ_our, step, (-r, r), (-r, r)
        )
        if hits > best_hits or (hits == best_hits and jac > best_jaccard):
            best_theta, best_tx, best_ty = theta, tx, ty
            best_hits, best_jaccard = hits, jac

    # Fine search around coarse optimum
    fine = args.fine_step
    fine_pts = apply_rotation(off_occ_pts, best_theta) if best_theta != 0.0 else off_occ_pts
    q_off_fine = quantize(fine_pts, fine)
    q_our_fine = quantize(our_occ_pts, fine)
    fr = max(fine * 2, args.coarse_step)
    best_tx, best_ty, best_hits, best_jaccard = search_translation(
        q_off_fine,
        q_our_fine,
        fine,
        (best_tx - fr, best_tx + fr),
        (best_ty - fr, best_ty + fr),
    )

    n_off = len(q_off_fine)
    n_our = len(q_our_fine)
    recall = best_hits / n_off if n_off else 0.0
    precision = best_hits / n_our if n_our else 0.0

    print("=" * 60)
    print("Estimated transform (official map -> team map)")
    print("=" * 60)
    print("  p_ours = R(theta) * p_official + t")
    print(f"  theta = {math.degrees(best_theta):.2f} deg  ({best_theta:.4f} rad)")
    print(f"  t     = ({best_tx:+.4f}, {best_ty:+.4f}) m")
    print()
    print("Inverse (team map -> official map):")
    if abs(best_theta) < 1e-9:
        print(f"  x_official = x_ours - ({best_tx:+.4f})")
        print(f"  y_official = y_ours - ({best_ty:+.4f})")
    else:
        print("  Apply R(-theta) to (p_ours - t); see --try-rotation output above.")
    print()
    print("Overlap quality (occupied cells, fine grid):")
    print(f"  matching cells: {best_hits}")
    print(f"  official occupied (quantized): {n_off}")
    print(f"  team occupied (quantized):     {n_our}")
    print(f"  recall (hits/n_off):    {recall:.3f}")
    print(f"  precision (hits/n_our): {precision:.3f}")
    print(f"  Jaccard overlap score:  {best_jaccard:.3f}")
    print()

    if best_jaccard < 0.35:
        print(
            "WARNING: Low Jaccard — official map has more wall pixels than team map;\n"
            "         check precision/recall. If precision is high, translation may still be OK.",
            file=sys.stderr,
        )
    elif best_jaccard < 0.55:
        print("CAUTION: Moderate overlap — verify in RViz before using for coordinates.", file=sys.stderr)

    print("=" * 60)
    print("Example: convert official map pose to team map frame")
    print("=" * 60)
    print("  x_ours = x_off + {:.4f}".format(best_tx))
    print("  y_ours = y_off + {:.4f}".format(best_ty))
    if abs(best_theta) > 1e-6:
        print("  (apply rotation first if theta != 0)")
    print()
    print("This transform aligns OCCUPANCY geometry only.")
    print("GOAL_LIST waypoints still need RViz tuning (non-center stops).")
    print("=" * 60)

    if not args.no_image:
        out_base = args.output
        if out_base is None:
            out_base = script_dir / "map_compare_overlay"
        else:
            out_base = out_base.resolve()
            if out_base.suffix.lower() in (".ppm", ".png", ".txt"):
                out_base = out_base.with_suffix("")

        try:
            ppm_path, png_path = render_overlay(
                out_base,
                off_w, off_h, off_px, off_meta,
                our_w, our_h, our_px, our_meta,
                best_theta, best_tx, best_ty,
            )
            print()
            print("=" * 60)
            print("Overlay image (team map frame)")
            print("=" * 60)
            print(f"  {ppm_path}")
            if png_path:
                print(f"  {png_path}")
            print(f"  {out_base.with_suffix('.txt')}  (legend)")
            print("  Gold=overlap  Blue=team only  Red=official (shifted)")
            print("  Open PPM/PNG with any image viewer to verify alignment.")
        except Exception as exc:
            print(f"ERROR: failed to write overlay: {exc}", file=sys.stderr)
            return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
