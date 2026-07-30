#!/usr/bin/env python3
"""
Generate first-pass 3D-printable enclosure STLs for the expression controller.

This deliberately avoids CAD dependencies so it can run anywhere Python runs.
The geometry is parametric and prototype-oriented: print, test-fit, then tune
the constants below for the exact jacks, encoder nut, and OLED board in hand.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, sin, pi, sqrt
from pathlib import Path
from typing import Callable, Iterable, List, Sequence, Tuple

Point = Tuple[float, float, float]
Triangle = Tuple[Point, Point, Point]


@dataclass(frozen=True)
class RectHole:
    cx: float
    cy: float
    w: float
    h: float

    def contains(self, x: float, y: float) -> bool:
        return abs(x - self.cx) <= self.w / 2 and abs(y - self.cy) <= self.h / 2


@dataclass(frozen=True)
class CircleHole:
    cx: float
    cy: float
    diameter: float

    def contains(self, x: float, y: float) -> bool:
        r = self.diameter / 2
        return (x - self.cx) ** 2 + (y - self.cy) ** 2 <= r * r


# Main enclosure dimensions. This fits comfortably on a Prusa Mini bed.
OUTER_WIDTH = 126.0
OUTER_DEPTH = 112.0
BODY_HEIGHT = 34.0
WALL = 2.4
BOTTOM = 2.4
LID_THICKNESS = 3.0
LID_OVERHANG = 0.4

# Panel cutouts.
OLED_WINDOW_W = 30.0
OLED_WINDOW_H = 11.0
OLED_CENTER_Y = 18.0
ENCODER_HOLE_D = 7.4
ENCODER_CENTER_Y = -18.0

# Side cutouts. Most 1/4" panel jacks want about 9.5-10 mm.
TRS_HOLE_D = 10.5
TRS_CENTER_Z = 18.0
LFO_HOLE_D = 6.5
LFO_CENTER_X_OFFSET = 16.0
LFO_CENTER_Z = 16.0
USB_WINDOW_W = 18.0
USB_WINDOW_H = 10.0
USB_CENTER_Z = 16.0

# Lid screw/post geometry.
SCREW_HOLE_D = 2.9
POST_OUTER_D = 8.0
POST_INNER_D = 2.7
POST_MARGIN_X = 10.0
POST_MARGIN_Y = 8.0

# Perfboard mounting geometry. This targets the 70 mm x 90 mm A-Z / 1-31
# perfboard used in the docs. Measure the real board before printing and tune
# PERFBOARD_MOUNT_HOLE_INSET_MM if its corner mounting holes differ.
PERFBOARD_WIDTH = 70.0
PERFBOARD_DEPTH = 90.0
PERFBOARD_MOUNT_HOLE_INSET_MM = 3.0
PERFBOARD_POST_HEIGHT = 7.0
PERFBOARD_POST_OUTER_D = 7.0
PERFBOARD_POST_INNER_D = 2.7

# Mesh resolution for cutout edges. Smaller is cleaner but larger STL.
GRID = 0.8


def normal(a: Point, b: Point, c: Point) -> Point:
    ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
    vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
    nx = uy * vz - uz * vy
    ny = uz * vx - ux * vz
    nz = ux * vy - uy * vx
    length = sqrt(nx * nx + ny * ny + nz * nz)
    if length == 0:
        return (0.0, 0.0, 0.0)
    return (nx / length, ny / length, nz / length)


def add_quad(mesh: List[Triangle], a: Point, b: Point, c: Point, d: Point) -> None:
    mesh.append((a, b, c))
    mesh.append((a, c, d))


def add_box(mesh: List[Triangle], xmin: float, xmax: float, ymin: float, ymax: float,
            zmin: float, zmax: float) -> None:
    p000 = (xmin, ymin, zmin)
    p100 = (xmax, ymin, zmin)
    p110 = (xmax, ymax, zmin)
    p010 = (xmin, ymax, zmin)
    p001 = (xmin, ymin, zmax)
    p101 = (xmax, ymin, zmax)
    p111 = (xmax, ymax, zmax)
    p011 = (xmin, ymax, zmax)

    add_quad(mesh, p000, p100, p110, p010)
    add_quad(mesh, p001, p011, p111, p101)
    add_quad(mesh, p000, p001, p101, p100)
    add_quad(mesh, p100, p101, p111, p110)
    add_quad(mesh, p110, p111, p011, p010)
    add_quad(mesh, p010, p011, p001, p000)


def add_hollow_cylinder(mesh: List[Triangle], cx: float, cy: float, z0: float, z1: float,
                        outer_d: float, inner_d: float, segments: int = 48) -> None:
    ro = outer_d / 2
    ri = inner_d / 2
    for i in range(segments):
        a0 = 2 * pi * i / segments
        a1 = 2 * pi * (i + 1) / segments
        o0b = (cx + ro * cos(a0), cy + ro * sin(a0), z0)
        o1b = (cx + ro * cos(a1), cy + ro * sin(a1), z0)
        o0t = (cx + ro * cos(a0), cy + ro * sin(a0), z1)
        o1t = (cx + ro * cos(a1), cy + ro * sin(a1), z1)
        i0b = (cx + ri * cos(a0), cy + ri * sin(a0), z0)
        i1b = (cx + ri * cos(a1), cy + ri * sin(a1), z0)
        i0t = (cx + ri * cos(a0), cy + ri * sin(a0), z1)
        i1t = (cx + ri * cos(a1), cy + ri * sin(a1), z1)

        add_quad(mesh, o0b, o1b, o1t, o0t)  # outer wall
        add_quad(mesh, i1b, i0b, i0t, i1t)  # inner wall
        add_quad(mesh, o0t, o1t, i1t, i0t)  # top ring
        add_quad(mesh, o1b, o0b, i0b, i1b)  # bottom ring


def frange(start: float, stop: float, step: float) -> List[float]:
    vals = [start]
    value = start
    while value + step < stop:
        value += step
        vals.append(value)
    if vals[-1] != stop:
        vals.append(stop)
    return vals


def add_grid_plate(mesh: List[Triangle], umin: float, umax: float, vmin: float, vmax: float,
                   w0: float, w1: float, holes: Sequence[object],
                   mapper: Callable[[float, float, float], Point],
                   grid: float = GRID) -> None:
    us = frange(umin, umax, grid)
    vs = frange(vmin, vmax, grid)
    nu = len(us) - 1
    nv = len(vs) - 1

    solid = [[False for _ in range(nv)] for _ in range(nu)]
    for i in range(nu):
        uc = (us[i] + us[i + 1]) / 2
        for j in range(nv):
            vc = (vs[j] + vs[j + 1]) / 2
            solid[i][j] = not any(h.contains(uc, vc) for h in holes)

    def p(u: float, v: float, w: float) -> Point:
        return mapper(u, v, w)

    for i in range(nu):
        for j in range(nv):
            if not solid[i][j]:
                continue
            u0, u1 = us[i], us[i + 1]
            v0, v1 = vs[j], vs[j + 1]

            add_quad(mesh, p(u0, v0, w1), p(u1, v0, w1), p(u1, v1, w1), p(u0, v1, w1))
            add_quad(mesh, p(u0, v1, w0), p(u1, v1, w0), p(u1, v0, w0), p(u0, v0, w0))

            if i == 0 or not solid[i - 1][j]:
                add_quad(mesh, p(u0, v1, w0), p(u0, v0, w0), p(u0, v0, w1), p(u0, v1, w1))
            if i == nu - 1 or not solid[i + 1][j]:
                add_quad(mesh, p(u1, v0, w0), p(u1, v1, w0), p(u1, v1, w1), p(u1, v0, w1))
            if j == 0 or not solid[i][j - 1]:
                add_quad(mesh, p(u0, v0, w0), p(u1, v0, w0), p(u1, v0, w1), p(u0, v0, w1))
            if j == nv - 1 or not solid[i][j + 1]:
                add_quad(mesh, p(u1, v1, w0), p(u0, v1, w0), p(u0, v1, w1), p(u1, v1, w1))


def make_lid() -> List[Triangle]:
    mesh: List[Triangle] = []
    w = OUTER_WIDTH + 2 * LID_OVERHANG
    d = OUTER_DEPTH + 2 * LID_OVERHANG
    holes = [
        RectHole(0.0, OLED_CENTER_Y, OLED_WINDOW_W, OLED_WINDOW_H),
        CircleHole(0.0, ENCODER_CENTER_Y, ENCODER_HOLE_D),
    ]
    for sx in (-1, 1):
        for sy in (-1, 1):
            holes.append(CircleHole(sx * (OUTER_WIDTH / 2 - POST_MARGIN_X),
                                    sy * (OUTER_DEPTH / 2 - POST_MARGIN_Y),
                                    SCREW_HOLE_D))

    add_grid_plate(
        mesh,
        -w / 2, w / 2,
        -d / 2, d / 2,
        0.0, LID_THICKNESS,
        holes,
        lambda u, v, z: (u, v, z),
    )
    return mesh


def make_body() -> List[Triangle]:
    mesh: List[Triangle] = []
    w = OUTER_WIDTH
    d = OUTER_DEPTH
    h = BODY_HEIGHT
    inner_w = w - 2 * WALL
    inner_d = d - 2 * WALL

    # Bottom plate.
    add_box(mesh, -w / 2, w / 2, -d / 2, d / 2, 0.0, BOTTOM)

    # Left and right side walls with TRS jack cutouts.
    side_holes = [CircleHole(0.0, TRS_CENTER_Z, TRS_HOLE_D)]
    add_grid_plate(mesh, -inner_d / 2, inner_d / 2, 0.0, h,
                   -w / 2, -w / 2 + WALL, side_holes,
                   lambda y, z, x: (x, y, z))
    add_grid_plate(mesh, -inner_d / 2, inner_d / 2, 0.0, h,
                   w / 2 - WALL, w / 2, side_holes,
                   lambda y, z, x: (x, y, z))

    # Front wall with two 3.5 mm LFO output jack cutouts.
    front_holes = [
        CircleHole(-LFO_CENTER_X_OFFSET, LFO_CENTER_Z, LFO_HOLE_D),
        CircleHole(LFO_CENTER_X_OFFSET, LFO_CENTER_Z, LFO_HOLE_D),
    ]
    add_grid_plate(mesh, -inner_w / 2, inner_w / 2, 0.0, h,
                   d / 2 - WALL, d / 2, front_holes,
                   lambda x, z, y: (x, y, z))

    # Rear wall with USB/service cutout.
    rear_holes = [RectHole(0.0, USB_CENTER_Z, USB_WINDOW_W, USB_WINDOW_H)]
    add_grid_plate(mesh, -inner_w / 2, inner_w / 2, 0.0, h,
                   -d / 2, -d / 2 + WALL, rear_holes,
                   lambda x, z, y: (x, y, z))

    # Lid screw posts.
    for sx in (-1, 1):
        for sy in (-1, 1):
            add_hollow_cylinder(
                mesh,
                sx * (OUTER_WIDTH / 2 - POST_MARGIN_X),
                sy * (OUTER_DEPTH / 2 - POST_MARGIN_Y),
                BOTTOM,
                h - 1.0,
                POST_OUTER_D,
                POST_INNER_D,
            )

    # Short internal posts for the full-size 70 mm x 90 mm perfboard.
    board_post_x = PERFBOARD_WIDTH / 2 - PERFBOARD_MOUNT_HOLE_INSET_MM
    board_post_y = PERFBOARD_DEPTH / 2 - PERFBOARD_MOUNT_HOLE_INSET_MM
    for sx in (-1, 1):
        for sy in (-1, 1):
            add_hollow_cylinder(
                mesh,
                sx * board_post_x,
                sy * board_post_y,
                BOTTOM,
                BOTTOM + PERFBOARD_POST_HEIGHT,
                PERFBOARD_POST_OUTER_D,
                PERFBOARD_POST_INNER_D,
            )

    return mesh


def write_ascii_stl(path: Path, name: str, mesh: Iterable[Triangle]) -> None:
    with path.open("w", encoding="ascii") as f:
        f.write(f"solid {name}\n")
        for tri in mesh:
            n = normal(*tri)
            f.write(f"  facet normal {n[0]:.6g} {n[1]:.6g} {n[2]:.6g}\n")
            f.write("    outer loop\n")
            for p in tri:
                f.write(f"      vertex {p[0]:.6g} {p[1]:.6g} {p[2]:.6g}\n")
            f.write("    endloop\n")
            f.write("  endfacet\n")
        f.write(f"endsolid {name}\n")


def main() -> None:
    out_dir = Path(__file__).resolve().parent / "stl"
    out_dir.mkdir(parents=True, exist_ok=True)
    write_ascii_stl(out_dir / "expression_controller_body.stl", "expression_controller_body", make_body())
    write_ascii_stl(out_dir / "expression_controller_lid.stl", "expression_controller_lid", make_lid())
    print(f"Wrote {out_dir / 'expression_controller_body.stl'}")
    print(f"Wrote {out_dir / 'expression_controller_lid.stl'}")


if __name__ == "__main__":
    main()
