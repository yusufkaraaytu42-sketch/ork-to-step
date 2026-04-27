"""
ORK-to-STEP Python Bridge
==========================

Reads an OpenRocket (.ork) design file, extracts the geometry AND materials of
every major component (nose cone, body tube, fins, transitions, motor casing),
builds parametric 3-D solids with **CadQuery**, exports them as STEP files that
ANSYS Mechanical (or any other CAE tool) can import directly, and writes an
ANSYS Engineering Data XML so materials are ready to assign in Workbench.

**Mesh optimisation** : the default “Coarse” quality uses only 6 profile points,
dramatically reducing the number of faces/edges for ANSYS meshing.

Usage
-----
    python ork_to_step.py rocket.ork
    python ork_to_step.py rocket.ork --output step_output --mesh coarse
    python ork_to_step.py --gui          # full GUI
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import sys
import xml.etree.ElementTree as ET
import zipfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import cadquery as cq

logger = logging.getLogger("ork_to_step")

# ---------------------------------------------------------------------------
# Mesh quality presets (very low edge count by default)
# ---------------------------------------------------------------------------

@dataclass
class MeshQuality:
    """
    Controls how many line segments are used to approximate curved profiles
    before they are revolved/extruded into solids.  Fewer segments = fewer
    faces = fewer mesh nodes in ANSYS.

    n_pts      : sample points along the nose-cone / transition profile curve.
                 Each point becomes an edge after the revolve, directly
                 driving the number of circumferential mesh nodes.
    """
    label: str
    n_pts: int          # profile curve samples  (nose cone & transitions)

    @classmethod
    def coarse(cls)  -> "MeshQuality": return cls("Coarse",  6)   # minimal edges
    @classmethod
    def medium(cls)  -> "MeshQuality": return cls("Medium",  12)  # balanced
    @classmethod
    def fine(cls)    -> "MeshQuality": return cls("Fine",    24)  # higher fidelity

    @classmethod
    def from_label(cls, label: str) -> "MeshQuality":
        return {"coarse": cls.coarse(), "medium": cls.medium(),
                "fine": cls.fine()}.get(label.lower(), cls.coarse())


# ---------------------------------------------------------------------------
# Material dataclass
# ---------------------------------------------------------------------------

@dataclass
class MaterialProps:
    name: str = "Unknown"
    mat_type: str = "bulk"          # "bulk", "surface", "line"
    density: float = 0.0            # kg/m³
    # Mechanical (populated from OpenRocket database lookup table)
    youngs_modulus: float = 0.0     # Pa
    poissons_ratio: float = 0.3
    tensile_strength: float = 0.0   # Pa
    # Thermal
    thermal_conductivity: float = 0.0   # W/(m·K)
    specific_heat: float = 0.0          # J/(kg·K)
    thermal_expansion: float = 0.0      # 1/K

    def is_empty(self) -> bool:
        return self.density == 0.0 and self.youngs_modulus == 0.0


# ---------------------------------------------------------------------------
# Geometry dataclasses  (each now carries an optional MaterialProps)
# ---------------------------------------------------------------------------

@dataclass
class NoseConeGeom:
    length: float
    base_radius: float
    thickness: float
    shape: str
    shape_param: float = 0.0
    shoulder_radius: float = 0.0
    shoulder_length: float = 0.0
    shoulder_thickness: float = 0.0
    material: MaterialProps = field(default_factory=MaterialProps)


@dataclass
class BodyTubeGeom:
    length: float
    outer_radius: float
    thickness: float
    material: MaterialProps = field(default_factory=MaterialProps)


@dataclass
class TrapezoidalFinGeom:
    count: int
    root_chord: float
    tip_chord: float
    span: float
    sweep_length: float
    thickness: float
    cant_angle: float = 0.0
    axial_offset: float = 0.0
    material: MaterialProps = field(default_factory=MaterialProps)


@dataclass
class FreeformFinGeom:
    count: int
    thickness: float
    points: List[Tuple[float, float]]
    cant_angle: float = 0.0
    axial_offset: float = 0.0
    material: MaterialProps = field(default_factory=MaterialProps)


@dataclass
class TransitionGeom:
    length: float
    fore_radius: float
    aft_radius: float
    thickness: float
    material: MaterialProps = field(default_factory=MaterialProps)


@dataclass
class RocketGeometry:
    name: str = "Rocket"
    nose: Optional[NoseConeGeom] = None
    body_tubes: List[BodyTubeGeom] = field(default_factory=list)
    trapezoidal_fins: List[TrapezoidalFinGeom] = field(default_factory=list)
    freeform_fins: List[FreeformFinGeom] = field(default_factory=list)
    transitions: List[TransitionGeom] = field(default_factory=list)


# ---------------------------------------------------------------------------
# Built-in material property database
# (OpenRocket uses these names; we add mechanical/thermal properties for ANSYS)
# ---------------------------------------------------------------------------

_MATERIAL_DB: Dict[str, Dict[str, float]] = {
    # ─── Metals ───
    "Aluminum": dict(density=2700, youngs_modulus=69e9, poissons_ratio=0.33,
                     tensile_strength=310e6, thermal_conductivity=205, specific_heat=897, thermal_expansion=23.6e-6),
    "Aluminium": dict(density=2700, youngs_modulus=69e9, poissons_ratio=0.33,
                      tensile_strength=310e6, thermal_conductivity=205, specific_heat=897, thermal_expansion=23.6e-6),
    "Steel": dict(density=7850, youngs_modulus=200e9, poissons_ratio=0.30,
                  tensile_strength=400e6, thermal_conductivity=50, specific_heat=490, thermal_expansion=12e-6),
    "Stainless Steel": dict(density=8000, youngs_modulus=193e9, poissons_ratio=0.29,
                            tensile_strength=515e6, thermal_conductivity=16, specific_heat=500, thermal_expansion=17.3e-6),
    "Titanium": dict(density=4510, youngs_modulus=116e9, poissons_ratio=0.32,
                     tensile_strength=434e6, thermal_conductivity=22, specific_heat=520, thermal_expansion=8.6e-6),
    "Copper": dict(density=8960, youngs_modulus=117e9, poissons_ratio=0.34,
                   tensile_strength=220e6, thermal_conductivity=385, specific_heat=385, thermal_expansion=16.5e-6),
    "Brass": dict(density=8490, youngs_modulus=97e9, poissons_ratio=0.31,
                  tensile_strength=350e6, thermal_conductivity=109, specific_heat=380, thermal_expansion=19e-6),
    # ─── Polymers ───
    "Polypropylene": dict(density=905, youngs_modulus=1.5e9, poissons_ratio=0.42,
                          tensile_strength=35e6, thermal_conductivity=0.22, specific_heat=1920, thermal_expansion=150e-6),
    "Polyethylene": dict(density=960, youngs_modulus=1.1e9, poissons_ratio=0.43,
                         tensile_strength=25e6, thermal_conductivity=0.49, specific_heat=1900, thermal_expansion=100e-6),
    "Nylon": dict(density=1140, youngs_modulus=2.8e9, poissons_ratio=0.39,
                  tensile_strength=80e6, thermal_conductivity=0.25, specific_heat=1670, thermal_expansion=80e-6),
    "Polycarbonate": dict(density=1200, youngs_modulus=2.4e9, poissons_ratio=0.37,
                          tensile_strength=55e6, thermal_conductivity=0.21, specific_heat=1200, thermal_expansion=65e-6),
    "ABS": dict(density=1050, youngs_modulus=2.1e9, poissons_ratio=0.39,
                tensile_strength=40e6, thermal_conductivity=0.17, specific_heat=1386, thermal_expansion=70e-6),
    # ─── Composites / Paper ───
    "Cardboard": dict(density=680, youngs_modulus=4.0e9, poissons_ratio=0.30,
                      tensile_strength=30e6, thermal_conductivity=0.06, specific_heat=1340, thermal_expansion=8e-6),
    "Kraft phenolic": dict(density=1300, youngs_modulus=10e9, poissons_ratio=0.30,
                           tensile_strength=120e6, thermal_conductivity=0.26, specific_heat=1300, thermal_expansion=8e-6),
    "Fiberglass": dict(density=1900, youngs_modulus=72e9, poissons_ratio=0.22,
                       tensile_strength=3450e6, thermal_conductivity=1.0, specific_heat=840, thermal_expansion=5e-6),
    "Fiberglass, G10": dict(density=1850, youngs_modulus=18e9, poissons_ratio=0.22,
                            tensile_strength=310e6, thermal_conductivity=0.29, specific_heat=870, thermal_expansion=14e-6),
    "Carbon fiber": dict(density=1600, youngs_modulus=70e9, poissons_ratio=0.10,
                         tensile_strength=600e6, thermal_conductivity=7.0, specific_heat=710, thermal_expansion=0.5e-6),
    "Balsa": dict(density=130, youngs_modulus=3.5e9, poissons_ratio=0.23,
                  tensile_strength=15e6, thermal_conductivity=0.05, specific_heat=2900, thermal_expansion=4e-6),
    "Plywood": dict(density=600, youngs_modulus=9.5e9, poissons_ratio=0.10,
                    tensile_strength=45e6, thermal_conductivity=0.13, specific_heat=1700, thermal_expansion=5e-6),
}


def _lookup_material(name: str) -> MaterialProps:
    """
    Return a MaterialProps populated from the built-in database.
    Falls back gracefully if the name is not found.
    """
    # Try exact match first, then case-insensitive, then partial match.
    db_key: Optional[str] = None
    if name in _MATERIAL_DB:
        db_key = name
    else:
        low = name.lower()
        for k in _MATERIAL_DB:
            if k.lower() == low:
                db_key = k
                break
        if db_key is None:
            for k in _MATERIAL_DB:
                if k.lower() in low or low in k.lower():
                    db_key = k
                    break

    if db_key is None:
        logger.warning("Material '%s' not in database; using defaults.", name)
        return MaterialProps(name=name, density=0.0)

    d = _MATERIAL_DB[db_key]
    return MaterialProps(
        name=name,
        density=d.get("density", 0.0),
        youngs_modulus=d.get("youngs_modulus", 0.0),
        poissons_ratio=d.get("poissons_ratio", 0.3),
        tensile_strength=d.get("tensile_strength", 0.0),
        thermal_conductivity=d.get("thermal_conductivity", 0.0),
        specific_heat=d.get("specific_heat", 0.0),
        thermal_expansion=d.get("thermal_expansion", 0.0),
    )


# ---------------------------------------------------------------------------
# XML / .ork parsing helpers
# ---------------------------------------------------------------------------

def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        if value is None:
            return default
        for token in str(value).split():
            try:
                return float(token)
            except ValueError:
                continue
        return default
    except (TypeError, ValueError):
        return default


def _text(elem: Optional[ET.Element], default: str = "") -> str:
    if elem is None or elem.text is None:
        return default
    return elem.text.strip()


def _read_ork_xml(ork_path: Path) -> ET.Element:
    raw = ork_path.read_bytes()
    if raw[:2] == b"PK":
        with zipfile.ZipFile(ork_path) as zf:
            candidates = [n for n in zf.namelist() if n.endswith(".ork")]
            if not candidates:
                raise ValueError(f"No .ork entry found inside ZIP {ork_path}")
            xml_bytes = zf.read(candidates[0])
            return ET.fromstring(xml_bytes)
    return ET.parse(ork_path).getroot()


def _parse_material(elem: ET.Element) -> MaterialProps:
    """
    Extract material from a component element.

    OpenRocket XML can have two styles:
        <material type="bulk" density="1300.0">Kraft phenolic</material>
    or
        <material type="bulk" density="1300.0" name="Kraft phenolic"/>
    """
    mat_elem = elem.find("material")
    if mat_elem is None:
        return MaterialProps(name="Unknown")

    # Name: text content or 'name' attribute
    name = (mat_elem.text or "").strip() or mat_elem.get("name", "Unknown").strip()
    mat_type = mat_elem.get("type", "bulk")
    # Density may be overridden directly in the XML
    xml_density = _safe_float(mat_elem.get("density"), 0.0)

    props = _lookup_material(name)
    props.mat_type = mat_type
    if xml_density > 0:
        props.density = xml_density  # prefer the value from the .ork file

    return props


# ---------------------------------------------------------------------------
# Main .ork parser
# ---------------------------------------------------------------------------

def parse_ork(ork_path: Path) -> RocketGeometry:
    root = _read_ork_xml(ork_path)

    rocket_name_elem = root.find(".//rocket/name")
    geom = RocketGeometry(name=_text(rocket_name_elem, ork_path.stem))

    # ── Nose cone ──────────────────────────────────────────────────────────
    nose = next(iter(root.iter("nosecone")), None)
    if nose is not None:
        shape_raw = _text(nose.find("shape"), "ogive").lower()
        geom.nose = NoseConeGeom(
            length=_safe_float(_text(nose.find("length"))),
            base_radius=_safe_float(_text(nose.find("aftradius"))),
            thickness=_safe_float(_text(nose.find("thickness")), 0.002),
            shape=shape_raw,
            shape_param=_safe_float(_text(nose.find("shapeparameter"))),
            shoulder_radius=_safe_float(_text(nose.find("aftshoulderradius"))),
            shoulder_length=_safe_float(_text(nose.find("aftshoulderlength"))),
            shoulder_thickness=_safe_float(_text(nose.find("aftshoulderthickness"))),
            material=_parse_material(nose),
        )

    # ── Body tubes ─────────────────────────────────────────────────────────
    for bt in root.iter("bodytube"):
        geom.body_tubes.append(BodyTubeGeom(
            length=_safe_float(_text(bt.find("length"))),
            outer_radius=_safe_float(_text(bt.find("radius"))),
            thickness=_safe_float(_text(bt.find("thickness")), 0.002),
            material=_parse_material(bt),
        ))

    # ── Trapezoidal fins ───────────────────────────────────────────────────
    for fin in root.iter("trapezoidfinset"):
        axial = fin.find("axialoffset")
        geom.trapezoidal_fins.append(TrapezoidalFinGeom(
            count=int(_safe_float(_text(fin.find("fincount")), 3)),
            root_chord=_safe_float(_text(fin.find("rootchord"))),
            tip_chord=_safe_float(_text(fin.find("tipchord"))),
            span=_safe_float(_text(fin.find("height"))),
            sweep_length=_safe_float(_text(fin.find("sweeplength"))),
            thickness=_safe_float(_text(fin.find("thickness")), 0.003),
            cant_angle=_safe_float(_text(fin.find("cant"))),
            axial_offset=_safe_float(_text(axial)) if axial is not None else 0.0,
            material=_parse_material(fin),
        ))

    # ── Freeform fins ──────────────────────────────────────────────────────
    for fin in root.iter("freeformfinset"):
        pts: List[Tuple[float, float]] = []
        finpoints = fin.find("finpoints")
        if finpoints is not None:
            for p in finpoints.findall("point"):
                try:
                    pts.append((float(p.get("x", "0")), float(p.get("y", "0"))))
                except ValueError:
                    continue
        if not pts:
            continue
        axial = fin.find("axialoffset")
        geom.freeform_fins.append(FreeformFinGeom(
            count=int(_safe_float(_text(fin.find("fincount")), 3)),
            thickness=_safe_float(_text(fin.find("thickness")), 0.003),
            points=pts,
            cant_angle=_safe_float(_text(fin.find("cant"))),
            axial_offset=_safe_float(_text(axial)) if axial is not None else 0.0,
            material=_parse_material(fin),
        ))

    # ── Transitions ────────────────────────────────────────────────────────
    for tr in root.iter("transition"):
        geom.transitions.append(TransitionGeom(
            length=_safe_float(_text(tr.find("length"))),
            fore_radius=_safe_float(_text(tr.find("foreradius"))),
            aft_radius=_safe_float(_text(tr.find("aftradius"))),
            thickness=_safe_float(_text(tr.find("thickness")), 0.002),
            material=_parse_material(tr),
        ))

    return geom


# ---------------------------------------------------------------------------
# Nose-cone profile generators
# ---------------------------------------------------------------------------

def _haack_profile(length: float, radius: float, C: float,
                   n_pts: int = 80) -> List[Tuple[float, float]]:
    pts: List[Tuple[float, float]] = []
    for i in range(n_pts + 1):
        x = length * i / n_pts
        theta = math.acos(1.0 - 2.0 * x / length) if length > 0 else 0.0
        r = (radius / math.sqrt(math.pi)) * math.sqrt(
            theta - math.sin(2.0 * theta) / 2.0 + C * math.sin(theta) ** 3
        )
        pts.append((x, r))
    return pts


def _ogive_profile(length: float, radius: float,
                   n_pts: int = 80) -> List[Tuple[float, float]]:
    rho = (radius ** 2 + length ** 2) / (2.0 * radius) if radius > 0 else length
    pts: List[Tuple[float, float]] = []
    for i in range(n_pts + 1):
        x = length * i / n_pts
        r = math.sqrt(rho ** 2 - (length - x) ** 2) + radius - rho
        pts.append((x, max(r, 0.0)))
    return pts


def _conical_profile(length: float, radius: float,
                     n_pts: int = 80) -> List[Tuple[float, float]]:
    return [(length * i / n_pts,
             radius * (length * i / n_pts) / length if length > 0 else 0.0)
            for i in range(n_pts + 1)]


def _parabolic_profile(length: float, radius: float, k: float = 0.5,
                       n_pts: int = 80) -> List[Tuple[float, float]]:
    k = max(0.0, min(k, 1.0))
    pts: List[Tuple[float, float]] = []
    for i in range(n_pts + 1):
        x = length * i / n_pts
        xn = x / length if length > 0 else 0.0
        r = radius * (2 * xn - k * xn ** 2) / (2 - k) if (2 - k) != 0 else 0.0
        pts.append((x, r))
    return pts


def _power_profile(length: float, radius: float, n: float = 0.5,
                   n_pts: int = 80) -> List[Tuple[float, float]]:
    return [(length * i / n_pts,
             radius * (length * i / n_pts / length) ** n if length > 0 else 0.0)
            for i in range(n_pts + 1)]


def nose_profile(nose: NoseConeGeom, n_pts: int = 80) -> List[Tuple[float, float]]:
    shape = nose.shape.lower().replace(" ", "")
    L, R, C = nose.length, nose.base_radius, nose.shape_param
    if shape in ("haack", "lvhaack", "vonkarman"):
        return _haack_profile(L, R, C, n_pts)
    elif shape == "ogive":
        return _ogive_profile(L, R, n_pts)
    elif shape in ("conical", "cone"):
        return _conical_profile(L, R, n_pts)
    elif shape in ("parabolic", "ellipsoid"):
        return _parabolic_profile(L, R, C if C > 0 else 0.5, n_pts)
    elif shape == "power":
        return _power_profile(L, R, C if C > 0 else 0.5, n_pts)
    else:
        logger.warning("Unknown nose shape '%s'; defaulting to ogive.", shape)
        return _ogive_profile(L, R, n_pts)


# ---------------------------------------------------------------------------
# CadQuery solid builders
# ---------------------------------------------------------------------------

def _build_nose_cone(nose: NoseConeGeom, n_pts: int = 32) -> cq.Workplane:
    profile = nose_profile(nose, n_pts=n_pts)
    if not profile:
        raise ValueError("Empty nose-cone profile.")

    pts = [(x * 1000.0, r * 1000.0) for x, r in profile]
    eps = 0.01
    if pts[0][1] < eps:
        pts[0] = (pts[0][0], eps)

    filtered = [pts[0]]
    for p in pts[1:]:
        if abs(p[0] - filtered[-1][0]) > 1e-6 or abs(p[1] - filtered[-1][1]) > 1e-6:
            filtered.append(p)
    pts = filtered

    wp = cq.Workplane("XZ").moveTo(pts[0][0], pts[0][1])
    for x, r in pts[1:]:
        wp = wp.lineTo(x, r)
    last_x, first_x = pts[-1][0], pts[0][0]
    if abs(pts[-1][1]) > eps:
        wp = wp.lineTo(last_x, 0.0)
    if abs(last_x - first_x) > eps:
        wp = wp.lineTo(first_x, 0.0)
    wp = wp.close()
    solid = wp.revolve(360, (0, 0, 0), (1, 0, 0))

    if nose.shoulder_length > 0 and nose.shoulder_radius > 0:
        sl = nose.shoulder_length * 1000.0
        sr = nose.shoulder_radius * 1000.0
        st = nose.shoulder_thickness * 1000.0 if nose.shoulder_thickness > 0 else nose.thickness * 1000.0
        shoulder = (
            cq.Workplane("YZ")
            .workplane(offset=nose.length * 1000.0)
            .circle(sr).circle(sr - st)
            .extrude(sl)
        )
        solid = solid.union(shoulder)

    return solid


def _build_body_tube(tube: BodyTubeGeom, z_offset: float = 0.0) -> cq.Workplane:
    outer_r = tube.outer_radius * 1000.0
    inner_r = (tube.outer_radius - tube.thickness) * 1000.0
    length = tube.length * 1000.0
    return (
        cq.Workplane("YZ")
        .workplane(offset=z_offset)
        .circle(outer_r).circle(max(inner_r, 0.1))
        .extrude(length)
    )


def _build_trapezoidal_fin(fin: TrapezoidalFinGeom, body_radius: float,
                            axial_start_mm: float) -> cq.Workplane:
    rc, tc, sp = fin.root_chord * 1000.0, fin.tip_chord * 1000.0, fin.span * 1000.0
    sw, th, br = fin.sweep_length * 1000.0, fin.thickness * 1000.0, body_radius * 1000.0

    single = (
        cq.Workplane("XZ").workplane(offset=br)
        .moveTo(0.0, 0.0).lineTo(rc, 0.0).lineTo(sw + tc, sp).lineTo(sw, sp)
        .close().extrude(th)
    )
    single = single.translate((axial_start_mm, 0, 0))

    if fin.count > 1:
        step = 360.0 / fin.count
        parts = single
        for i in range(1, fin.count):
            parts = parts.union(single.rotate((0, 0, 0), (1, 0, 0), step * i))
        return parts
    return single


def _build_freeform_fin(fin: FreeformFinGeom, body_radius: float,
                         axial_start_mm: float) -> cq.Workplane:
    th, br = fin.thickness * 1000.0, body_radius * 1000.0
    pts_mm = [(x * 1000.0, y * 1000.0) for x, y in fin.points]
    if len(pts_mm) < 3:
        raise ValueError("Freeform fin needs at least 3 points.")

    wp = cq.Workplane("XZ").workplane(offset=br).moveTo(*pts_mm[0])
    for x, z in pts_mm[1:]:
        wp = wp.lineTo(x, z)
    single = wp.close().extrude(th)
    single = single.translate((axial_start_mm, 0, 0))

    if fin.count > 1:
        step = 360.0 / fin.count
        parts = single
        for i in range(1, fin.count):
            parts = parts.union(single.rotate((0, 0, 0), (1, 0, 0), step * i))
        return parts
    return single


def _build_transition(tr: TransitionGeom, z_offset: float = 0.0, n_pts: int = 32) -> cq.Workplane:
    fr, ar = tr.fore_radius * 1000.0, tr.aft_radius * 1000.0
    ln, th = tr.length * 1000.0, tr.thickness * 1000.0
    eps = 0.01

    inner_fr = max(fr - th, eps)
    inner_ar = max(ar - th, eps)

    return (
        cq.Workplane("XZ").workplane(offset=z_offset)
        .moveTo(0.0, max(fr, eps)).lineTo(ln, max(ar, eps))
        .lineTo(ln, inner_ar).lineTo(0.0, inner_fr)
        .close().revolve(360, (0, 0, 0), (1, 0, 0))
    )


# ---------------------------------------------------------------------------
# Assembly builder
# ---------------------------------------------------------------------------

def build_rocket(geom: RocketGeometry,
                 quality: Optional[MeshQuality] = None) -> Dict[str, cq.Workplane]:
    """
    Build every component and return a dict {name: solid}.
    *quality* controls profile resolution (node count).  Defaults to Coarse.
    """
    if quality is None:
        quality = MeshQuality.coarse()  # minimal edges by default
    n_pts = quality.n_pts

    parts: Dict[str, cq.Workplane] = {}
    x_cursor = 0.0  # mm

    if geom.nose is not None:
        parts["nose_cone"] = _build_nose_cone(geom.nose, n_pts=n_pts)
        x_cursor += geom.nose.length * 1000.0
        if geom.nose.shoulder_length > 0:
            x_cursor += geom.nose.shoulder_length * 1000.0

    for idx, bt in enumerate(geom.body_tubes):
        name = f"body_tube_{idx}" if len(geom.body_tubes) > 1 else "body_tube"
        parts[name] = _build_body_tube(bt, z_offset=x_cursor)
        tube_end = x_cursor + bt.length * 1000.0

        for fidx, fin in enumerate(geom.trapezoidal_fins):
            fin_x = tube_end - fin.root_chord * 1000.0 + fin.axial_offset * 1000.0
            parts[f"trap_fin_{fidx}"] = _build_trapezoidal_fin(fin, bt.outer_radius, fin_x)

        for fidx, fin in enumerate(geom.freeform_fins):
            fin_x = tube_end - max(p[0] for p in fin.points) * 1000.0 + fin.axial_offset * 1000.0
            parts[f"freeform_fin_{fidx}"] = _build_freeform_fin(fin, bt.outer_radius, fin_x)

        x_cursor = tube_end

    for idx, tr in enumerate(geom.transitions):
        name = f"transition_{idx}" if len(geom.transitions) > 1 else "transition"
        parts[name] = _build_transition(tr, z_offset=x_cursor, n_pts=n_pts)
        x_cursor += tr.length * 1000.0

    return parts


# ---------------------------------------------------------------------------
# STEP export
# ---------------------------------------------------------------------------

def export_step(parts: Dict[str, cq.Workplane], output_dir: Path) -> List[Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    exported: List[Path] = []

    for name, solid in parts.items():
        path = output_dir / f"{name}.step"
        cq.exporters.export(solid, str(path))
        logger.info("Exported %s", path)
        exported.append(path)

    assembly_path = output_dir / "rocket_assembly.step"
    if len(parts) > 1:
        assy = cq.Assembly()
        for name, solid in parts.items():
            assy.add(solid, name=name)
        cq.exporters.assembly.exportAssembly(assy, str(assembly_path))
        logger.info("Exported assembly %s", assembly_path)
        exported.append(assembly_path)
    elif len(parts) == 1:
        only_solid = next(iter(parts.values()))
        cq.exporters.export(only_solid, str(assembly_path))
        exported.append(assembly_path)

    return exported


# ---------------------------------------------------------------------------
# Material export  (ANSYS Engineering Data XML  +  plain-text report)
# ---------------------------------------------------------------------------

def _collect_materials(geom: RocketGeometry) -> Dict[str, MaterialProps]:
    """Return {component_name: MaterialProps} for every component."""
    mats: Dict[str, MaterialProps] = {}

    if geom.nose:
        mats["nose_cone"] = geom.nose.material

    for idx, bt in enumerate(geom.body_tubes):
        name = f"body_tube_{idx}" if len(geom.body_tubes) > 1 else "body_tube"
        mats[name] = bt.material

    for idx, fin in enumerate(geom.trapezoidal_fins):
        mats[f"trap_fin_{idx}"] = fin.material

    for idx, fin in enumerate(geom.freeform_fins):
        mats[f"freeform_fin_{idx}"] = fin.material

    for idx, tr in enumerate(geom.transitions):
        name = f"transition_{idx}" if len(geom.transitions) > 1 else "transition"
        mats[name] = tr.material

    return mats


def export_materials(geom: RocketGeometry, output_dir: Path) -> List[Path]:
    """
    Write:
      ansys_materials.xml   — ANSYS Engineering Data XML (importable in Workbench)
      materials_report.txt  — human-readable summary
    """
    output_dir.mkdir(parents=True, exist_ok=True)
    mats = _collect_materials(geom)
    # Deduplicate by material name for the ANSYS XML
    unique_mats: Dict[str, MaterialProps] = {}
    for props in mats.values():
        if props.name not in unique_mats:
            unique_mats[props.name] = props

    exported: List[Path] = []

    # ── ANSYS Engineering Data XML ─────────────────────────────────────────
    xml_path = output_dir / "ansys_materials.xml"
    _write_ansys_engdata_xml(unique_mats, mats, geom.name, xml_path)
    exported.append(xml_path)
    logger.info("Exported %s", xml_path)

    # ── Plain-text report ──────────────────────────────────────────────────
    txt_path = output_dir / "materials_report.txt"
    _write_materials_txt(geom.name, mats, unique_mats, txt_path)
    exported.append(txt_path)
    logger.info("Exported %s", txt_path)

    return exported


def _write_ansys_engdata_xml(unique_mats: Dict[str, MaterialProps],
                              component_mats: Dict[str, MaterialProps],
                              rocket_name: str,
                              path: Path) -> None:
    """
    Write an ANSYS Workbench Engineering Data XML.
    Import via: Engineering Data > File > Import Engineering Data.
    """
    root = ET.Element("EngineeringData", attrib={
        "version": "1",
        "description": f"Materials for {rocket_name} (auto-generated by ork_to_step)",
    })

    # MaterialData section
    mat_data = ET.SubElement(root, "MaterialData")

    for mat_name, props in unique_mats.items():
        mat_el = ET.SubElement(mat_data, "Material", attrib={"Name": mat_name})

        # Structural
        if props.density > 0:
            _ansys_prop(mat_el, "Density", props.density, "kg m^-3")
        if props.youngs_modulus > 0:
            _ansys_prop(mat_el, "Young's Modulus", props.youngs_modulus, "Pa")
            _ansys_prop(mat_el, "Poisson's Ratio", props.poissons_ratio, "")
        if props.tensile_strength > 0:
            _ansys_prop(mat_el, "Tensile Ultimate Strength", props.tensile_strength, "Pa")
        # Thermal
        if props.thermal_conductivity > 0:
            _ansys_prop(mat_el, "Thermal Conductivity", props.thermal_conductivity, "W m^-1 K^-1")
        if props.specific_heat > 0:
            _ansys_prop(mat_el, "Specific Heat", props.specific_heat, "J kg^-1 K^-1")
        if props.thermal_expansion > 0:
            _ansys_prop(mat_el, "Coefficient of Thermal Expansion",
                        props.thermal_expansion, "K^-1")

    # Component-to-material mapping (informational)
    mapping = ET.SubElement(root, "ComponentMaterialMapping",
                            attrib={"description": "Map part name to material"})
    for comp, props in component_mats.items():
        ET.SubElement(mapping, "Part", attrib={"name": comp, "material": props.name})

    tree = ET.ElementTree(root)
    ET.indent(tree, space="  ")
    tree.write(str(path), encoding="utf-8", xml_declaration=True)


def _ansys_prop(parent: ET.Element, prop_name: str, value: float, unit: str) -> None:
    p = ET.SubElement(parent, "Property", attrib={"Name": prop_name})
    if unit:
        p.set("Units", unit)
    ET.SubElement(p, "Value").text = f"{value:.6g}"


def _write_materials_txt(rocket_name: str,
                          component_mats: Dict[str, MaterialProps],
                          unique_mats: Dict[str, MaterialProps],
                          path: Path) -> None:
    lines = [
        "=" * 70,
        f"Material Report  —  {rocket_name}",
        "=" * 70,
        "",
        "  COMPONENT ASSIGNMENTS",
        "  " + "-" * 40,
    ]
    for comp, props in component_mats.items():
        lines.append(f"  {comp:<22s}  →  {props.name}")
    lines += ["", "  UNIQUE MATERIAL PROPERTIES", "  " + "-" * 40]
    for mat_name, props in unique_mats.items():
        lines += [
            f"",
            f"  [{mat_name}]",
            f"    Density              : {props.density:.1f} kg/m³",
            f"    Young's Modulus      : {props.youngs_modulus/1e9:.2f} GPa" if props.youngs_modulus else
            f"    Young's Modulus      : N/A",
            f"    Poisson's Ratio      : {props.poissons_ratio:.3f}",
            f"    Tensile Strength     : {props.tensile_strength/1e6:.1f} MPa" if props.tensile_strength else
            f"    Tensile Strength     : N/A",
            f"    Thermal Conductivity : {props.thermal_conductivity:.3f} W/(m·K)",
            f"    Specific Heat        : {props.specific_heat:.0f} J/(kg·K)",
            f"    Thermal Expansion    : {props.thermal_expansion*1e6:.2f} µm/(m·K)",
        ]
    lines += [
        "",
        "=" * 70,
        "Import ansys_materials.xml in Workbench:",
        "  Engineering Data > File > Import Engineering Data",
        "=" * 70,
    ]
    path.write_text("\n".join(lines), encoding="utf-8")


# ---------------------------------------------------------------------------
# Geometry report helper
# ---------------------------------------------------------------------------

def _report_geometry(geom: RocketGeometry) -> str:
    lines = [f"Rocket: {geom.name}"]
    if geom.nose:
        n = geom.nose
        lines.append(f"  Nose cone  : {n.shape}  L={n.length*1000:.1f} mm  R={n.base_radius*1000:.1f} mm  mat={n.material.name}")
    for i, bt in enumerate(geom.body_tubes):
        lines.append(f"  Body tube {i}: L={bt.length*1000:.1f} mm  R={bt.outer_radius*1000:.1f} mm  t={bt.thickness*1000:.2f} mm  mat={bt.material.name}")
    for i, f in enumerate(geom.trapezoidal_fins):
        lines.append(f"  Trap fin  {i}: {f.count}x  root={f.root_chord*1000:.1f}  tip={f.tip_chord*1000:.1f}  span={f.span*1000:.1f} mm  mat={f.material.name}")
    for i, f in enumerate(geom.freeform_fins):
        lines.append(f"  Free fin  {i}: {f.count}x  {len(f.points)} pts  mat={f.material.name}")
    for i, t in enumerate(geom.transitions):
        lines.append(f"  Transition{i}: L={t.length*1000:.1f} mm  fore_R={t.fore_radius*1000:.1f}  aft_R={t.aft_radius*1000:.1f} mm  mat={t.material.name}")
    return "\n".join(lines)


def _enrich_from_params(geom: RocketGeometry, params_path: Path) -> None:
    try:
        data = json.loads(params_path.read_text(encoding="utf-8"))
    except Exception as exc:
        logger.warning("Could not read %s: %s", params_path, exc)
        return

    if geom.nose is None and "nosecones" in data:
        nc = data["nosecones"]
        geom.nose = NoseConeGeom(
            length=nc.get("length", 0.0),
            base_radius=nc.get("base_radius", 0.02),
            thickness=0.002,
            shape=nc.get("kind", "ogive"),
            material=_lookup_material(nc.get("material", "Unknown")),
        )

    if not geom.body_tubes and "rocket" in data:
        rkt = data["rocket"]
        total_len = rkt.get("total_length", 0.0)
        nose_len = geom.nose.length if geom.nose else 0.0
        tube_len = total_len - nose_len
        if tube_len > 0:
            geom.body_tubes.append(BodyTubeGeom(
                length=tube_len,
                outer_radius=rkt.get("radius", 0.02),
                thickness=0.002,
                material=_lookup_material(rkt.get("material", "Unknown")),
            ))


# ---------------------------------------------------------------------------
# Full conversion pipeline
# ---------------------------------------------------------------------------

def run_conversion(ork_path: Path, params_path: Optional[Path],
                   output_dir: Path,
                   quality: Optional[MeshQuality] = None,
                   log_callback=None) -> Dict[str, Any]:
    """
    Run the full pipeline. Returns a result dict with keys:
        success, message, exported_files, geometry_report, material_report
    """
    if quality is None:
        quality = MeshQuality.coarse()  # minimal edges by default

    def log(msg: str):
        logger.info(msg)
        if log_callback:
            log_callback(msg)

    result: Dict[str, Any] = {
        "success": False, "message": "", "exported_files": [],
        "geometry_report": "", "material_report": "",
    }

    try:
        log(f"Parsing {ork_path.name} ...")
        geom = parse_ork(ork_path)

        if params_path and params_path.exists():
            log(f"Enriching from {params_path.name} ...")
            _enrich_from_params(geom, params_path)

        result["geometry_report"] = _report_geometry(geom)
        log(result["geometry_report"])

        log(f"Building CadQuery solids  [{quality.label}  n_pts={quality.n_pts}] ...")
        parts = build_rocket(geom, quality=quality)
        log(f"Built {len(parts)} part(s): {', '.join(parts)}")

        log(f"Exporting STEP files to {output_dir} ...")
        step_files = export_step(parts, output_dir)
        result["exported_files"].extend(step_files)

        log("Exporting material data ...")
        mat_files = export_materials(geom, output_dir)
        result["exported_files"].extend(mat_files)

        # Build material report string for GUI
        mats = _collect_materials(geom)
        unique = {v.name: v for v in mats.values()}
        mat_lines = ["Component → Material", "-" * 40]
        for comp, props in mats.items():
            mat_lines.append(f"{comp:<22s}  {props.name}")
        mat_lines += ["", "Unique materials:", "-" * 40]
        for name, props in unique.items():
            mat_lines.append(
                f"{name}: ρ={props.density:.0f} kg/m³  E={props.youngs_modulus/1e9:.1f} GPa  ν={props.poissons_ratio:.2f}"
                if props.youngs_modulus > 0
                else f"{name}: ρ={props.density:.0f} kg/m³  (no mechanical data)"
            )
        result["material_report"] = "\n".join(mat_lines)

        result["success"] = True
        result["message"] = (
            f"Done! {len(step_files)} STEP file(s) + {len(mat_files)} material file(s) "
            f"written to: {output_dir}"
        )
        log(result["message"])

    except Exception as exc:
        result["message"] = str(exc)
        logger.exception("Conversion failed")

    return result


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------

def _cli_main() -> None:
    parser = argparse.ArgumentParser(
        description="Convert an OpenRocket .ork design to STEP + material files for ANSYS.",
    )
    parser.add_argument("ork", nargs="?", type=Path, default=None,
                        help="Path to the .ork file.")
    parser.add_argument("--params", type=Path, default=None,
                        help="Path to parameters.json (optional).")
    parser.add_argument("--output", "-o", type=Path, default=Path("step_output"),
                        help="Output directory (default: step_output/).")
    parser.add_argument("--gui", action="store_true", help="Launch GUI.")
    parser.add_argument("--mesh", choices=["coarse", "medium", "fine"], default="coarse",
                        help="Geometry resolution / mesh node density (default: coarse). "
                             "Use 'coarse' to minimize ANSYS node count.")
    parser.add_argument("--verbose", "-v", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(levelname)s: %(message)s",
    )

    if args.gui:
        _gui_main()
        return

    if args.ork is None:
        candidates = list(Path(".").glob("*.ork"))
        if len(candidates) == 1:
            args.ork = candidates[0]
        elif len(candidates) > 1:
            parser.error("Multiple .ork files found; specify one.")
        else:
            parser.error("No .ork file found. Pass one as argument or use --gui.")

    if not args.ork.exists():
        parser.error(f"File not found: {args.ork}")

    result = run_conversion(args.ork, args.params, args.output,
                            quality=MeshQuality.from_label(args.mesh))

    print()
    if result["success"]:
        print("=" * 60)
        print("Exported files:")
        for p in result["exported_files"]:
            print(f"  {p}")
        print("=" * 60)
        print()
        print("Import STEP files into ANSYS Workbench via:")
        print("  Geometry > Import > External Geometry File")
        print()
        print("Import material data via:")
        print("  Engineering Data > File > Import Engineering Data > ansys_materials.xml")
        print()
    else:
        print(f"ERROR: {result['message']}")
        sys.exit(1)


# ---------------------------------------------------------------------------
# GUI (tkinter)  — full-featured with log, geometry tree, material panel
# ---------------------------------------------------------------------------

def _gui_main() -> None:
    try:
        import tkinter as tk
        from tkinter import filedialog, messagebox, ttk, scrolledtext
    except ImportError:
        print("ERROR: tkinter is not available. Install python3-tk or use CLI.", file=sys.stderr)
        sys.exit(1)

    # ── Colours & fonts ───────────────────────────────────────────────────
    BG      = "#1e1e2e"
    PANEL   = "#2a2a3e"
    ACCENT  = "#7c6af7"
    ACCENT2 = "#56cfb2"
    FG      = "#cdd6f4"
    FG2     = "#a6adc8"
    WARN    = "#f38ba8"
    OK      = "#a6e3a1"
    FONT    = ("Segoe UI", 10)
    FONT_B  = ("Segoe UI", 10, "bold")
    FONT_M  = ("Consolas", 9)

    root = tk.Tk()
    root.title("ORK → STEP  |  ANSYS Material Extractor")
    root.geometry("960x680")
    root.configure(bg=BG)
    root.resizable(True, True)

    style = ttk.Style(root)
    style.theme_use("clam")
    style.configure(".",           background=BG,    foreground=FG,    font=FONT)
    style.configure("TFrame",      background=BG)
    style.configure("TLabel",      background=BG,    foreground=FG,    font=FONT)
    style.configure("TEntry",      fieldbackground=PANEL, foreground=FG, insertcolor=FG)
    style.configure("TButton",     background=ACCENT, foreground="#ffffff", font=FONT_B,
                    relief="flat", padding=(10, 6))
    style.map("TButton",           background=[("active", ACCENT2)])
    style.configure("TLabelframe", background=BG,    foreground=ACCENT, font=FONT_B)
    style.configure("TLabelframe.Label", background=BG, foreground=ACCENT)
    style.configure("TNotebook",   background=BG,    tabmargins=[2, 5, 2, 0])
    style.configure("TNotebook.Tab", background=PANEL, foreground=FG2, padding=(10, 4))
    style.map("TNotebook.Tab",     background=[("selected", ACCENT)],
                                   foreground=[("selected", "#ffffff")])
    style.configure("Treeview",    background=PANEL, fieldbackground=PANEL,
                    foreground=FG, rowheight=20, font=FONT_M)
    style.configure("Treeview.Heading", background=ACCENT, foreground="#ffffff", font=FONT_B)
    style.map("Treeview",          background=[("selected", ACCENT2)],
                                   foreground=[("selected", "#000000")])

    # ── State vars ────────────────────────────────────────────────────────
    ork_var    = tk.StringVar()
    params_var = tk.StringVar()
    out_var    = tk.StringVar(value="step_output")
    status_var = tk.StringVar(value="Ready — select an .ork file to begin.")
    mesh_var   = tk.StringVar(value="Coarse")

    # ── Layout: top bar + notebook + status ───────────────────────────────
    top = ttk.Frame(root, padding=10)
    top.pack(fill="x")

    def _lbl(parent, text):
        return ttk.Label(parent, text=text, foreground=FG2)

    def _entry(parent, var, width=52):
        e = ttk.Entry(parent, textvariable=var, width=width)
        return e

    def _browse_btn(parent, cmd, text="Browse…"):
        return ttk.Button(parent, text=text, command=cmd, width=9)

    # Row: ORK file
    r0 = ttk.Frame(top); r0.pack(fill="x", pady=2)
    _lbl(r0, ".ork file:").pack(side="left", padx=(0, 6))
    _entry(r0, ork_var).pack(side="left", padx=(0, 4), expand=True, fill="x")
    def browse_ork():
        p = filedialog.askopenfilename(
            title="Select OpenRocket file",
            filetypes=[("OpenRocket", "*.ork"), ("All files", "*.*")])
        if p:
            ork_var.set(p)
            if not out_var.get() or out_var.get() == "step_output":
                out_var.set(str(Path(p).parent / "step_output"))
    _browse_btn(r0, browse_ork).pack(side="left")

    # Row: params.json
    r1 = ttk.Frame(top); r1.pack(fill="x", pady=2)
    _lbl(r1, "parameters.json:").pack(side="left", padx=(0, 6))
    _entry(r1, params_var).pack(side="left", padx=(0, 4), expand=True, fill="x")
    def browse_params():
        p = filedialog.askopenfilename(
            title="Select parameters.json (optional)",
            filetypes=[("JSON", "*.json"), ("All files", "*.*")])
        if p:
            params_var.set(p)
    _browse_btn(r1, browse_params).pack(side="left")

    # Row: output dir
    r2 = ttk.Frame(top); r2.pack(fill="x", pady=2)
    _lbl(r2, "Output folder:").pack(side="left", padx=(0, 6))
    _entry(r2, out_var).pack(side="left", padx=(0, 4), expand=True, fill="x")
    def browse_out():
        p = filedialog.askdirectory(title="Select output folder")
        if p:
            out_var.set(p)
    _browse_btn(r2, browse_out).pack(side="left")

    # Row: mesh quality
    r_mesh = ttk.Frame(top); r_mesh.pack(fill="x", pady=2)
    _lbl(r_mesh, "Mesh quality:").pack(side="left", padx=(0, 6))
    for label, tip in [("Coarse", "6 points — fewest edges, fastest solve"),
                       ("Medium", "12 points — balanced"),
                       ("Fine",   "24 points — highest fidelity")]:
        rb = tk.Radiobutton(
            r_mesh, text=label, variable=mesh_var, value=label,
            bg=BG, fg=FG, selectcolor=PANEL, activebackground=BG,
            activeforeground=ACCENT2, font=FONT, indicatoron=True,
        )
        rb.pack(side="left", padx=(0, 14))
    ttk.Label(r_mesh, text="← controls profile curve segments → directly affects ANSYS node count",
              foreground=FG2, font=("Segoe UI", 8)).pack(side="left", padx=(4, 0))

    # Action button row
    r3 = ttk.Frame(top); r3.pack(fill="x", pady=(8, 0))
    convert_btn = ttk.Button(r3, text="▶  Convert to STEP + Materials", width=30)
    convert_btn.pack(side="left")
    open_btn = ttk.Button(r3, text="📂 Open Output Folder", width=22)
    open_btn.pack(side="left", padx=(8, 0))
    open_btn.config(state="disabled")

    # ── Notebook (Log / Geometry / Materials) ─────────────────────────────
    nb = ttk.Notebook(root)
    nb.pack(fill="both", expand=True, padx=10, pady=(4, 0))

    # Tab 1: Log
    log_frame = ttk.Frame(nb, padding=4)
    nb.add(log_frame, text=" 📋 Conversion Log ")
    log_text = scrolledtext.ScrolledText(
        log_frame, bg=PANEL, fg=FG, font=FONT_M,
        insertbackground=FG, state="disabled", wrap="word",
        relief="flat", borderwidth=0)
    log_text.pack(fill="both", expand=True)
    log_text.tag_config("ok",   foreground=OK)
    log_text.tag_config("err",  foreground=WARN)
    log_text.tag_config("info", foreground=ACCENT2)
    log_text.tag_config("head", foreground=ACCENT, font=FONT_B)

    # Tab 2: Geometry
    geom_frame = ttk.Frame(nb, padding=4)
    nb.add(geom_frame, text=" 🔩 Geometry ")
    geom_tree = ttk.Treeview(geom_frame, columns=("value",), show="tree headings", selectmode="browse")
    geom_tree.heading("#0",     text="Component")
    geom_tree.heading("value",  text="Properties")
    geom_tree.column("#0",      width=200, stretch=False)
    geom_tree.column("value",   width=500)
    gsb = ttk.Scrollbar(geom_frame, orient="vertical", command=geom_tree.yview)
    geom_tree.configure(yscrollcommand=gsb.set)
    gsb.pack(side="right", fill="y")
    geom_tree.pack(fill="both", expand=True)

    # Tab 3: Materials
    mat_frame = ttk.Frame(nb, padding=4)
    nb.add(mat_frame, text=" 🧪 Materials ")
    mat_pane = tk.PanedWindow(mat_frame, orient="horizontal", bg=BG, sashwidth=6)
    mat_pane.pack(fill="both", expand=True)

    # Left: component → material table
    mat_lf = ttk.LabelFrame(mat_pane, text="Component Assignments", padding=4)
    mat_pane.add(mat_lf, width=320)
    mat_tree = ttk.Treeview(mat_lf, columns=("material",), show="headings")
    mat_tree.heading("material", text="Component → Material")
    mat_tree.column("material", width=300)
    mtsb = ttk.Scrollbar(mat_lf, orient="vertical", command=mat_tree.yview)
    mat_tree.configure(yscrollcommand=mtsb.set)
    mtsb.pack(side="right", fill="y")
    mat_tree.pack(fill="both", expand=True)

    # Right: property detail
    mat_rf = ttk.LabelFrame(mat_pane, text="Material Properties", padding=4)
    mat_pane.add(mat_rf)
    mat_detail = scrolledtext.ScrolledText(
        mat_rf, bg=PANEL, fg=FG, font=FONT_M,
        insertbackground=FG, state="disabled", wrap="word",
        relief="flat", borderwidth=0)
    mat_detail.pack(fill="both", expand=True)
    mat_detail.tag_config("prop",  foreground=ACCENT2)
    mat_detail.tag_config("label", foreground=FG2)
    mat_detail.tag_config("head",  foreground=ACCENT, font=FONT_B)

    # ── Status bar ────────────────────────────────────────────────────────
    sb = ttk.Frame(root, relief="flat", padding=(10, 4))
    sb.pack(fill="x", side="bottom")
    prog = ttk.Progressbar(sb, mode="indeterminate", length=180)
    prog.pack(side="right", padx=(8, 0))
    ttk.Label(sb, textvariable=status_var, foreground=FG2, font=("Segoe UI", 9)).pack(side="left")

    # ── Helper: append to log ─────────────────────────────────────────────
    def log_append(msg: str, tag: str = ""):
        log_text.config(state="normal")
        log_text.insert("end", msg + "\n", tag)
        log_text.see("end")
        log_text.config(state="disabled")

    def log_clear():
        log_text.config(state="normal")
        log_text.delete("1.0", "end")
        log_text.config(state="disabled")

    # ── Populate geometry tree ─────────────────────────────────────────────
    def populate_geom_tree(geom: "RocketGeometry"):
        for item in geom_tree.get_children():
            geom_tree.delete(item)

        root_id = geom_tree.insert("", "end", text=f"🚀 {geom.name}", values=("",), open=True)

        if geom.nose:
            n = geom.nose
            nc = geom_tree.insert(root_id, "end", text="Nose Cone",
                                  values=(f"{n.shape}  L={n.length*1000:.1f}mm  R={n.base_radius*1000:.1f}mm",))
            geom_tree.insert(nc, "end", text="Material", values=(n.material.name,))
            geom_tree.insert(nc, "end", text="Thickness", values=(f"{n.thickness*1000:.2f} mm",))
            geom_tree.insert(nc, "end", text="Shape param", values=(f"{n.shape_param:.3f}",))

        for i, bt in enumerate(geom.body_tubes):
            label = f"Body Tube {i}" if len(geom.body_tubes) > 1 else "Body Tube"
            bt_id = geom_tree.insert(root_id, "end", text=label,
                                     values=(f"L={bt.length*1000:.1f}mm  Ø{bt.outer_radius*2000:.1f}mm  t={bt.thickness*1000:.2f}mm",))
            geom_tree.insert(bt_id, "end", text="Material", values=(bt.material.name,))

        for i, f in enumerate(geom.trapezoidal_fins):
            fin_id = geom_tree.insert(root_id, "end", text=f"Trap. Fin Set {i}",
                                      values=(f"{f.count}x  root={f.root_chord*1000:.1f}  tip={f.tip_chord*1000:.1f}  span={f.span*1000:.1f}mm",))
            geom_tree.insert(fin_id, "end", text="Material", values=(f.material.name,))

        for i, f in enumerate(geom.freeform_fins):
            fin_id = geom_tree.insert(root_id, "end", text=f"Freeform Fin Set {i}",
                                      values=(f"{f.count}x  {len(f.points)} pts  t={f.thickness*1000:.2f}mm",))
            geom_tree.insert(fin_id, "end", text="Material", values=(f.material.name,))

        for i, t in enumerate(geom.transitions):
            label = f"Transition {i}" if len(geom.transitions) > 1 else "Transition"
            tr_id = geom_tree.insert(root_id, "end", text=label,
                                     values=(f"L={t.length*1000:.1f}mm  fore_R={t.fore_radius*1000:.1f}  aft_R={t.aft_radius*1000:.1f}mm",))
            geom_tree.insert(tr_id, "end", text="Material", values=(t.material.name,))

        geom_tree.item(root_id, open=True)

    # ── Populate materials panel ────────────────────────────────────────────
    _mat_props_cache: Dict[str, MaterialProps] = {}

    def populate_mat_panel(geom: "RocketGeometry"):
        for item in mat_tree.get_children():
            mat_tree.delete(item)
        mat_detail.config(state="normal")
        mat_detail.delete("1.0", "end")
        mat_detail.config(state="disabled")
        _mat_props_cache.clear()

        mats = _collect_materials(geom)
        unique = {v.name: v for v in mats.values()}
        _mat_props_cache.update(unique)

        # Show each component with a tag for its unique material name
        for comp, props in mats.items():
            mat_tree.insert("", "end", iid=comp, text=comp,
                            values=(f"{comp}  →  {props.name}",),
                            tags=(props.name,))
        mat_tree.tag_configure("unknown", foreground=WARN)

    def on_mat_select(event):
        sel = mat_tree.selection()
        if not sel:
            return
        iid = sel[0]
        # Find which material it belongs to (from the tag)
        tags = mat_tree.item(iid, "tags")
        mat_name = tags[0] if tags else "Unknown"
        props = _mat_props_cache.get(mat_name)
        mat_detail.config(state="normal")
        mat_detail.delete("1.0", "end")
        if props:
            mat_detail.insert("end", f"{props.name}\n", "head")
            mat_detail.insert("end", "─" * 36 + "\n", "label")

            def row(label, val):
                mat_detail.insert("end", f"  {label:<28s}", "label")
                mat_detail.insert("end", f"{val}\n", "prop")

            row("Type",                props.mat_type)
            row("Density",             f"{props.density:.1f} kg/m³")
            row("Young's Modulus",     f"{props.youngs_modulus/1e9:.2f} GPa" if props.youngs_modulus else "N/A")
            row("Poisson's Ratio",     f"{props.poissons_ratio:.3f}")
            row("Tensile Strength",    f"{props.tensile_strength/1e6:.1f} MPa" if props.tensile_strength else "N/A")
            row("Thermal Conductivity",f"{props.thermal_conductivity:.3f} W/(m·K)")
            row("Specific Heat",       f"{props.specific_heat:.0f} J/(kg·K)")
            row("Thermal Expansion",   f"{props.thermal_expansion*1e6:.2f} µm/(m·K)")

            if props.is_empty():
                mat_detail.insert("end",
                    "\n⚠ No properties found in database.\n"
                    "  Values will be 0 in ansys_materials.xml.\n"
                    "  Assign manually in ANSYS Engineering Data.\n",
                    "label")
        mat_detail.config(state="disabled")

    mat_tree.bind("<<TreeviewSelect>>", on_mat_select)

    # ── Conversion logic ───────────────────────────────────────────────────
    _last_result: Dict[str, Any] = {}

    def do_convert():
        ork_path = ork_var.get().strip()
        if not ork_path:
            messagebox.showerror("Missing file", "Please select an .ork file.", parent=root)
            return
        ork_p = Path(ork_path)
        if not ork_p.exists():
            messagebox.showerror("File not found", f"{ork_p}", parent=root)
            return

        params_p = Path(params_var.get().strip()) if params_var.get().strip() else None
        out_p    = Path(out_var.get().strip() or "step_output")
        quality  = MeshQuality.from_label(mesh_var.get())

        log_clear()
        log_append(f"=== Starting conversion: {ork_p.name} ===", "head")

        convert_btn.config(state="disabled")
        open_btn.config(state="disabled")
        prog.start(12)
        status_var.set("Converting …")
        root.update()

        def worker():
            res = run_conversion(ork_p, params_p, out_p,
                                 quality=quality,
                                 log_callback=lambda m: root.after(0, lambda: log_append(m)))
            root.after(0, lambda: on_done(res))

        import threading
        threading.Thread(target=worker, daemon=True).start()

    def on_done(res: Dict[str, Any]):
        prog.stop()
        convert_btn.config(state="normal")
        _last_result.update(res)

        if res["success"]:
            status_var.set("✔ Conversion complete")
            log_append("\n✔ " + res["message"], "ok")
            open_btn.config(state="normal")

            # Rebuild geometry & material tabs
            try:
                geom = parse_ork(Path(ork_var.get().strip()))
                params_p = Path(params_var.get().strip()) if params_var.get().strip() else None
                if params_p and params_p.exists():
                    _enrich_from_params(geom, params_p)
                populate_geom_tree(geom)
                populate_mat_panel(geom)
                nb.select(1)   # switch to geometry tab
            except Exception:
                pass
        else:
            status_var.set("✖ Conversion failed")
            log_append("\n✖ ERROR: " + res["message"], "err")
            messagebox.showerror("Conversion failed", res["message"], parent=root)

    def open_output():
        out_p = Path(out_var.get().strip() or "step_output")
        if out_p.exists():
            import subprocess, platform
            if platform.system() == "Windows":
                subprocess.Popen(["explorer", str(out_p)])
            elif platform.system() == "Darwin":
                subprocess.Popen(["open", str(out_p)])
            else:
                subprocess.Popen(["xdg-open", str(out_p)])

    convert_btn.config(command=do_convert)
    open_btn.config(command=open_output)

    root.mainloop()


# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import sys
    if len(sys.argv) == 1:
        # No arguments → launch GUI
        _gui_main()
    else:
        _cli_main()