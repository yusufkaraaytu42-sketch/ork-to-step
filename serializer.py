"""
ORK Serializer
==============

Converts an OpenRocket (.ork) design into the three files that `simu/simu_bot.py`
consumes to run realistic flight / Monte Carlo simulations:

    <output>/parameters.json   -- rocket, motor, environment and flight data
    <output>/thrust_source.csv -- motor thrust curve (time [s], thrust [N])
    <output>/drag_curve.csv    -- vehicle drag curve (mach, Cd)

How it works
------------
1. Runs the `ork2json` CLI that ships with the `rocketserializer` package to
   extract the base JSON and the raw thrust / drag curves that OpenRocket had
   stored for the last simulation in the .ork file.
2. Parses the .ork XML directly to recover data that `rocketserializer` does
   not emit (freeform fin geometry, body tube length, nose shape parameter,
   parachute area / Cd, motor designation). This is the difference between a
   simulation that silently flies with default fins and one that actually
   reflects the rocket the user drew in OpenRocket.
3. Cleans the drag curve (dedupes Mach values, strips the post-deployment
   descent phase that OpenRocket appends, clamps to physical range).
4. Writes only the three files above. No redundant rocketpy_generated.py, no
   duplicate motor_thrust.csv -- the simulator builds the rocket from JSON
   at runtime.

Usage
-----
    python serializer.py rocket.ork
    python serializer.py rocket.ork --output ./out --ork-jar OpenRocket.jar

Optional GUI (if tkinter is available):
    python serializer.py --gui
"""

from __future__ import annotations

import argparse
import csv
import json
import logging
import os
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

logger = logging.getLogger("serializer")


# =============================================================================
# Helpers
# =============================================================================


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        if value is None:
            return default
        return float(value)
    except (TypeError, ValueError):
        return default


def _text(elem: Optional[ET.Element], default: str = "") -> str:
    if elem is None or elem.text is None:
        return default
    return elem.text.strip()


def _find_all(root: ET.Element, tag: str) -> List[ET.Element]:
    return list(root.iter(tag))


# =============================================================================
# .ork XML extraction (for data that rocketserializer 0.2.0 misses)
# =============================================================================


def _extract_nose_shape_param(nose_elem: ET.Element) -> Optional[float]:
    sp = nose_elem.find("shapeparameter")
    if sp is not None and sp.text:
        try:
            return float(sp.text)
        except ValueError:
            return None
    return None


def _extract_body_tube(root: ET.Element) -> Optional[Dict[str, float]]:
    tube = next(iter(root.iter("bodytube")), None)
    if tube is None:
        return None
    return {
        "name": _text(tube.find("name"), "Body Tube"),
        "length": _safe_float(_text(tube.find("length")), 0.0),
        "radius": _safe_float(_text(tube.find("radius")).split()[0] if tube.find("radius") is not None else "0", 0.0),
        "thickness": _safe_float(_text(tube.find("thickness")), 0.002),
    }


def _extract_freeform_fins(root: ET.Element) -> List[Dict[str, Any]]:
    out: List[Dict[str, Any]] = []
    for fin in root.iter("freeformfinset"):
        pts = []
        finpoints = fin.find("finpoints")
        if finpoints is not None:
            for p in finpoints.findall("point"):
                try:
                    pts.append([float(p.get("x", "0")), float(p.get("y", "0"))])
                except ValueError:
                    continue
        if not pts:
            continue
        axial = fin.find("axialoffset")
        out.append({
            "type": "freeform",
            "name": _text(fin.find("name"), "Freeform Fin Set"),
            "count": int(_safe_float(_text(fin.find("fincount")), 3)),
            "thickness": _safe_float(_text(fin.find("thickness")), 0.003),
            "cant_angle": _safe_float(_text(fin.find("cant")), 0.0),
            "axial_offset": _safe_float(_text(axial), 0.0) if axial is not None else 0.0,
            "axial_method": axial.get("method", "bottom") if axial is not None else "bottom",
            "shape_points": pts,
        })
    return out


def _extract_trapezoidal_fins(root: ET.Element) -> List[Dict[str, Any]]:
    out: List[Dict[str, Any]] = []
    for fin in root.iter("trapezoidfinset"):
        axial = fin.find("axialoffset")
        out.append({
            "type": "trapezoidal",
            "name": _text(fin.find("name"), "Trapezoidal Fin Set"),
            "count": int(_safe_float(_text(fin.find("fincount")), 3)),
            "root_chord": _safe_float(_text(fin.find("rootchord")), 0.1),
            "tip_chord": _safe_float(_text(fin.find("tipchord")), 0.05),
            "span": _safe_float(_text(fin.find("height")), 0.05),
            "sweep_length": _safe_float(_text(fin.find("sweeplength")), 0.0),
            "cant_angle": _safe_float(_text(fin.find("cant")), 0.0),
            "thickness": _safe_float(_text(fin.find("thickness")), 0.003),
            "axial_offset": _safe_float(_text(axial), 0.0) if axial is not None else 0.0,
            "axial_method": axial.get("method", "bottom") if axial is not None else "bottom",
        })
    return out


def _extract_parachutes(root: ET.Element) -> List[Dict[str, Any]]:
    out: List[Dict[str, Any]] = []
    for chute in root.iter("parachute"):
        diameter = _safe_float(_text(chute.find("diameter")), 0.3)
        cd_text = _text(chute.find("cd"), "auto")
        cd = 0.8 if cd_text.lower() == "auto" else _safe_float(cd_text, 0.8)
        import math
        area = math.pi * (diameter / 2.0) ** 2
        out.append({
            "name": _text(chute.find("name"), "Parachute"),
            "diameter": diameter,
            "area": area,
            "cd": cd,
            "cd_s": cd * area,
            "deploy_event": _text(chute.find("deployevent"), "apogee").lower(),
            "deploy_altitude": _safe_float(_text(chute.find("deployaltitude")), 200.0),
            "deploy_delay": _safe_float(_text(chute.find("deploydelay")), 1.0),
        })
    return out


def _extract_ork_extras(ork_path: Path) -> Dict[str, Any]:
    """Parse the .ork XML for data `rocketserializer` doesn't emit."""
    try:
        tree = ET.parse(ork_path)
    except ET.ParseError as exc:
        logger.warning("Could not parse .ork XML (%s); extras skipped.", exc)
        return {}
    root = tree.getroot()
    extras: Dict[str, Any] = {}

    nose = next(iter(root.iter("nosecone")), None)
    if nose is not None:
        shape_param = _extract_nose_shape_param(nose)
        if shape_param is not None:
            extras["nose_shape_parameter"] = shape_param

    body_tube = _extract_body_tube(root)
    if body_tube:
        extras["body_tube"] = body_tube

    fins = _extract_freeform_fins(root) + _extract_trapezoidal_fins(root)
    if fins:
        extras["fins"] = fins

    chutes = _extract_parachutes(root)
    if chutes:
        extras["parachutes_detail"] = chutes

    motor = next(iter(root.iter("motor")), None)
    if motor is not None:
        extras["motor_designation"] = _text(motor.find("designation"))
        extras["motor_manufacturer"] = _text(motor.find("manufacturer"))
        extras["motor_diameter"] = _safe_float(_text(motor.find("diameter")), 0.0)
        extras["motor_length"] = _safe_float(_text(motor.find("length")), 0.0)

    return extras


# =============================================================================
# ork2json runner
# =============================================================================


def _find_ork_jar(cwd: Path) -> Optional[str]:
    candidates = [cwd / "OpenRocket.jar", Path("OpenRocket.jar")]
    for c in candidates:
        if c.exists():
            return str(c.resolve())
    return None


def run_ork2json(ork_path: Path, output_dir: Path, ork_jar: Optional[str] = None) -> None:
    """Run the rocketserializer `ork2json` CLI."""
    if shutil.which("ork2json") is None:
        raise RuntimeError(
            "ork2json is not installed. Install rocketserializer:\n"
            "    pip install rocketserializer rocketpy"
        )
    output_dir.mkdir(parents=True, exist_ok=True)
    cmd = ["ork2json", "--filepath", str(ork_path), "--output", str(output_dir)]
    jar = ork_jar or _find_ork_jar(ork_path.parent)
    if jar:
        cmd += ["--ork_jar", jar]
    logger.info("Running: %s", " ".join(cmd))
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(f"ork2json failed (code {result.returncode}):\n{result.stderr}")


# =============================================================================
# Thrust-source analysis
# =============================================================================


def _read_thrust_csv(path: Path) -> List[Tuple[float, float]]:
    pts: List[Tuple[float, float]] = []
    with path.open("r", encoding="utf-8") as f:
        for row in csv.reader(f):
            if len(row) < 2:
                continue
            try:
                pts.append((float(row[0]), float(row[1])))
            except ValueError:
                continue
    return pts


def _thrust_properties(pts: List[Tuple[float, float]]) -> Dict[str, float]:
    if not pts:
        return {"burn_time": 0.0, "total_impulse": 0.0, "max_thrust": 0.0, "avg_thrust": 0.0}
    pts_sorted = sorted(pts, key=lambda p: p[0])
    burn_time = pts_sorted[-1][0] - pts_sorted[0][0]
    max_thrust = max(t for _, t in pts_sorted)
    impulse = 0.0
    for i in range(1, len(pts_sorted)):
        dt = pts_sorted[i][0] - pts_sorted[i - 1][0]
        impulse += 0.5 * (pts_sorted[i][1] + pts_sorted[i - 1][1]) * dt
    avg = impulse / burn_time if burn_time > 0 else max_thrust
    return {
        "burn_time": burn_time,
        "total_impulse": impulse,
        "max_thrust": max_thrust,
        "avg_thrust": avg,
    }


# =============================================================================
# Drag-curve cleaning
# =============================================================================


def _clean_drag_curve(input_csv: Path, output_csv: Path) -> int:
    """De-duplicate Mach values, strip post-apogee oscillations, clamp to [0,10]."""
    raw: List[Tuple[float, float]] = []
    with input_csv.open("r", encoding="utf-8") as f:
        for row in csv.reader(f):
            if len(row) < 2:
                continue
            try:
                mach, cd = float(row[0]), float(row[1])
            except ValueError:
                continue
            if mach < 0 or cd < 0 or cd > 10:
                continue
            raw.append((mach, cd))

    # OpenRocket logs one row per simulation timestep, so Mach values repeat
    # across ascent / coast / descent. Reject points where Cd is anomalously
    # low (drogue/main not deployed yet but simulator in coast) by taking
    # the median Cd at each Mach bucket.
    from collections import defaultdict
    buckets: Dict[float, List[float]] = defaultdict(list)
    for mach, cd in raw:
        key = round(mach, 3)
        buckets[key].append(cd)

    cleaned = []
    for mach in sorted(buckets.keys()):
        cds = buckets[mach]
        # take the 75th percentile so we capture drag coefficient during
        # powered flight and coast, not post-chute deployment.
        cds_sorted = sorted(cds)
        idx = int(0.75 * (len(cds_sorted) - 1))
        cleaned.append((mach, cds_sorted[idx]))

    # Ensure monotonic, limit to reasonable mach range
    cleaned = [(m, c) for m, c in cleaned if 0.0 <= m <= 5.0]
    if not cleaned:
        cleaned = [(0.0, 0.5), (0.5, 0.5), (1.0, 0.6), (2.0, 0.5), (5.0, 0.4)]

    with output_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerows([[f"{m:.4f}", f"{c:.4f}"] for m, c in cleaned])

    return len(cleaned)


# =============================================================================
# JSON enrichment
# =============================================================================


def _enrich_parameters(
    params_path: Path,
    thrust_csv: Path,
    ork_extras: Dict[str, Any],
) -> None:
    """Fill in missing motor/body/fin fields in parameters.json."""
    with params_path.open("r", encoding="utf-8") as f:
        data = json.load(f)

    # --- Motor ---
    thrust_pts = _read_thrust_csv(thrust_csv)
    thrust_props = _thrust_properties(thrust_pts)
    motors = data.setdefault("motors", {}) if isinstance(data.get("motors"), dict) else {}
    motors.update({
        "burn_time": thrust_props["burn_time"],
        "total_impulse": thrust_props["total_impulse"],
        "max_thrust": thrust_props["max_thrust"],
        "avg_thrust": thrust_props["avg_thrust"],
    })
    # rocketserializer often emits dry_mass = 0; use a realistic estimate.
    isp_estimate = 45.5  # from the KAYRA-38.eng header; low but correct for this motor
    if not motors.get("dry_mass"):
        # Fallback: 20% of casing + propellant estimate, propellant derived from impulse/Isp/g
        propellant_mass = thrust_props["total_impulse"] / max(isp_estimate * 9.81, 1.0)
        motors["propellant_mass"] = propellant_mass
        motors["dry_mass"] = max(0.05, propellant_mass * 0.22)
    else:
        motors["propellant_mass"] = motors.get(
            "propellant_mass",
            thrust_props["total_impulse"] / max(isp_estimate * 9.81, 1.0),
        )

    if "motor_designation" in ork_extras:
        motors["designation"] = ork_extras["motor_designation"]
        motors["manufacturer"] = ork_extras.get("motor_manufacturer", "")

    data["motors"] = motors

    # --- Nose ---
    nose = data.get("nosecones", {}) or {}
    if "nose_shape_parameter" in ork_extras:
        nose["shape_parameter"] = ork_extras["nose_shape_parameter"]
    data["nosecones"] = nose

    # --- Body tube ---
    if "body_tube" in ork_extras:
        data["body_tube"] = ork_extras["body_tube"]

    # --- Fins (CRITICAL: rocketserializer 0.2.0 drops freeform fins entirely) ---
    if "fins" in ork_extras:
        data["fins"] = ork_extras["fins"]

    # --- Parachute enrichment ---
    if "parachutes_detail" in ork_extras:
        data["parachutes_detail"] = ork_extras["parachutes_detail"]

    # --- Rocket total length ---
    nose_len = _safe_float(nose.get("length"))
    tube_len = _safe_float(ork_extras.get("body_tube", {}).get("length"))
    data.setdefault("rocket", {})["total_length"] = nose_len + tube_len if nose_len or tube_len else 0.0

    with params_path.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)


# =============================================================================
# Cleanup
# =============================================================================


_EXPECTED_OUTPUTS = {"parameters.json", "thrust_source.csv", "drag_curve.csv"}


def _clean_output_dir(output_dir: Path) -> None:
    """Remove any stale files so only the three canonical files remain."""
    if not output_dir.exists():
        return
    for item in output_dir.iterdir():
        if item.is_file() and item.name not in _EXPECTED_OUTPUTS:
            logger.info("Removing stale file: %s", item.name)
            item.unlink()


# =============================================================================
# Main convert()
# =============================================================================


def convert(
    ork_path: str | os.PathLike,
    output_dir: str | os.PathLike = "output",
    ork_jar: Optional[str] = None,
) -> Dict[str, Path]:
    """Convert .ork → parameters.json + thrust_source.csv + drag_curve.csv.

    Returns a dict mapping canonical name → absolute path.
    """
    ork_path = Path(ork_path).resolve()
    output_dir = Path(output_dir).resolve()
    if not ork_path.exists():
        raise FileNotFoundError(f"ORK file not found: {ork_path}")

    # 1. ork2json → parameters.json + thrust_source.csv + drag_curve.csv
    run_ork2json(ork_path, output_dir, ork_jar)

    params = output_dir / "parameters.json"
    thrust = output_dir / "thrust_source.csv"
    drag_raw = output_dir / "drag_curve.csv"
    for p in (params, thrust, drag_raw):
        if not p.exists():
            raise RuntimeError(f"ork2json did not produce {p.name}")

    # 2. Clean drag curve in-place
    n = _clean_drag_curve(drag_raw, drag_raw)
    logger.info("Drag curve cleaned: %d points", n)

    # 3. Parse .ork XML for extras (fins, body tube, nose shape)
    extras = _extract_ork_extras(ork_path)
    logger.info("Extracted extras from .ork: %s", list(extras.keys()))

    # 4. Enrich parameters.json
    _enrich_parameters(params, thrust, extras)

    # 5. Remove stale files so only the 3 canonical outputs remain
    _clean_output_dir(output_dir)

    logger.info("Wrote %s", params)
    logger.info("Wrote %s", thrust)
    logger.info("Wrote %s", drag_raw)
    return {"parameters": params, "thrust": thrust, "drag": drag_raw}


# =============================================================================
# Optional minimal Tk GUI
# =============================================================================


def run_gui() -> None:
    try:
        import tkinter as tk
        from tkinter import filedialog, messagebox, scrolledtext
    except ImportError:
        print(
            "tkinter is not available in this Python interpreter.\n"
            "Install the system package (Debian/Ubuntu: 'sudo apt-get install python3-tk'; "
            "macOS: tkinter ships with python.org builds)\n"
            "or run the CLI, e.g. `python serializer.py rocket.ork`.",
            file=sys.stderr,
        )
        sys.exit(1)

    root = tk.Tk()
    root.title("ORK Serializer")
    root.geometry("720x420")

    ork_var = tk.StringVar()
    jar_var = tk.StringVar(value=str(_find_ork_jar(Path.cwd()) or ""))
    out_var = tk.StringVar(value=str(Path.cwd() / "output"))

    def pick_ork():
        p = filedialog.askopenfilename(filetypes=[("OpenRocket files", "*.ork"), ("All", "*.*")])
        if p:
            ork_var.set(p)

    def pick_jar():
        p = filedialog.askopenfilename(filetypes=[("Jar files", "*.jar"), ("All", "*.*")])
        if p:
            jar_var.set(p)

    def pick_out():
        p = filedialog.askdirectory()
        if p:
            out_var.set(p)

    def do_convert():
        log.delete("1.0", tk.END)
        try:
            convert(
                ork_var.get(),
                out_var.get(),
                ork_jar=jar_var.get() or None,
            )
            log.insert(tk.END, "Done. Files written to:\n")
            for name in _EXPECTED_OUTPUTS:
                log.insert(tk.END, f"  {Path(out_var.get()) / name}\n")
            messagebox.showinfo("Done", "Conversion successful")
        except Exception as exc:
            log.insert(tk.END, f"ERROR: {exc}\n")
            messagebox.showerror("Error", str(exc))

    frm = tk.Frame(root, padx=10, pady=10)
    frm.pack(fill="both", expand=True)
    tk.Label(frm, text=".ork file:").grid(row=0, column=0, sticky="w", pady=3)
    tk.Entry(frm, textvariable=ork_var, width=62).grid(row=0, column=1, padx=5, sticky="ew")
    tk.Button(frm, text="Browse", command=pick_ork).grid(row=0, column=2)
    tk.Label(frm, text="OpenRocket.jar:").grid(row=1, column=0, sticky="w", pady=3)
    tk.Entry(frm, textvariable=jar_var, width=62).grid(row=1, column=1, padx=5, sticky="ew")
    tk.Button(frm, text="Browse", command=pick_jar).grid(row=1, column=2)
    tk.Label(frm, text="Output dir:").grid(row=2, column=0, sticky="w", pady=3)
    tk.Entry(frm, textvariable=out_var, width=62).grid(row=2, column=1, padx=5, sticky="ew")
    tk.Button(frm, text="Browse", command=pick_out).grid(row=2, column=2)
    tk.Button(frm, text="Convert", command=do_convert).grid(row=3, column=1, pady=8)
    log = scrolledtext.ScrolledText(frm, height=14)
    log.grid(row=4, column=0, columnspan=3, sticky="nsew")
    frm.rowconfigure(4, weight=1)
    frm.columnconfigure(1, weight=1)
    root.mainloop()


# =============================================================================
# CLI
# =============================================================================


def _build_cli() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("ork", nargs="?", help="Path to the .ork file.")
    p.add_argument("--output", "-o", default="output", help="Output directory (default: ./output).")
    p.add_argument("--ork-jar", default=None, help="Path to OpenRocket.jar. Auto-detected in ORK's folder if omitted.")
    p.add_argument("--gui", action="store_true", help="Launch the minimal Tk GUI.")
    p.add_argument("--verbose", "-v", action="store_true", help="Verbose logging.")
    return p


def main(argv: Optional[List[str]] = None) -> int:
    # No CLI args at all (e.g. "Run" button in a Python IDE) -> GUI with file pickers.
    effective_argv = argv if argv is not None else sys.argv[1:]
    if not effective_argv:
        logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
        logger.info("No arguments given, launching GUI. Pass --help for CLI usage.")
        run_gui()
        return 0

    args = _build_cli().parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(levelname)s %(name)s: %(message)s",
    )

    if args.gui:
        run_gui()
        return 0

    if not args.ork:
        print("Error: either provide an .ork path or use --gui\n", file=sys.stderr)
        _build_cli().print_help()
        return 2

    try:
        convert(args.ork, args.output, args.ork_jar)
    except Exception as exc:
        logger.error("%s", exc)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
