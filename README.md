# simulation-rocket

Two-stage pipeline that turns an OpenRocket (`.ork`) design into a realistic
RocketPy Monte-Carlo flight simulation, **plus** a Python Bridge that generates
ANSYS-ready STEP files from the same `.ork` geometry.

```
                                        ┌──► step_output/nose_cone.step
rocket.ork ──► ork_to_step.py ──────────┤    step_output/body_tube.step
         │                              │    step_output/freeform_fin_0.step
         │                              └──► step_output/rocket_assembly.step
         │                                          │
         │                                          ▼
         │                                   ANSYS Mechanical import
         │
         └──► serializer.py ──► output/parameters.json
                                output/thrust_source.csv
                                output/drag_curve.csv
                                       │
                                       ▼
                             simu/simu_bot.py ──► simu_output/results.csv
                                                  simu_output/summary.csv
                                                  simu_output/plots/
```

## Requirements

```bash
# Simulation pipeline
pip install rocketpy rocketserializer pandas

# Python Bridge (ork_to_step.py)
pip install -r requirements-cad.txt   # or: pip install cadquery
```

`rocketserializer` provides the `ork2json` CLI used by `serializer.py`. It
needs the bundled `OpenRocket.jar` (already in this repo) and Java 17+.

On Debian/Ubuntu also install Tk so the GUI file pickers work:

```bash
sudo apt-get install python3-tk
```

## Running from an IDE

Both scripts **launch a GUI with file pickers when run without CLI arguments**,
so in PyCharm / VS Code / Spyder you can:

1. Open the repo in your IDE.
2. Right-click `serializer.py` → *Run* → pick the `.ork`, `OpenRocket.jar`
   and output directory.
3. Right-click `simu/simu_bot.py` → *Run* → pick `parameters.json`,
   `thrust_source.csv`, `drag_curve.csv` and the output directory.

No command-line arguments needed. Passing any flag (e.g. `--help`) switches
to CLI mode.

## 1. Convert .ork → serializer output

```bash
# CLI
python serializer.py rocket.ork --output output --ork-jar OpenRocket.jar

# GUI with file pickers for .ork, OpenRocket.jar, and output dir
python serializer.py --gui
```

Produces exactly three files in `output/`:

| File | Contents |
|---|---|
| `parameters.json` | rocket, motor, environment, flight parameters (enriched with fin geometry, body-tube length, nose shape, motor-performance derived from thrust curve) |
| `thrust_source.csv` | time [s], thrust [N] |
| `drag_curve.csv` | mach, Cd (de-duplicated and cleaned) |

The serializer also fixes two things `ork2json` alone does not:

- Extracts **freeform-fin geometry** directly from the `.ork` XML
  (`rocketserializer 0.2.0` only handles trapezoidal/elliptical fins).
- Fills in `dry_mass`, `propellant_mass`, `burn_time`, `total_impulse`,
  `max_thrust` in `parameters.json` even when OpenRocket left them as 0.

## 2. Run the simulation

```bash
# deterministic single run, picking a directory
python simu/simu_bot.py --input output --iterations 1

# full Monte Carlo with explicit per-file selection
python simu/simu_bot.py \
    --params output/parameters.json \
    --thrust output/thrust_source.csv \
    --drag   output/drag_curve.csv \
    --output simu_output --iterations 200

# GUI with a browse button for each file
python simu/simu_bot.py --gui
```

Flags:

- `--input` / `-i`  — directory with the three serializer outputs (default `output`). Overridden per-file by:
- `--params` — path to `parameters.json`
- `--thrust` — path to `thrust_source.csv`
- `--drag`   — path to `drag_curve.csv`
- `--output` / `-o` — directory to write results + plots (default `simu_output`)
- `--iterations` / `-n` — number of MC iterations (default 100)
- `--seed` — PRNG seed (default 42)
- `--no-plots` — skip PNG generation
- `--gui` — launch a minimal Tk GUI instead

Outputs:

- `results.csv` — one row per iteration (apogee, max_v, impact, inputs)
- `summary.csv` — `DataFrame.describe()` over successful iterations
- `plots/apogee.png`, `plots/max_velocity.png`, `plots/impact_scatter.png`,
  `plots/sensitivity.png`

## 3. Generate STEP files for ANSYS (Python Bridge)

`ork_to_step.py` reads the `.ork` XML directly (supports both plain-XML and
ZIP-packed `.ork` files), extracts the geometry of every major component, builds
parametric 3-D solids with [CadQuery](https://cadquery.readthedocs.io/), and
exports them as `.step` files.

```bash
# CLI -- auto-detects .ork in the current directory
python ork_to_step.py rocket.ork

# Specify output directory and enable verbose logging
python ork_to_step.py rocket.ork --output step_output -v

# Optionally supplement geometry from serializer's parameters.json
python ork_to_step.py rocket.ork --params output/parameters.json

# GUI file picker
python ork_to_step.py --gui
```

**Generated STEP files** (one per component + full assembly):

| File | Contents |
|---|---|
| `nose_cone.step` | Solid-of-revolution nose cone (Haack, ogive, conical, parabolic, power series) |
| `body_tube.step` | Hollow cylinder body tube |
| `freeform_fin_N.step` | Freeform fins (circular-patterned) |
| `trap_fin_N.step` | Trapezoidal fins (if present) |
| `transition_N.step` | Transitions / boat-tails (if present) |
| `rocket_assembly.step` | All parts combined in a single STEP assembly |

**Supported nose-cone shapes:** Haack (including LV-Haack / Von Karman),
ogive, conical, parabolic, power series.

**ANSYS import:**
Open ANSYS Workbench → *Geometry* → *Import* → *External Geometry File* →
select `rocket_assembly.step` (or individual part files).

## What was broken before

- The old `simu/monte_carlo_gui.py` **hard-coded** the rocket geometry (mass,
  radius, nose, fins, motor position) and only read wind / launch conditions
  from JSON. Any rocket whose actual mass differed from 14.426 kg would
  essentially not fly — for a 0.31 kg design with a 54 N peak thrust motor,
  T/W was 0.4 so the rocket stayed on the rail and apogee was reported as 0.
- The old `serializer.py` produced `motor_thrust.csv` (duplicate of
  `thrust_source.csv`) and `rocketpy_generated.py` (never consumed) on top of
  the three canonical files, and synthesized fake drag / thrust curves when
  it failed to parse the real ones.
- The `eng to csv/` folder contained a stray copy of the old simulator.

This version throws those away, reads everything from `parameters.json`, and
produces non-zero, physically-plausible apogee values that track the
OpenRocket stored result (within ~20%).

## Reference data

- `rocket.ork` — primary rocket design used for testing
- `roc3ket.ork` — alternate design
- `KAYRA-38.eng`, `KAYRA-38.csv` — reference motor file and its CSV conversion
