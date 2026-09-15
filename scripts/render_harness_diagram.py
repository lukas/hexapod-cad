#!/usr/bin/env python3
"""Generate + render the full-robot wiring-harness diagram (WireViz).

Emits ``firmware/harness.yml`` — a WireViz description of the COMPLETE
18-servo distributed-power + serial-data harness from
``firmware/WIRING.md`` (§6), with the geometry-derived per-leg branch
lengths taken from ``motor_setup/wire_harness_plan.py`` — then invokes
WireViz to render ``firmware/harness.svg`` / ``harness.png`` (plus the
auto-generated BOM as ``harness.bom.tsv`` and an ``harness.html`` page
that embeds diagram + BOM together).

Topology encoded here (source of truth: firmware/WIRING.md, Aug 2026
as-built — NO PDB, NO external buck):

  * POWER trunk: 3S LiPo -> XT60 -> anti-spark switch -> 15-20 A main
    fuse -> CENTRAL TRUNK WAGO pair (two 5-port Wago 221-415 at the
    chassis_top centre, one for V+, one for GND; 12 AWG silicone).
  * Trunk Wagos fan out six per-leg 16 AWG branches, each landing on a
    CORNER POWER Wago pair (two 3-port 221-413 between the tray walls
    integrated into the chassis_bottom top face at the leg's hex corner
    flat) and then a Molex 5264
    injection pigtail.  As-built the pigtail mates the leg's HIP
    servo's spare port (the hip is the power tee; yaw/knee draw V+
    through the within-leg leads).  Branch lengths come from
    WIRE_HARNESS_PLAN.
  * Within-leg data chain: stock FEETECH 3-pin (V+/GND/SIG) leads,
    yaw -> hip -> knee.  Servo bus IDs are 2..19 (leg 0 = 2/3/4 …
    leg 5 = 17/18/19).
  * Leg-to-leg data jumpers: SIGNAL + GND ONLY (V+ omitted so power
    never bridges legs through a 3 A-rated 5264 pin).
  * Logic: Uno Q VIN takes its OWN battery tap at the LiPo XT60 (the
    on-board regulator handles the 3S rail; no buck).  Uno Q USB-C
    -> OTG hub -> USB bus-servo adapter (FE-URT-2 / Waveshare A,
    level switch 5 V) whose D/G pins drive the servo bus (fallback
    path; the preferred path is the MCU feetech_bridge over D0/D1);
    GY-521 IMU on the Uno Q's own I2C (3V3!).

Run (repo venv):
    ../../run.sh hexapod_walker/prototype_sts3215/scripts/render_harness_diagram.py
or
    .venv/bin/python hexapod_walker/prototype_sts3215/scripts/render_harness_diagram.py

Requires ``wireviz`` (pip, in the repo venv) and Graphviz ``dot``
(``brew install graphviz``).
"""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import yaml

_THIS_DIR = Path(__file__).resolve().parent
_PROTOTYPE_DIR = _THIS_DIR.parent

from wire_harness_plan import (
    STOCK_PIGTAIL_MM,
    WIRE_HARNESS_PLAN,
)
from hexapod_core.joint_frame import N_LEGS, leg_joints, servo_id

FIRMWARE_DIR = _PROTOTYPE_DIR / "firmware"
HARNESS_YML = FIRMWARE_DIR / "harness.yml"

_AXIS_LABEL = {"yaw": "yaw", "hip_pitch": "hip", "knee": "knee"}

# Stock FEETECH 3-pin lead / jumper lengths (m).  All 18 joints fit the
# 300 mm stock lead (see WIRE_HARNESS_PLAN's extension_required column).
STOCK_LEAD_M = round(STOCK_PIGTAIL_MM * 1e-3, 2)


def _plan_by(leg: int, axis: str) -> dict:
    for e in WIRE_HARNESS_PLAN:
        if e["leg_idx"] == leg and e["axis"] == axis:
            return e
    raise KeyError((leg, axis))


def _branch_length_m(leg: int) -> float:
    """Per-leg 16 AWG power-branch length (m), from the geometry-derived
    plan: the branch runs trunk Wago -> corner Wago pair -> drop slot ->
    the leg's injection point (budget value; the plan's YAW entry is the
    conservative full chassis-to-leg run)."""
    return round(_plan_by(leg, "yaw")["path_length_mm_min"] * 1e-3, 2)


def build_harness() -> dict:
    """Return the WireViz document (connectors / cables / connections)."""
    connectors: dict = {}
    cables: dict = {}
    connections: list = []

    # ---------------------------------------------------------------- power
    connectors["LIPO"] = {
        "type": "3S LiPo (11.1 V nom)",
        "subtype": "XT60 female",
        "pinlabels": ["V+", "GND"],
        "notes": "raw 12 V servo rail; no BEC anywhere",
    }
    connectors["SWITCH"] = {
        "type": "Anti-spark switch",
        "subtype": "XT60 in/out",
        "pinlabels": ["IN V+", "IN GND", "OUT V+", "OUT GND"],
        "notes": "precharge + on/off = e-stop",
    }
    connectors["FUSE"] = {
        "type": "Main fuse 15-20 A",
        "subtype": "blade/ANL in holder",
        "pinlabels": ["IN", "OUT"],
        "notes": "sized above ~9-12.6 A walking draw",
    }
    # Central trunk Wago splice pair (as-built Aug 2026: no PDB): two
    # 5-port Wago 221-415 side by side at the chassis_top centre -- one
    # splices the fused V+ trunk into the branch feeds, the other
    # splices GND (+ the data-GND bond).  Wago 221 = 32 A / 4 mm².
    # Pinlabels show the LOGICAL per-leg feeds; each nut has 5 physical
    # ports (1 trunk in + 4 free), so branches share ports as wired.
    connectors["TRUNK_WAGO"] = {
        "type": "Central trunk pair (2x 5-port Wago 221-415)",
        "pinlabels": (
            ["V+ IN", "GND IN"]
            + [f"V+ L{i}" for i in range(N_LEGS)]
            + [f"GND L{i}" for i in range(N_LEGS)] + ["GND DATA"]
        ),
        "notes": "chassis_top centre; logical per-leg feeds shown (5 "
                 "physical ports per nut); optional 5-7 A branch fuses; "
                 "no PDB — all lever nuts",
    }
    connectors["UNOQ"] = {
        "type": "Arduino Uno Q",
        "pinlabels": ["USB-C", "VIN", "GND", "3V3", "SCL", "SDA"],
        "notes": "gait/RL host (Linux side); own battery tap on VIN "
                 "(on-board regulator, no buck); drives bus via MCU "
                 "bridge (preferred) or USB adapter (fallback)",
    }
    connectors["ADAPTER"] = {
        "type": "USB bus-servo adapter",
        "subtype": "FE-URT-2 / Waveshare A",
        "pinlabels": ["USB-C", "D", "G"],
        "notes": "level switch 5 V (FE-URT-2) / mode B (Waveshare); "
                 "data-only on the full robot",
    }
    connectors["IMU"] = {
        "type": "GY-521 (MPU-6050)",
        "pinlabels": ["VCC", "GND", "SCL", "SDA"],
        "notes": ("I2C addr 0x68; POWER FROM 3V3, NEVER 5 V; glued on "
                  "chassis_top beside the central trunk Wagos"),
    }

    # 12 AWG trunk: LiPo -> switch -> fuse -> trunk Wagos.  The fuse sits
    # in the V+ line only; trunk GND runs switch -> GND Wago directly.
    cables["W_TRUNK_LIPO"] = {
        "gauge": "12 AWG", "length": 0.15, "colors": ["RD", "BK"],
        "notes": "silicone XT60 pigtail",
    }
    connections.append([
        {"LIPO": ["V+", "GND"]},
        {"W_TRUNK_LIPO": [1, 2]},
        {"SWITCH": ["IN V+", "IN GND"]},
    ])
    cables["W_TRUNK_VPLUS"] = {
        "gauge": "12 AWG", "length": 0.1, "colors": ["RD"],
        "notes": "switch → main fuse (V+ only)",
    }
    connections.append([
        {"SWITCH": ["OUT V+"]},
        {"W_TRUNK_VPLUS": [1]},
        {"FUSE": ["IN"]},
    ])
    cables["W_TRUNK_FUSED"] = {
        "gauge": "12 AWG", "length": 0.1, "colors": ["RD"],
        "notes": "fused V+ trunk into the V+ trunk Wago",
    }
    connections.append([
        {"FUSE": ["OUT"]},
        {"W_TRUNK_FUSED": [1]},
        {"TRUNK_WAGO": ["V+ IN"]},
    ])
    cables["W_TRUNK_GND"] = {
        "gauge": "12 AWG", "length": 0.2, "colors": ["BK"],
        "notes": "trunk GND, switch → GND trunk Wago",
    }
    connections.append([
        {"SWITCH": ["OUT GND"]},
        {"W_TRUNK_GND": [1]},
        {"TRUNK_WAGO": ["GND IN"]},
    ])

    # ---------------------------------------------------- logic + data spine
    # Uno Q battery tap: straight from the LiPo XT60 (before the servo
    # switch), 20-22 AWG.  No buck — the Uno Q regulates the 3S rail.
    cables["W_UNOQ_TAP"] = {
        "gauge": "20 AWG", "length": 0.25, "colors": ["RD", "BK"],
        "notes": "Uno Q own battery tap (share ground only; no buck)",
    }
    connections.append([
        {"LIPO": ["V+", "GND"]},
        {"W_UNOQ_TAP": [1, 2]},
        {"UNOQ": ["VIN", "GND"]},
    ])
    cables["W_USB"] = {
        "category": "bundle", "length": 0.3, "colors": ["BK"],
        "notes": "USB-C-to-C via OTG hub (Uno Q is USB host)",
    }
    connections.append([
        {"UNOQ": ["USB-C"]},
        {"W_USB": [1]},
        {"ADAPTER": ["USB-C"]},
    ])
    # Adapter D/G -> head of the servo bus (leg 0's yaw servo, ID 2).
    cables["W_BUS_HEAD"] = {
        "gauge": "24 AWG", "length": STOCK_LEAD_M, "colors": ["WH", "BK"],
        "notes": "adapter D/G → bus signal + GND (V+ OMITTED)",
    }
    connections.append([
        {"ADAPTER": ["D", "G"]},
        {"W_BUS_HEAD": [1, 2]},
        {"SERVO2": ["SIG", "GND"]},
    ])
    # Mandatory ground bond: adapter servo-terminal GND <-> trunk GND Wago.
    cables["W_GND_BOND"] = {
        "gauge": "22 AWG", "length": 0.1, "colors": ["BK"],
        "notes": "common-ground bond (bus has no return without it)",
    }
    connections.append([
        {"ADAPTER": ["G"]},
        {"W_GND_BOND": [1]},
        {"TRUNK_WAGO": ["GND DATA"]},
    ])
    cables["W_I2C"] = {
        "gauge": "28 AWG", "length": 0.15,
        "colors": ["RD", "BK", "YE", "GN"],
        "notes": "Uno Q I2C (own bus, separate from servo bus); 3V3!",
    }
    connections.append([
        {"UNOQ": ["3V3", "GND", "SCL", "SDA"]},
        {"W_I2C": [1, 2, 3, 4]},
        {"IMU": ["VCC", "GND", "SCL", "SDA"]},
    ])

    # ------------------------------------------------------------- per leg
    for leg in range(N_LEGS):
        # As-built bus IDs run 2..19: leg 0 = 2/3/4 … leg 5 = 17/18/19
        # (WIRING.md § intro; L5 knee = ID 19).
        ids = [servo_id(j) for j in leg_joints(leg)]  # yaw, hip, knee

        # 18x STS3215 (Molex 5264 3-pin: black GND, red V+, white SIG).
        for sid, axis in zip(ids, ("yaw", "hip_pitch", "knee")):
            entry = _plan_by(leg, axis)
            connectors[f"SERVO{sid}"] = {
                "type": "FEETECH STS3215",
                "subtype": "Molex 5264 3-pin",
                "pinlabels": ["GND", "V+", "SIG"],
                "notes": (f"L{leg} {_AXIS_LABEL[axis]} — joint "
                          f"{entry['joint_idx']}, bus ID {sid}; lead budget "
                          f"{entry['path_length_mm_min']:.0f} mm "
                          f"({entry['extension_required']})"),
            }

        # Corner power Wago pair in the tray walls integrated into the
        # chassis_bottom top face at this leg's hex corner flat
        # (az = leg*60 deg).
        connectors[f"L{leg}_WAGO"] = {
            "type": "Power Wago pair (2x Wago 221-413)",
            "pinlabels": ["V+", "GND"],
            "notes": f"leg {leg} 12 V+G 3-port lever nuts in the integrated "
                     "chassis corner tray, entries facing inward",
        }
        # Power injection pigtail — as-built it mates the leg's HIP
        # servo's spare port (the hip is the power tee; yaw + knee take
        # V+ through the within-leg leads).
        connectors[f"L{leg}_INJ"] = {
            "type": "Molex 5264 pigtail",
            "subtype": "crimped 3-pin",
            "pinlabels": ["GND", "V+", "SIG"],
            "notes": f"leg {leg} V+/GND injection at the HIP "
                     f"(mates SERVO{ids[1]}'s spare port; ~3 A/pin "
                     "limit = one leg max)",
        }
        cables[f"W_PWR_L{leg}"] = {
            "gauge": "16 AWG", "length": 0.1, "colors": ["RD", "BK"],
            "notes": f"trunk Wagos → leg {leg} corner power Wago pair",
        }
        connections.append([
            {"TRUNK_WAGO": [f"V+ L{leg}", f"GND L{leg}"]},
            {f"W_PWR_L{leg}": [1, 2]},
            {f"L{leg}_WAGO": ["V+", "GND"]},
        ])
        cables[f"W_BR_L{leg}"] = {
            "gauge": "16 AWG",
            "length": _branch_length_m(leg),
            "colors": ["RD", "BK"],
            "notes": f"leg {leg} power branch (length from "
                     "wire_harness_plan geometry)",
        }
        connections.append([
            {f"L{leg}_WAGO": ["V+", "GND"]},
            {f"W_BR_L{leg}": [1, 2]},
            {f"L{leg}_INJ": ["V+", "GND"]},
        ])
        # The pigtail's mate onto the hip servo's spare 5264 port
        # (WireViz "direct connection" arrow, no cable in between).
        # NB: mate arrows need NUMERIC pins in WireViz 0.4.1 — its mate
        # renderer indexes connector.pins (ints), not pinlabels.
        connections.append([
            {f"L{leg}_INJ": [1, 2, 3]},
            "-->",
            {f"SERVO{ids[1]}": [1, 2, 3]},
        ])

        # Within-leg stock 3-pin chain: yaw -> hip -> knee.
        for a, b, tag in ((ids[0], ids[1], "A"), (ids[1], ids[2], "B")):
            cname = f"W_CHAIN_L{leg}{tag}"
            cables[cname] = {
                "gauge": "22 AWG", "length": STOCK_LEAD_M,
                "colors": ["BK", "RD", "WH"],
                "notes": "stock FEETECH 3-pin bus lead (within-leg only)",
            }
            connections.append([
                {f"SERVO{a}": ["GND", "V+", "SIG"]},
                {cname: [1, 2, 3]},
                {f"SERVO{b}": ["GND", "V+", "SIG"]},
            ])

        # Leg-to-leg data jumper: SIGNAL + GND only, V+ OMITTED.
        if leg < N_LEGS - 1:
            cname = f"W_DATA_L{leg}L{leg + 1}"
            cables[cname] = {
                "gauge": "24 AWG", "length": STOCK_LEAD_M,
                "colors": ["WH", "BK"],
                "notes": "leg-to-leg DATA jumper — V+ OMITTED "
                         "(power never bridges legs)",
            }
            connections.append([
                {f"SERVO{ids[2]}": ["SIG", "GND"]},
                {cname: [1, 2]},
                {f"SERVO{ids[2] + 1}": ["SIG", "GND"]},
            ])

    return {
        "metadata": {
            "title": "Hexapod STS3215 — distributed-power + serial-bus harness",
            "description": (
                "Generated by scripts/render_harness_diagram.py from "
                "firmware/WIRING.md (topology) + "
                "motor_setup/wire_harness_plan.py (per-leg branch lengths). "
                "DO NOT EDIT harness.yml BY HAND."
            ),
        },
        "options": {"fontname": "Helvetica"},
        "connectors": connectors,
        "cables": cables,
        "connections": connections,
    }


def render(yml_path: Path) -> None:
    """Invoke WireViz on ``yml_path`` (html + png + svg + BOM tsv)."""
    wireviz = Path(sys.executable).parent / "wireviz"
    if not wireviz.exists():
        raise SystemExit(
            f"wireviz not found at {wireviz}; pip install wireviz into the "
            "repo venv first")
    env = os.environ.copy()
    # Graphviz `dot` (brew) may not be on a minimal PATH.
    for extra in ("/opt/homebrew/bin", "/usr/local/bin"):
        if extra not in env.get("PATH", ""):
            env["PATH"] = env.get("PATH", "") + os.pathsep + extra
    # h = html (diagram + BOM), p = png, s = svg, t = BOM tsv.
    subprocess.run(
        [str(wireviz), str(yml_path), "-f", "hpst"],
        check=True, env=env, cwd=str(yml_path.parent))


def main() -> None:
    doc = build_harness()
    header = (
        "# GENERATED by scripts/render_harness_diagram.py — do not edit.\n"
        "# Topology: firmware/WIRING.md; branch lengths: "
        "motor_setup/wire_harness_plan.py.\n"
    )
    HARNESS_YML.write_text(
        header + yaml.dump(doc, sort_keys=False, allow_unicode=True,
                           width=78))
    print(f"Wrote {HARNESS_YML}")
    render(HARNESS_YML)
    for suffix in (".svg", ".png", ".html"):
        out = HARNESS_YML.with_suffix(suffix)
        if out.exists():
            print(f"Rendered {out}")


if __name__ == "__main__":
    main()
