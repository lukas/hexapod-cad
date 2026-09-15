#!/usr/bin/env python3
"""Extract stand/walk/rise/lower structural loads from the MuJoCo twin.

MuJoCo is good at answering "what forces and torques did the robot see while
standing or walking?"  FEA is good at answering "where do those loads stress
the CAD?"  This script is the bridge: it records foot contact forces and
actuator torques, reduces them into conservative peak/percentile load cases,
and writes a JSON artifact for the strength/Onshape workflow.

Run from ``hexapod_walker/prototype_sts3215`` or the repo root::

    uv run --with mujoco --with numpy \
      python strength/extract_mujoco_loads.py --mode all
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
from typing import Callable

import numpy as np

THIS_DIR = Path(__file__).resolve().parent
PROTO_DIR = THIS_DIR.parent
ROOT_DIR = PROTO_DIR.parents[1]


DEG2RAD = math.pi / 180.0
COUNTS_PER_DEG = 4096.0 / 360.0
DEFAULT_OUT = PROTO_DIR / "artifacts" / "strength" / "mujoco_loads.json"
DEFAULT_RISE_REF = (
    PROTO_DIR / "rl_move" / "sim" / "refs" / "rise_ref_belly2plant.npz"
)
AXES = ("yaw", "hip", "knee")


def _round_list(values, digits: int = 6) -> list[float]:
    return [round(float(v), digits) for v in values]


def _pct(values: np.ndarray, q: float) -> float:
    if values.size == 0:
        return 0.0
    return float(np.percentile(values, q))


def _top_indices(values: np.ndarray, count: int) -> list[int]:
    if values.size == 0 or count <= 0:
        return []
    count = min(int(count), int(values.size))
    return [int(i) for i in np.argsort(values)[::-1][:count]]


def _rel(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT_DIR))
    except ValueError:
        return str(path)


class MujocoLoadProbe:
    """Small MuJoCo rig that records foot loads and joint torques."""

    def __init__(self, *, params_name: str, model_source: str):
        import mujoco
        from rl_move.sim.servo_model import (
            LOADED_MODEL_PATH,
            SimServoParams,
            apply_params_to_model,
            build_model,
            joint_names,
            joint_qpos_addrs,
            joint_qvel_addrs,
            position_actuator_ids,
        )
        from rl_move.sim.sim_env import soften_contacts

        self.mujoco = mujoco
        if params_name == "loaded":
            self.params = SimServoParams.load(LOADED_MODEL_PATH)
        elif params_name == "air":
            self.params = SimServoParams.load()
        else:
            self.params = SimServoParams.load(Path(params_name))

        self.model = build_model(
            fixed_base=False,
            flat_terrain=True,
            mesh_visuals=False,
            source=model_source,
        )
        apply_params_to_model(self.model, self.params)
        soften_contacts(self.model)

        self.data = mujoco.MjData(self.model)
        self.qadr = joint_qpos_addrs(self.model)
        self.vadr = joint_qvel_addrs(self.model)
        self.pos_act = position_actuator_ids(self.model)
        self.joint_names = joint_names()
        self.dt = float(self.model.opt.timestep)

        self.foot_gid = [
            mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_GEOM, f"L{i}_foot"
            )
            for i in range(6)
        ]
        missing = [i for i, gid in enumerate(self.foot_gid) if gid < 0]
        if missing:
            raise RuntimeError(f"missing MuJoCo foot geoms for legs {missing}")

    def place_at_pose(self, q_rad: np.ndarray) -> None:
        """Place the free-base robot with feet just touching the flat ground."""
        import mujoco_prototype as mp
        from rl_move.body_ik import fk_all_feet

        feet = fk_all_feet(q_rad)
        base_z = mp.YAW_OUTPUT_HEIGHT - float(np.min(feet[:, 2])) + mp.FOOT_R + 0.002
        self.mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:3] = (0.0, 0.0, base_z)
        self.data.qpos[3:7] = (1.0, 0.0, 0.0, 0.0)
        self.data.qpos[self.qadr] = q_rad
        self.data.ctrl[self.pos_act] = q_rad
        self.mujoco.mj_forward(self.model, self.data)

        for _ in range(40):
            worst = min(
                (float(self.data.contact[ci].dist) for ci in range(self.data.ncon)),
                default=0.0,
            )
            if worst > -1.0e-4:
                break
            self.data.qpos[2] += -worst + 0.001
            self.mujoco.mj_forward(self.model, self.data)

    def _foot_contact_loads(self) -> list[dict]:
        """Return aggregated contact data for each foot.

        ``force_world_N`` is normalized so +Z is upward when a vertical contact
        force exists. This is the useful ground-on-foot convention for load-case
        extraction; ``normal_N`` remains the sign-free MuJoCo normal magnitude.
        """
        buf = np.zeros(6)
        loads = [
            {
                "normal_N": 0.0,
                "force_world_N": np.zeros(3),
                "point_weighted_world_m": np.zeros(3),
                "contact_count": 0,
                "other_geoms": set(),
            }
            for _ in range(6)
        ]

        for ci in range(self.data.ncon):
            con = self.data.contact[ci]
            for leg, gid in enumerate(self.foot_gid):
                if gid not in (con.geom1, con.geom2):
                    continue
                self.mujoco.mj_contactForce(self.model, self.data, ci, buf)
                normal = abs(float(buf[0]))
                frame = np.asarray(con.frame, dtype=float).reshape(3, 3)
                force_world = frame.T @ np.asarray(buf[:3], dtype=float)
                if force_world[2] < 0.0:
                    force_world *= -1.0
                loads[leg]["normal_N"] += normal
                loads[leg]["force_world_N"] += force_world
                loads[leg]["point_weighted_world_m"] += normal * np.asarray(
                    con.pos, dtype=float
                )
                loads[leg]["contact_count"] += 1
                other_gid = con.geom2 if con.geom1 == gid else con.geom1
                other = self.mujoco.mj_id2name(
                    self.model, self.mujoco.mjtObj.mjOBJ_GEOM, other_gid
                )
                if other:
                    loads[leg]["other_geoms"].add(other)

        out = []
        for leg, row in enumerate(loads):
            normal = float(row["normal_N"])
            if normal > 0.0:
                point = row["point_weighted_world_m"] / normal
            else:
                point = np.asarray(self.data.geom_xpos[self.foot_gid[leg]], dtype=float)
            force = row["force_world_N"]
            out.append(
                {
                    "leg": leg,
                    "normal_N": normal,
                    "vertical_N": float(force[2]),
                    "horizontal_N": float(np.linalg.norm(force[:2])),
                    "force_world_N": force,
                    "point_world_m": point,
                    "contact_count": int(row["contact_count"]),
                    "other_geoms": sorted(row["other_geoms"]),
                }
            )
        return out

    def sample(self, t_s: float) -> dict:
        foot = self._foot_contact_loads()
        tau = self.data.qfrc_actuator[self.vadr].copy()
        return {
            "t_s": float(t_s),
            "foot_normal_N": np.asarray([f["normal_N"] for f in foot], dtype=float),
            "foot_vertical_N": np.asarray([f["vertical_N"] for f in foot], dtype=float),
            "foot_horizontal_N": np.asarray([f["horizontal_N"] for f in foot], dtype=float),
            "foot_force_world_N": np.asarray([f["force_world_N"] for f in foot], dtype=float),
            "foot_point_world_m": np.asarray([f["point_world_m"] for f in foot], dtype=float),
            "foot_contact_count": np.asarray([f["contact_count"] for f in foot], dtype=int),
            "foot_other_geoms": [f["other_geoms"] for f in foot],
            "joint_torque_Nm": tau,
            "joint_torque_abs_Nm": np.abs(tau),
            "base_xyz_m": self.data.qpos[:3].copy(),
        }

    def run_profiled(
        self,
        *,
        name: str,
        q0_rad: np.ndarray,
        command_fn: Callable[[float], np.ndarray],
        settle_s: float,
        record_s: float,
        command_dt: float,
        speed_deg_s: float,
        acc_units: float,
    ) -> dict:
        from rl_move.sim.servo_model import ServoProfile
        from hexapod_core.joint_frame import robot_abs_rad_to_mujoco_rel_rad

        q0_model = robot_abs_rad_to_mujoco_rel_rad(q0_rad)
        self.place_at_pose(q0_model)
        for _ in range(int(round(settle_s / self.dt))):
            self.data.ctrl[self.pos_act] = q0_model
            self.mujoco.mj_step(self.model, self.data)

        profile = ServoProfile(self.params, self.data.qpos[self.qadr].copy())
        per_command = max(1, int(round(command_dt / self.dt)))
        total_ticks = max(1, int(round(record_s / command_dt)))
        samples = []
        t = 0.0
        for tick in range(total_ticks):
            cmd = robot_abs_rad_to_mujoco_rel_rad(
                np.asarray(command_fn(t), dtype=float))
            profile.command(cmd, speed_deg_s=speed_deg_s, acc_units=acc_units)
            for _ in range(per_command):
                self.data.ctrl[self.pos_act] = profile.tick(self.dt)
                self.mujoco.mj_step(self.model, self.data)
            samples.append(self.sample(t))
            t = (tick + 1) * command_dt
        return summarize_run(name, samples, self.joint_names)


def summarize_run(name: str, samples: list[dict], joint_names: list[str]) -> dict:
    foot_normal = np.asarray([s["foot_normal_N"] for s in samples], dtype=float)
    foot_vertical = np.asarray([s["foot_vertical_N"] for s in samples], dtype=float)
    foot_horizontal = np.asarray([s["foot_horizontal_N"] for s in samples], dtype=float)
    foot_force = np.asarray([s["foot_force_world_N"] for s in samples], dtype=float)
    foot_point = np.asarray([s["foot_point_world_m"] for s in samples], dtype=float)
    foot_contacts = np.asarray([s["foot_contact_count"] for s in samples], dtype=int)
    tau_abs = np.asarray([s["joint_torque_abs_Nm"] for s in samples], dtype=float)
    tau_signed = np.asarray([s["joint_torque_Nm"] for s in samples], dtype=float)

    foot_rows = []
    for leg in range(6):
        imax = int(np.argmax(foot_normal[:, leg])) if len(samples) else 0
        foot_rows.append(
            {
                "leg": leg,
                "normal_N": {
                    "mean": round(float(np.mean(foot_normal[:, leg])), 4),
                    "p95": round(_pct(foot_normal[:, leg], 95), 4),
                    "p99": round(_pct(foot_normal[:, leg], 99), 4),
                    "max": round(float(foot_normal[imax, leg]), 4),
                },
                "vertical_N": {
                    "p99": round(_pct(foot_vertical[:, leg], 99), 4),
                    "max": round(float(np.max(foot_vertical[:, leg])), 4),
                },
                "horizontal_N": {
                    "p99": round(_pct(foot_horizontal[:, leg], 99), 4),
                    "max": round(float(np.max(foot_horizontal[:, leg])), 4),
                },
                "contact_duty": round(float(np.mean(foot_contacts[:, leg] > 0)), 4),
                "max_event": {
                    "t_s": round(float(samples[imax]["t_s"]), 4),
                    "force_world_N": _round_list(foot_force[imax, leg], 4),
                    "point_world_m": _round_list(foot_point[imax, leg], 6),
                    "other_geoms": samples[imax]["foot_other_geoms"][leg],
                },
            }
        )

    joint_rows = []
    for j, jname in enumerate(joint_names):
        imax = int(np.argmax(tau_abs[:, j])) if len(samples) else 0
        joint_rows.append(
            {
                "joint": jname,
                "leg": int(j // 3),
                "axis": AXES[j % 3],
                "abs_torque_Nm": {
                    "mean": round(float(np.mean(tau_abs[:, j])), 5),
                    "p95": round(_pct(tau_abs[:, j], 95), 5),
                    "p99": round(_pct(tau_abs[:, j], 99), 5),
                    "max": round(float(tau_abs[imax, j]), 5),
                },
                "max_event": {
                    "t_s": round(float(samples[imax]["t_s"]), 4),
                    "signed_torque_Nm": round(float(tau_signed[imax, j]), 5),
                },
            }
        )

    max_foot_per_sample = foot_normal.max(axis=1)
    total_foot_per_sample = foot_normal.sum(axis=1)
    max_tau_per_sample = tau_abs.max(axis=1)

    def frame_event(idx: int, selector: str) -> dict:
        idx = int(idx)
        joints = []
        for j in _top_indices(tau_abs[idx], 6):
            joints.append(
                {
                    "joint": joint_names[j],
                    "leg": int(j // 3),
                    "axis": AXES[j % 3],
                    "abs_torque_Nm": round(float(tau_abs[idx, j]), 5),
                    "signed_torque_Nm": round(float(tau_signed[idx, j]), 5),
                }
            )
        return {
            "selector": selector,
            "t_s": round(float(samples[idx]["t_s"]), 4),
            "max_single_foot_normal_N": round(float(max_foot_per_sample[idx]), 4),
            "total_foot_normal_N": round(float(total_foot_per_sample[idx]), 4),
            "max_joint_abs_torque_Nm": round(float(max_tau_per_sample[idx]), 5),
            "base_xyz_m": _round_list(samples[idx]["base_xyz_m"], 6),
            "feet": [
                {
                    "leg": leg,
                    "normal_N": round(float(foot_normal[idx, leg]), 4),
                    "vertical_N": round(float(foot_vertical[idx, leg]), 4),
                    "horizontal_N": round(float(foot_horizontal[idx, leg]), 4),
                    "force_world_N": _round_list(foot_force[idx, leg], 4),
                    "point_world_m": _round_list(foot_point[idx, leg], 6),
                    "contact_count": int(foot_contacts[idx, leg]),
                    "other_geoms": samples[idx]["foot_other_geoms"][leg],
                }
                for leg in range(6)
            ],
            "top_joints": joints,
        }

    frame_indices: list[tuple[int, str]] = [
        (int(np.argmax(max_foot_per_sample)), "max_single_foot_normal"),
        (int(np.argmax(total_foot_per_sample)), "max_total_foot_normal"),
        (int(np.argmax(max_tau_per_sample)), "max_joint_abs_torque"),
    ]
    top_frame_events = []
    seen_frames: set[int] = set()
    for idx, selector in frame_indices:
        if idx in seen_frames:
            continue
        seen_frames.add(idx)
        top_frame_events.append(frame_event(idx, selector))

    top_foot_events = []
    for idx in _top_indices(max_foot_per_sample, 8):
        leg = int(np.argmax(foot_normal[idx]))
        top_foot_events.append(
            {
                "t_s": round(float(samples[idx]["t_s"]), 4),
                "leg": leg,
                "normal_N": round(float(foot_normal[idx, leg]), 4),
                "force_world_N": _round_list(foot_force[idx, leg], 4),
                "point_world_m": _round_list(foot_point[idx, leg], 6),
            }
        )

    top_joint_events = []
    for idx in _top_indices(max_tau_per_sample, 8):
        joint = int(np.argmax(tau_abs[idx]))
        top_joint_events.append(
            {
                "t_s": round(float(samples[idx]["t_s"]), 4),
                "joint": joint_names[joint],
                "leg": int(joint // 3),
                "axis": AXES[joint % 3],
                "abs_torque_Nm": round(float(tau_abs[idx, joint]), 5),
                "signed_torque_Nm": round(float(tau_signed[idx, joint]), 5),
            }
        )

    return {
        "name": name,
        "sample_count": len(samples),
        "summary": {
            "max_single_foot_normal_N": round(float(max_foot_per_sample.max()), 4),
            "p99_single_foot_normal_N": round(_pct(max_foot_per_sample, 99), 4),
            "max_total_foot_normal_N": round(float(total_foot_per_sample.max()), 4),
            "max_joint_abs_torque_Nm": round(float(max_tau_per_sample.max()), 5),
            "p99_joint_abs_torque_Nm": round(_pct(max_tau_per_sample, 99), 5),
        },
        "feet": foot_rows,
        "joints": joint_rows,
        "top_frame_events": top_frame_events,
        "top_foot_events": top_foot_events,
        "top_joint_events": top_joint_events,
        "onshape_mapping_notes": [
            "Use foot max/p99 force vectors as external loads at the boot or tube interface.",
            "Use joint max/p99 torques to derive bearing loads at yaw, hip, and knee cylinders.",
            "Use normal_N for conservative vertical foot reaction; horizontal_N captures scrub/friction load.",
        ],
    }


def _default_plant_rad() -> np.ndarray:
    from rl_move.sim.sim_env import _default_plant_deg

    return np.asarray(_default_plant_deg(), dtype=float) * DEG2RAD


def run_stand(probe: MujocoLoadProbe, args: argparse.Namespace) -> dict:
    q0 = _default_plant_rad()
    return probe.run_profiled(
        name="stand_settled",
        q0_rad=q0,
        command_fn=lambda _t: q0,
        settle_s=args.settle_s,
        record_s=args.stand_s,
        command_dt=args.sample_dt,
        speed_deg_s=args.write_speed_counts / COUNTS_PER_DEG,
        acc_units=args.write_acc,
    )


def run_scripted_walk(probe: MujocoLoadProbe, args: argparse.Namespace) -> dict:
    from hexapod_core.tripod_gait import TripodGait

    q0 = _default_plant_rad()
    gait = TripodGait(vx=args.walk_speed_m_s)
    gait.sync_plant_stance(20.0, 80.0)
    gait.set_lift_mm(args.walk_lift_mm)
    gait.reset_phase()

    return probe.run_profiled(
        name=f"scripted_tripod_walk_{args.walk_speed_m_s:.3f}mps",
        q0_rad=q0,
        command_fn=lambda t: np.asarray(gait.desired_deg(t)) * DEG2RAD,
        settle_s=args.settle_s,
        record_s=args.walk_s,
        command_dt=args.sample_dt,
        speed_deg_s=args.write_speed_counts / COUNTS_PER_DEG,
        acc_units=args.write_acc,
    )


def _reference_q_rad(path: Path, *, reverse: bool) -> tuple[np.ndarray, float]:
    from hexapod_core.joint_frame import (
        FRAME_ROBOT_ABS, JOINT_CONTRACT, require_robot_abs_joint_frame,
    )
    data = np.load(path)
    if "q_rad" not in data:
        raise RuntimeError(f"{path} does not contain q_rad")
    require_robot_abs_joint_frame(
        {"joint_frame": (str(data["joint_frame"])
                         if "joint_frame" in data else None),
         "joint_contract": (str(data["joint_contract"])
                            if "joint_contract" in data else None)},
        source=str(path),
    )
    q_rad = np.asarray(data["q_rad"], dtype=float)
    if q_rad.ndim != 2 or q_rad.shape[1] != 18:
        raise RuntimeError(f"{path} q_rad has shape {q_rad.shape}; expected (T, 18)")
    dt = float(data["dt"]) if "dt" in data else 0.04
    if reverse:
        q_rad = q_rad[::-1].copy()
    return q_rad, dt


def run_reference_motion(
    probe: MujocoLoadProbe,
    args: argparse.Namespace,
    *,
    name: str,
    ref_path: Path,
    reverse: bool,
    record_s: float,
) -> dict:
    q_ref, ref_dt = _reference_q_rad(ref_path, reverse=reverse)
    duration_s = max(ref_dt, (len(q_ref) - 1) * ref_dt)
    if record_s <= 0.0:
        record_s = duration_s

    def command_fn(t: float) -> np.ndarray:
        idx = int(round(t / ref_dt))
        idx = max(0, min(idx, len(q_ref) - 1))
        return q_ref[idx]

    return probe.run_profiled(
        name=name,
        q0_rad=q_ref[0],
        command_fn=command_fn,
        settle_s=args.settle_s,
        record_s=record_s,
        command_dt=args.sample_dt,
        speed_deg_s=args.write_speed_counts / COUNTS_PER_DEG,
        acc_units=args.write_acc,
    )


def run_rise(probe: MujocoLoadProbe, args: argparse.Namespace) -> dict:
    ref_path = args.rise_ref.resolve()
    return run_reference_motion(
        probe,
        args,
        name=f"rise_ref_{ref_path.stem}",
        ref_path=ref_path,
        reverse=False,
        record_s=args.rise_s,
    )


def run_lower(probe: MujocoLoadProbe, args: argparse.Namespace) -> dict:
    ref_path = args.rise_ref.resolve()
    return run_reference_motion(
        probe,
        args,
        name=f"lower_ref_{ref_path.stem}",
        ref_path=ref_path,
        reverse=True,
        record_s=args.lower_s,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mode",
        choices=("stand", "walk", "rise", "lower", "both", "all"),
        default="both",
    )
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument(
        "--params",
        default="loaded",
        help="'loaded', 'air', or a SimServoParams JSON path (default: loaded).",
    )
    parser.add_argument(
        "--model-source",
        choices=("mesh", "mesh_mjx", "primitive"),
        default=os.environ.get("HEXAPOD_MODEL_SOURCE", "mesh_mjx"),
        help="MuJoCo model family. mesh_mjx is checked in and portable.",
    )
    parser.add_argument("--settle-s", type=float, default=1.5)
    parser.add_argument("--stand-s", type=float, default=2.0)
    parser.add_argument("--walk-s", type=float, default=8.0)
    parser.add_argument(
        "--rise-s",
        type=float,
        default=0.0,
        help="Reference replay duration; <=0 uses the whole rise reference.",
    )
    parser.add_argument(
        "--lower-s",
        type=float,
        default=0.0,
        help="Reference replay duration; <=0 uses the whole reversed rise reference.",
    )
    parser.add_argument(
        "--rise-ref",
        type=Path,
        default=DEFAULT_RISE_REF,
        help="NPZ with q_rad/dt used for rise and reversed lowering.",
    )
    parser.add_argument("--sample-dt", type=float, default=0.04)
    parser.add_argument("--walk-speed-m-s", type=float, default=0.03)
    parser.add_argument("--walk-lift-mm", type=float, default=25.0)
    parser.add_argument("--write-speed-counts", type=float, default=1500.0)
    parser.add_argument("--write-acc", type=float, default=30.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    probe = MujocoLoadProbe(params_name=args.params, model_source=args.model_source)
    runs = []
    if args.mode in ("stand", "both", "all"):
        print("[mujoco-loads] running settled stand")
        runs.append(run_stand(probe, args))
    if args.mode in ("walk", "both", "all"):
        print("[mujoco-loads] running scripted tripod walk")
        runs.append(run_scripted_walk(probe, args))
    if args.mode in ("rise", "all"):
        print(f"[mujoco-loads] running rise reference {args.rise_ref}")
        runs.append(run_rise(probe, args))
    if args.mode in ("lower", "all"):
        print(f"[mujoco-loads] running lowering reference {args.rise_ref}")
        runs.append(run_lower(probe, args))

    artifact = {
        "schema": "hexapod_mujoco_structural_loads_v1",
        "units": {
            "force": "N",
            "torque": "N*m",
            "position": "m",
            "time": "s",
        },
        "source": {
            "script": _rel(Path(__file__).resolve()),
            "model_source": args.model_source,
            "servo_params": args.params,
            "servo_params_source": probe.params.source,
            "sample_dt_s": args.sample_dt,
            "write_speed_counts": args.write_speed_counts,
            "write_acc": args.write_acc,
            "rise_ref": _rel(args.rise_ref.resolve()),
        },
        "runs": runs,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(artifact, indent=2) + "\n")
    print(f"[mujoco-loads] wrote {_rel(args.out.resolve())}")
    for run in runs:
        s = run["summary"]
        print(
            f"[mujoco-loads] {run['name']}: "
            f"max foot {s['max_single_foot_normal_N']:.1f} N, "
            f"p99 foot {s['p99_single_foot_normal_N']:.1f} N, "
            f"max joint {s['max_joint_abs_torque_Nm']:.3f} N*m"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
