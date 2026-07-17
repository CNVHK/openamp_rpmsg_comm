#!/usr/bin/env python3
"""Summarize balance_logger CSV files for safe controller tuning."""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter
from pathlib import Path
from typing import Any

import numpy as np


NUMERIC_COLUMNS = (
    "epoch_ms",
    "latency_us",
    "control_hz",
    "pitch_rad",
    "pitch_rate_rad_s",
    "position_m",
    "velocity_m_s",
    "left_torque_nm",
    "right_torque_nm",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("files", nargs="+", type=Path)
    parser.add_argument("--merge", action="store_true", help="merge hourly files from one run")
    parser.add_argument("--settle-seconds", type=float, default=10.0)
    parser.add_argument("--max-seconds", type=float, help="explicitly exclude a later manual interval")
    parser.add_argument("--torque-limit", type=float, default=0.22)
    parser.add_argument("--turn-window", type=float, default=0.35)
    parser.add_argument("--turn-min-speed", type=float, default=0.025)
    parser.add_argument("--gains", type=float, nargs=4, metavar=("K1", "K2", "K3", "K4"))
    parser.add_argument("--json", action="store_true")
    return parser.parse_args()


def is_true(value: str | None) -> bool:
    return (value or "").strip().lower() in {"1", "true", "yes", "ok"}


def read_file(path: Path) -> tuple[list[dict[str, str]], list[dict[str, float]]]:
    with path.open(newline="", encoding="utf-8-sig") as stream:
        raw = list(csv.DictReader(stream))

    active: list[dict[str, float]] = []
    for row in raw:
        if row.get("state") != "active" or not is_true(row.get("read_ok", "1")):
            continue
        try:
            item = {key: float(row.get(key, "nan")) for key in NUMERIC_COLUMNS}
        except (TypeError, ValueError):
            continue
        if all(math.isfinite(item[key]) for key in ("epoch_ms", "pitch_rad", "pitch_rate_rad_s")):
            active.append(item)
    return raw, active


def rms(values: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(values)))) if values.size else math.nan


def dominant_frequency(t: np.ndarray, values: np.ndarray, low: float, high: float) -> float | None:
    if len(t) < 16 or t[-1] <= t[0]:
        return None
    dt = float(np.median(np.diff(t)))
    if dt <= 0:
        return None
    uniform_t = np.arange(t[0], t[-1] + 0.5 * dt, dt)
    uniform_y = np.interp(uniform_t, t, values)
    if len(uniform_y) >= 3:
        uniform_y -= np.polyval(np.polyfit(uniform_t, uniform_y, 1), uniform_t)
    else:
        uniform_y -= np.mean(uniform_y)
    spectrum = np.abs(np.fft.rfft(uniform_y * np.hanning(len(uniform_y))))
    frequency = np.fft.rfftfreq(len(uniform_y), dt)
    candidates = np.where((frequency >= low) & (frequency <= high))[0]
    if not candidates.size:
        return None
    return float(frequency[candidates[np.argmax(spectrum[candidates])]])


def subset_metrics(data: dict[str, np.ndarray], mask: np.ndarray, torque_limit: float) -> dict[str, float]:
    pitch = data["pitch_rad"][mask]
    rate = data["pitch_rate_rad_s"][mask]
    position = data["position_m"][mask]
    velocity = data["velocity_m_s"][mask]
    left = data["left_torque_nm"][mask]
    right = data["right_torque_nm"][mask]
    torque = np.maximum(np.abs(left), np.abs(right))
    return {
        "pitch_mean_deg": math.degrees(float(np.mean(pitch))),
        "pitch_rms_deg": math.degrees(rms(pitch)),
        "pitch_max_abs_deg": math.degrees(float(np.max(np.abs(pitch)))),
        "pitch_rate_rms_rad_s": rms(rate),
        "position_mean_m": float(np.mean(position)),
        "position_range_m": float(np.ptp(position)),
        "position_delta_m": float(position[-1] - position[0]),
        "position_final_m": float(position[-1]),
        "velocity_mean_m_s": float(np.mean(velocity)),
        "velocity_rms_m_s": rms(velocity),
        "velocity_max_abs_m_s": float(np.max(np.abs(velocity))),
        "left_torque_mean_nm": float(np.mean(left)),
        "torque_rms_nm": rms(torque),
        "torque_saturation_percent": 100.0 * float(np.mean(torque >= 0.995 * torque_limit)),
    }


def analyze(name: str, raw: list[dict[str, str]], rows: list[dict[str, float]], args: argparse.Namespace) -> dict[str, Any]:
    result: dict[str, Any] = {
        "name": name,
        "rows": len(raw),
        "states": dict(Counter(row.get("state", "") for row in raw)),
        "read_errors": sum(not is_true(row.get("read_ok", "1")) for row in raw),
        "final_state": raw[-1].get("state") if raw else None,
        "final_fault": raw[-1].get("fault") if raw else None,
    }
    if len(rows) < 3:
        result["error"] = "fewer than three valid active samples"
        return result

    rows.sort(key=lambda row: row["epoch_ms"])
    epoch = np.array([row["epoch_ms"] for row in rows], dtype=float) / 1000.0
    t = epoch - epoch[0]
    if args.max_seconds is not None:
        keep = t <= args.max_seconds
        rows = [row for row, selected in zip(rows, keep) if selected]
        t = t[keep]
    data = {key: np.array([row[key] for row in rows], dtype=float) for key in NUMERIC_COLUMNS}
    valid_dt = np.diff(t)
    valid_dt = valid_dt[valid_dt > 0]
    all_mask = np.ones(len(t), dtype=bool)
    settled_mask = t >= args.settle_seconds
    if not np.any(settled_mask):
        settled_mask = all_mask

    result.update(
        {
            "active_samples": len(t),
            "active_duration_s": float(t[-1]),
            "logger_rate_median_hz": float(1.0 / np.median(valid_dt)) if valid_dt.size else None,
            "logger_rate_mean_hz": float(1.0 / np.mean(valid_dt)) if valid_dt.size else None,
            "latency_mean_us": float(np.nanmean(data["latency_us"])),
            "latency_p95_us": float(np.nanpercentile(data["latency_us"], 95)),
            "latency_max_us": float(np.nanmax(data["latency_us"])),
            "firmware_control_hz": sorted(set(data["control_hz"][np.isfinite(data["control_hz"])])),
            "overall": subset_metrics(data, all_mask, args.torque_limit),
            "settled": subset_metrics(data, settled_mask, args.torque_limit),
            "position_peak_hz": dominant_frequency(t, data["position_m"], 0.03, 1.0),
            "pitch_rate_peak_hz": dominant_frequency(t, data["pitch_rate_rad_s"], 2.0, 15.0),
        }
    )

    median_dt = float(np.median(valid_dt)) if valid_dt.size else 0.02
    refractory = max(1, int(round(0.5 / median_dt)))
    half_window = max(1, int(round(args.turn_window / median_dt)))
    velocity = data["velocity_m_s"]
    turn_indices: list[int] = []
    for index in range(2, len(velocity) - 2):
        crossed = velocity[index - 1] * velocity[index + 1] < 0
        meaningful = max(abs(velocity[index - 1]), abs(velocity[index + 1])) > args.turn_min_speed
        separated = not turn_indices or index - turn_indices[-1] > refractory
        if crossed and meaningful and separated:
            turn_indices.append(index)
    turn_mask = np.zeros(len(t), dtype=bool)
    for index in turn_indices:
        turn_mask[max(0, index - half_window) : min(len(t), index + half_window + 1)] = True
    torque = np.maximum(np.abs(data["left_torque_nm"]), np.abs(data["right_torque_nm"]))
    result["turning"] = {
        "count": len(turn_indices),
        "pitch_rate_rms_rad_s": rms(data["pitch_rate_rad_s"][turn_mask]),
        "other_pitch_rate_rms_rad_s": rms(data["pitch_rate_rad_s"][~turn_mask]),
        "torque_rms_nm": rms(torque[turn_mask]),
        "other_torque_rms_nm": rms(torque[~turn_mask]),
        "torque_saturation_percent": 100.0 * float(np.mean(torque[turn_mask] >= 0.995 * args.torque_limit)) if np.any(turn_mask) else math.nan,
    }

    if args.gains:
        state = np.column_stack(
            (data["pitch_rad"], data["pitch_rate_rad_s"], data["position_m"], data["velocity_m_s"])
        )
        contributions = -0.5 * state * np.asarray(args.gains)
        result["single_wheel_torque_contribution_rms_nm"] = {
            key: rms(contributions[:, index])
            for index, key in enumerate(("pitch", "pitch_rate", "position", "velocity"))
        }
        predicted = np.sum(contributions, axis=1)
        result["predicted_single_wheel_torque_rms_nm"] = rms(predicted)
    return result


def print_report(report: dict[str, Any]) -> None:
    print(f"\n== {report['name']} ==")
    print(
        f"rows={report['rows']} states={report['states']} read_errors={report['read_errors']} "
        f"final={report['final_state']} fault={report['final_fault']}"
    )
    if "error" in report:
        print(f"error: {report['error']}")
        return
    print(
        f"active={report['active_duration_s']:.2f}s logger={report['logger_rate_mean_hz']:.2f}Hz "
        f"control={report['firmware_control_hz']} latency_p95={report['latency_p95_us']:.0f}us"
    )
    for label in ("overall", "settled"):
        item = report[label]
        print(
            f"{label}: pitch_mean={item['pitch_mean_deg']:+.3f}deg "
            f"pitch_rms={item['pitch_rms_deg']:.3f}deg max={item['pitch_max_abs_deg']:.3f}deg "
            f"rate_rms={item['pitch_rate_rms_rad_s']:.3f}rad/s velocity_rms={item['velocity_rms_m_s']:.3f}m/s "
            f"position_mean={item['position_mean_m']:+.3f}m range={item['position_range_m']:.3f}m "
            f"torque_rms={item['torque_rms_nm']:.3f}Nm sat={item['torque_saturation_percent']:.2f}%"
        )
    turning = report["turning"]
    print(
        f"frequency: position={report['position_peak_hz']}Hz pitch_rate={report['pitch_rate_peak_hz']}Hz; "
        f"turns={turning['count']} turn_rate={turning['pitch_rate_rms_rad_s']:.3f} "
        f"other_rate={turning['other_pitch_rate_rms_rad_s']:.3f}rad/s"
    )
    if "single_wheel_torque_contribution_rms_nm" in report:
        print(f"torque contribution RMS: {report['single_wheel_torque_contribution_rms_nm']}")


def main() -> int:
    args = parse_args()
    loaded = [(path, *read_file(path)) for path in args.files]
    reports: list[dict[str, Any]] = []
    if args.merge:
        raw = [row for _, raw_rows, _ in loaded for row in raw_rows]
        active = [row for _, _, active_rows in loaded for row in active_rows]
        reports.append(analyze(" + ".join(str(path) for path in args.files), raw, active, args))
    else:
        reports.extend(analyze(str(path), raw, active, args) for path, raw, active in loaded)

    if args.json:
        print(json.dumps(reports, ensure_ascii=False, indent=2, allow_nan=True))
    else:
        for report in reports:
            print_report(report)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
