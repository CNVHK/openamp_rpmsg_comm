---
name: tune-wheel-balance-control
description: Analyze two-wheel and fixed-leg wheel-legged robot balance CSV logs, diagnose state-feedback/LQR oscillation, drift, saturation, trim, filtering, and safety faults, and plan controlled runtime gain experiments. Use when Codex needs to compare balance_logger CSV files, tune K1-K4 or pitch trim/filter settings, distinguish posture-loop oscillation from position-loop motion, validate a balancing run, or update the plant model from real hardware evidence.
---

# Tune Wheel Balance Control

Use measured evidence to tune a two-wheel balance controller without confusing logger artifacts, manual intervention, or unmodeled dynamics with controller behavior.

## Core Rules

1. Treat safety protections as constraints, not parameters to relax during tuning.
2. Record the exact runtime configuration before every run. Never infer gains, trim, cutoff, torque limit, or speed limit from a filename.
3. Change one independent factor per comparison. Change coupled gains together only when a pole calculation or model explicitly justifies it.
4. Repeat important configurations. A short surviving run is not proof of stability.
5. Separate verified measurements, likely explanations, and unresolved hypotheses.
6. Call manually modified LQR gains a state-feedback controller initialized from LQR; do not claim they remain optimal for the original Q/R.

## Workflow

### 1. Establish the experiment

Capture:

- controller frequency and logger rate;
- K1, K2, K3, K4;
- pitch trim and pitch-rate filter cutoff;
- torque and wheel-speed limits;
- robot mass/configuration, floor, battery, payload, leg height, and support-rope condition;
- whether the final fault or tail interval was caused manually.

Reject comparisons that changed multiple undocumented conditions.

### 2. Validate and summarize logs

Run:

```bash
python3 .codex/skills/tune-wheel-balance-control/scripts/analyze_balance_log.py \
  path/to/balance.csv \
  --gains K1 K2 K3 K4
```

For hourly split files from one run, use `--merge`. Use `--max-seconds` only when the user explicitly identifies a later interval as manual intervention.

Inspect:

- active duration, read errors, final state/fault;
- logger timestamp rate separately from the firmware `control_hz` field;
- pitch mean/RMS/max and pitch-rate RMS;
- position mean/range/delta and velocity RMS/max;
- torque RMS and saturation percentage;
- low-frequency position peak and 2-15 Hz pitch-rate peak;
- turning-point metrics versus ordinary running;
- approximate K1-K4 torque contribution when gains are available.

Do not interpret Linux logger jitter as a real-time control-loop overrun when firmware `control_hz` and fault telemetry remain valid.

### 3. Classify the dominant problem

Use this order:

1. **Sign, unit, or feedback validity error**: motion runs away immediately, wheel directions disagree, feedback is stale, or measured speed contradicts position delta.
2. **Safety/configuration fault**: decode the fault before tuning gains.
3. **Sustained high-frequency posture oscillation**: pitch rate and torque share a stable peak, commonly with saturation growth.
4. **Turning-local high-frequency burst**: ordinary motion is quiet but reversal windows have much larger pitch-rate/torque RMS.
5. **Slow position oscillation or drift**: posture remains bounded while position and velocity cycle or migrate.
6. **Mechanical/IMU trim bias**: long-run rate and velocity approach zero while pitch error and position offset consistently cancel in the state-feedback law.

Read [references/tuning-guide.md](references/tuning-guide.md) before recommending a parameter change.

### 4. Select the next experiment

Preserve the best known rollback configuration. Choose the smallest safe change that tests one hypothesis:

- cutoff sweep for delay/noise discrimination;
- K2 sweep for pitch-rate feedback interaction;
- coordinated K3/K4 change for modeled outer-loop pole placement;
- trim calibration from repeated steady-state evidence;
- optional notch or observer only after a repeatable narrowband mode remains.

Keep torque, speed, fall-angle, feedback-timeout, and emergency-stop protections enabled. Prefer automatic log duration and stop-on-fault over manually lifting the robot to end a run.

### 5. Decide from repeated evidence

Compare effect sizes, not isolated samples. Require improvement in the target metric without unacceptable regression in pitch, velocity, saturation, or safety margin.

When a filter cutoff is changed, calculate its discrete magnitude and phase at both the balance bandwidth and the observed oscillation frequency. Lower cutoff can add destabilizing phase delay even while reducing noise.

### 6. Return to the model

Treat empirical tuning as plant identification. Update the discrete model with measured:

- mass, center of mass, inertia, and leg height;
- motor torque scale and actuator dynamics;
- command/feedback delays and zero-order hold;
- IMU/filter dynamics;
- torque quantization, saturation, tire compliance, and structural modes.

Then redesign with augmented discrete LQR/LQG, identified-state feedback, or another justified robust method. Revalidate after payload or geometry changes.

## Output Format

Report:

1. data-quality and configuration caveats;
2. a compact comparison table;
3. verified observations;
4. likely control interpretation with confidence;
5. one next experiment, rollback values, and acceptance criteria;
6. parameters that must remain unchanged during that experiment.

Never recommend removing safety limits merely to extend a failing run.
