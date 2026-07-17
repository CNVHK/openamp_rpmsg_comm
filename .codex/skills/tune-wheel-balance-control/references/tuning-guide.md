# Wheel Balance Tuning Guide

## Contents

1. State-feedback interpretation
2. Diagnostic patterns
3. Safe experiment design
4. Filter and delay treatment
5. Trim estimation
6. Model-based completion

## State-feedback interpretation

For state `x = [pitch, pitch_rate, position, velocity]` and total wheel torque:

```text
tau_total = -(K1*pitch + K2*pitch_rate + K3*position + K4*velocity)
```

Interpret changes using the actual project sign convention:

- `K1`: posture stiffness and gravitational recovery.
- `K2`: posture-rate damping. Excess delay can make nominal damping excite a structural/actuator mode.
- `K3`: position restoration. Increasing magnitude usually restores position sooner but can increase outer-loop oscillation.
- `K4`: translational velocity damping. Coordinate it with K3 when changing outer-loop poles.
- `trim`: IMU/mechanical upright reference, not a substitute for position control.

Always verify signs from source and a low-risk direction test. Do not apply generic sign advice blindly.

## Diagnostic patterns

### Sustained posture limit cycle

Evidence:

- pitch rate and signed torque share a narrowband peak;
- amplitude or saturation grows after an initially quiet interval;
- position terms are much smaller than pitch-rate contribution.

Actions:

1. Verify actuator and feedback delay.
2. Compare higher versus lower measurement-filter cutoff without changing gains.
3. If lower cutoff worsens the mode, suspect phase delay rather than raw noise.
4. Sweep K2 conservatively while holding K1/K3/K4 fixed.
5. Add a notch/observer only if a repeatable narrowband mode remains.

### Turning-point burst

Evidence:

- ordinary pitch-rate RMS is low;
- reversal windows show much larger pitch-rate and torque RMS;
- bursts coincide with wheel velocity sign changes.

Actions:

1. Quantify turn versus non-turn metrics.
2. Confirm torque quantization, friction, dead zone, and actuator reversal behavior.
3. Avoid increasing K3 alone.
4. Increase K4 only with pole/model support; otherwise test a small K2 change or targeted notch.

### Slow position oscillation

Evidence:

- posture is bounded;
- position has a low-frequency peak and repeated reversals;
- velocity peaks near position zero and changes sign at extrema.

Actions:

1. Compute discrete closed-loop poles.
2. Tune K3/K4 as an outer-loop pair.
3. Prefer a critically damped or mildly overdamped candidate over stronger position stiffness alone.

### Position offset with near-zero velocity

Check the steady-state means. If pitch rate, velocity, and torque are near zero while pitch and position cancel in `Kx`, estimate a trim bias. Repeat on the same level surface and orientation before changing trim.

## Safe experiment design

- Keep a physical safety support slack enough not to carry normal robot weight.
- Start from stationary wheels and acceptable arm pitch/rate.
- Use automatic duration and stop-on-fault.
- Preserve fall, wheel-speed, torque, CAN, feedback freshness, and emergency-stop protections.
- Repeat each important configuration at least twice.
- Use 20-60 seconds after initial validation; narrowband modes may take several seconds to grow.
- Do not include a manually lifted tail in natural stability metrics. Exclude it only when explicitly identified.

Suggested acceptance metrics must be adapted to the robot. Typical fixed-leg development gates are pitch RMS, maximum pitch, velocity RMS/max, torque saturation percentage, active duration, position range, and turn-local pitch-rate RMS.

## Filter and delay treatment

For the implemented first-order filter:

```text
alpha = dt / (1/(2*pi*fc) + dt)
y[k] = y[k-1] + alpha*(x[k] - y[k-1])
H(z) = alpha / (1 - (1-alpha) z^-1)
```

Calculate discrete magnitude and phase at frequencies of interest. Do not describe a higher cutoff as "more filtering". A higher cutoff reduces smoothing and delay.

Filtering is sensor conditioning, not a replacement for the plant model. Include filter state/dynamics in the augmented design when finalizing LQR/LQG.

## Trim estimation

Use long steady intervals, not rope angle or a single pose.

1. Keep gains and surface unchanged.
2. Measure mean pitch error, position, velocity, rate, and torque after settling.
3. Test two nearby trim values.
4. Interpolate only if the response is repeatable and monotonic.
5. Stop chasing sub-degree trim when floor slope, tire asymmetry, IMU temperature, or payload motion dominates repeatability.

After changing trim, disable and re-enable balance so the wheel-position reference is re-zeroed.

## Model-based completion

The final controller should model or identify:

- rigid-body dynamics at each fixed leg height;
- payload-dependent mass, center of mass, and pitch inertia;
- motor torque gain, bandwidth, slew, delay, and quantization;
- CAN/feedback sampling and command delay;
- IMU fusion and explicit filter dynamics;
- tire/ground compliance and repeatable structural modes.

Use fixed-height validation at several leg lengths before continuous dynamic height. Schedule or interpolate gains by leg length, then integrate VMC force/torque mapping without discarding the wheel-balance loop.
