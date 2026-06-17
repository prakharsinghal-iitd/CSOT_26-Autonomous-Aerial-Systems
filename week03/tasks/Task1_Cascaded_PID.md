# Task 1 — Cascaded PID Controller
### Week 3 · Autonomous Aerial Systems · CSOT 2026

**Estimated time:** 2 hours

**Deliverable:** Functional cascaded PID in your `.slx` with inner and outer loops connected to the Week 2 plant.

**Watch first:** Video 1 (Flight Control Law Design)

---

## Why Cascaded PID?

A single PID loop cannot control a quadcopter well because the dynamics operate at two very different timescales:

- **Attitude dynamics (fast):** Roll, pitch, and yaw respond in milliseconds to torque commands. These must be stabilised first.
- **Position dynamics (slow):** Altitude changes over seconds. You command altitude by tilting the drone (which redirects thrust).

Cascaded control solves this by nesting a fast inner loop inside a slow outer loop:

```
Altitude setpoint → [Outer Z-PID] → Attitude setpoint → [Inner attitude PID] → Motor commands → Plant → Altitude
                                         ↑_________________________________________________________|
```

The outer loop is **10x slower** than the inner loop. The inner loop must be tuned and stable before you close the outer loop. This ordering is not optional — it is the only way cascaded PID works.

---

## Architecture Overview

```
                          ┌─────────────────────────────────────────┐
                          │           Cascaded PID Controller       │
                          │                                         │
  z_setpoint ─────────►   │  [Altitude PID]  ──► θ_des (pitch cmd)  │
                          │                                         │
  φ_setpoint (0) ───────► │  [Roll PID]     ──► τ_roll              │ ──► [Mixer⁻¹] ──► ω1,ω2,ω3,ω4
                          │                                         │
      θ_des  ───────────► │  [Pitch PID]    ──► τ_pitch             │
                          │                                         │
  ψ_setpoint (0) ───────► │  [Yaw PID]      ──► τ_yaw               │
                          │                                         │
                          └─────────────────────────────────────────┘
                                 ▲ feedback: φ, θ, ψ, z from plant
```

**Note:** For hover, the altitude loop outputs a desired pitch angle θ_des. This is the "coupling" between outer and inner loops. Roll setpoint stays at 0 for pure altitude hold.

---

## Step 1 — Build the Inner Attitude Loop

### What It Controls
Three independent PID loops — one for each Euler angle:
- **Roll PID:** error = φ_setpoint − φ_measured → output: τ_roll
- **Pitch PID:** error = θ_setpoint − θ_measured → output: τ_pitch
- **Yaw PID:** error = ψ_setpoint − ψ_measured → output: τ_yaw

### Implementing a PID Block in Simulink

For each axis, use a **PID Controller** block from the Simulink / Continuous library:

1. Double-click the block → set controller type to **PID**
2. Set initial gains (tune later):
   - P = 1.0, I = 0.0, D = 0.1 (start here)
3. Enable **anti-windup** (clamping): set saturation limits ±5 N·m on the output

### Build Order for Inner Loop

**Step 1a:** Add three PID blocks, one per axis. Label them `Roll_PID`, `Pitch_PID`, `Yaw_PID`.

**Step 1b:** Add Sum blocks for error computation:
```
error_roll  = phi_setpoint   - phi_measured   (from IMU gyro-integrated or from plant phi)
error_pitch = theta_setpoint - theta_measured
error_yaw   = psi_setpoint   - psi_measured
```

For now, use the **true plant states** (φ, θ, ψ from Kinematics subsystem), not the IMU outputs. You will switch to IMU outputs in Week 5 when the EKF is ready.

**Step 1c:** Connect PID outputs (τ_roll, τ_pitch, τ_yaw) to your **Inverse Mixer** (see below).

**Step 1d:** Group in a subsystem named `Inner_Attitude_PID`.

### Inverse Mixer

The plant expects 4 rotor speeds ω1–ω4 as inputs (or you can restructure to accept Fz, τ_roll, τ_pitch, τ_yaw directly — see Note below).

**Option A (recommended):** Restructure Subsystem 1 so that the plant's top-level inputs are `[Fz, τ_roll, τ_pitch, τ_yaw]`. The Force & Torque Mixer moves from the plant into a block you can drive from the controller. This makes the controller interface clean.

**Option B:** Keep the plant as-is and add an Inverse Mixer that converts `[Fz, τ_roll, τ_pitch, τ_yaw]` → `[ω1, ω2, ω3, ω4]`.

Inverse mixer equations (+ layout, NED):
```
ω1² = (Fz/(4kT)) + (τ_roll/(2kT·L))  - (τ_pitch/(2kT·L)) - (τ_yaw/(4kQ))
ω2² = (Fz/(4kT)) - (τ_roll/(2kT·L))  - (τ_pitch/(2kT·L)) + (τ_yaw/(4kQ))
ω3² = (Fz/(4kT)) - (τ_roll/(2kT·L))  + (τ_pitch/(2kT·L)) - (τ_yaw/(4kQ))
ω4² = (Fz/(4kT)) + (τ_roll/(2kT·L))  + (τ_pitch/(2kT·L)) + (τ_yaw/(4kQ))
```
Take `sqrt()` of each result; clamp to [0, ω_max] where ω_max = 900 rad/s.

---

## Step 2 — Tune the Inner Loop

**Do not skip tuning.** An untuned inner loop makes the outer loop impossible to close.

### Manual Tuning Procedure

Run a test simulation: step input of φ_setpoint = 0.1 rad (≈6°). Monitor φ response.

**Starting gains:** `Kp = 1.0, Ki = 0.0, Kd = 0.1`

| Symptom | Action |
|---|---|
| Response too slow, no oscillation | Increase Kp |
| Oscillation that grows | Decrease Kp or increase Kd |
| Steady-state error remains | Add small Ki (start at 0.05) |
| Response overshoots badly | Reduce Kp, increase Kd |

**Targets for inner loop:**
- Settling time < 0.5 s for a 0.1 rad step
- Overshoot < 20%
- Steady-state error < 0.01 rad (with integral term)

### Suggested Gain Starting Points (tune from here)

| Axis | Kp | Ki | Kd |
|---|---|---|---|
| Roll | 4.0 | 0.5 | 0.8 |
| Pitch | 4.0 | 0.5 | 0.8 |
| Yaw | 2.0 | 0.1 | 0.3 |

These are starting values based on the given inertia parameters. Your tuned values may differ.

---

## Step 3 — Build the Outer Altitude Loop

### What It Controls
One PID loop for altitude (z-axis in NED):

```
error_z = z_setpoint − z_measured        (remember: NED z is positive down → negate for altitude)
Altitude PID output → θ_des (desired pitch angle)
```

**Sign convention:** In NED, flying upward means z is decreasing (becoming more negative). Take care with signs:
- Altitude = −z (positive up)
- Altitude error = z_setpoint_altitude − (−z_plant)

### Building the Altitude PID

1. Add a **PID Controller** block. Label it `Altitude_PID`.
2. Connect `z_measured` from the barometer output `baro_alt_meas` (or true plant z).
3. Saturate output: ±0.3 rad (±17°) — this is the maximum tilt the altitude loop can command.
4. Connect output to `theta_setpoint` input of the inner Pitch PID.

**Gains to start with:**

| Gain | Value |
|---|---|
| Kp | 0.5 |
| Ki | 0.1 |
| Kd | 0.2 |

**Note on hover thrust:** The altitude PID output is a pitch angle correction, not a thrust command. The total thrust Fz is set separately: at hover `Fz = m × g = 0.5 × 9.81 = 4.905 N`. Add this as a feedforward constant so the drone does not have to learn to hover from zero.

```
Fz_total = Fz_hover_feedforward + Fz_altitude_PID_output
```
where `Fz_altitude_PID_output` is an additional correction from a second PID specifically on altitude. Many implementations use a separate Z-velocity inner loop — for this week, a single altitude PID with feedforward is sufficient.

### Group in Subsystem

Create a subsystem named `Controller` containing:
```
Controller
├── Outer_Altitude_PID
└── Inner_Attitude_PID
    ├── Roll_PID
    ├── Pitch_PID
    └── Yaw_PID
```

---

## Step 4 — Connect Controller to Plant

At the top level of your model:

```
[Setpoints] → [Controller] → [Fz, τ_roll, τ_pitch, τ_yaw] → [Plant] → [States]
                    ↑                                                         |
                    └────────── feedback: φ, θ, ψ, z ──────────────────────┘
```

Setpoints for hover test:
- `z_setpoint` = −1.0 m (1 metre altitude in NED, so z = −1)
- `phi_setpoint` = 0 rad
- `psi_setpoint` = 0 rad

Use **Constant** blocks for these during the manual hover test. The Stateflow manager (Task 2) will replace these constants with mode-dependent setpoints.

---

## Step 5 — Initial Test (No Wind)

Before adding wind:
1. Set z_setpoint = −1.0 m
2. Disable the wind model temporarily (replace F_wind with [0;0;0])
3. Run 10-second simulation
4. Scope on z — must reach and hold −1.0 m

If the drone diverges:
- Check sign conventions (NED z vs altitude)
- Check that Fz_feedforward = 4.905 N is present
- Reduce outer loop gains first

---

## Checklist

- [ ] Inverse Mixer implemented (or plant restructured to accept [Fz, τ])
- [ ] Roll, Pitch, Yaw PID blocks built and labelled
- [ ] Inner attitude loop step-tested (0.1 rad roll step, settles in < 0.5 s)
- [ ] Altitude PID built with hover feedforward
- [ ] Altitude PID output saturated at ±0.3 rad
- [ ] Controller subsystem organised: Outer → Inner
- [ ] Controller connected to Week 2 plant with feedback signals
- [ ] Hover test (no wind): z reaches −1.0 m and holds
