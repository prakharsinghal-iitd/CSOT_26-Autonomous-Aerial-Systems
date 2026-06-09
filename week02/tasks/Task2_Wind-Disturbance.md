# Task 2 — Wind Disturbance Model
### Week 2 · Autonomous Aerial Systems · CSOT 2026

Estimated time: 45 min

---

## What You Are Modelling

Wind on a quadcopter acts as an external force on the body frame. A steady 3 m/s crosswind from the north exerts a drag force on the airframe proportional to the wind speed and the vehicle's drag coefficient.

For this project we use a simplified but physically reasonable model:
- Steady component: constant force vector in inertial frame (3 m/s → F_wind_x)
- Turbulence component: Band-Limited White Noise added on top

The drag force magnitude is estimated as:
  F_drag = 0.5 * rho * Cd * A * v_wind^2

Where:
  rho = 1.225 kg/m^3 (air density at sea level)
  Cd  = 1.0 (bluff body drag coefficient for quadcopter frame)
  A   = 0.04 m^2 (estimated frontal area)
  v_wind = 3 m/s

Substituting: F_drag = 0.5 * 1.225 * 1.0 * 0.04 * 9 = 0.22 N

---

## Build the Wind Subsystem

Create a subsystem named "Wind Model" with no inputs and output F_wind [3x1].

Inside:

Block 1 — Steady crosswind force:
  Constant block → value: [0.22; 0; 0] (force in body X direction, NED)

Block 2 — Turbulence:
  Band-Limited White Noise block
  Noise power: [0.001; 0.001; 0]
  Sample time: 0.001

Block 3 — Sum both:
  Add Block: Constant + Noise → output F_wind [3x1]

Connect F_wind to the wind input port in your Translational Dynamics subsystem.

---

## Test

Run a 10 second hover simulation with the wind model connected.

Expected result: the quadcopter drifts in the X direction because there is no controller yet to reject the disturbance.

Scope on x position should show a parabolic drift (constant acceleration from wind force).

If x stays flat → wind is not connected correctly. Check signal routing into Translational Dynamics.

---

## Checklist

- [ ] Wind Model subsystem created
- [ ] Constant force [0.22; 0; 0] N applied
- [ ] Band-Limited White Noise turbulence added
- [ ] F_wind connected to Translational Dynamics
- [ ] 10 s hover shows X-position drift in scope
