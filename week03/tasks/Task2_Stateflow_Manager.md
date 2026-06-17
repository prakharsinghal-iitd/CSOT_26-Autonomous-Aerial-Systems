# Task 2 — Stateflow Flight Manager
### Week 3 · Autonomous Aerial Systems · CSOT 2026

**Estimated time:** 1.5 hours

**Deliverable:** Stateflow chart with 4 flight modes, integrated into the model.

**Watch first:** Video 2 (Flight Management Design)

---

## Why a Flight Manager?

A drone autopilot does not simply run the same PID setpoints forever. During takeoff the altitude setpoint rises; during hover it holds; during landing it descends and disarms motors. Without explicit mode management, you would need complex conditional logic scattered throughout your model — impossible to debug or extend.

A **Stateflow chart** (hierarchical finite state machine) gives you a clean, visual, verifiable way to express mode logic. It is also the industry standard: every commercial flight controller (PX4, ArduPilot, DJI) implements a flight mode state machine.

Your state machine directly satisfies requirement **R7** (graceful mode transitions: Takeoff → Hover → Land, no abrupt jumps).

---

## Flight Modes

| Mode | What it does |
|---|---|
| **Idle** | Motors off. Drone on ground. PIDs disabled. |
| **Takeoff** | Altitude setpoint ramps up to 1.0 m. PIDs active. |
| **Hover** | Altitude setpoint holds at 1.0 m. Wind active. |
| **Land** | Altitude setpoint ramps down to 0. Motors disarm at ground. |

---

## Step 1 — Add Stateflow to Your Model

1. In the Simulink Library Browser, go to **Stateflow** → drag a **Chart** block into your top-level model.
2. Double-click the chart to open the Stateflow editor.
3. The chart will have inputs and outputs that you define.

### Chart Inputs
- `t` — simulation time (connect from **Clock** block at top level)
- `arm_cmd` — arm/disarm command (use a **Constant** block set to 1 to arm)

### Chart Outputs
- `z_setpoint` — altitude setpoint (m, NED: negative is up)
- `mode` — integer flag (1=Idle, 2=Takeoff, 3=Hover, 4=Land)
- `motors_enabled` — boolean (0 = motors off, 1 = on)

To define outputs in Stateflow:
- Open the chart → go to **Symbols** pane → add each output as **Output** type, data type `double`.

---

## Step 2 — Build the State Machine

Draw the following states inside the chart. Each state is a rectangle with a name.

### State 1: IDLE

```
IDLE
───────────────────────────────
entry: z_setpoint = 0;
       motors_enabled = 0;
       mode = 1;
```

**Default transition:** The chart starts in IDLE (add a default transition arrow to IDLE).

**Transition out:** `[arm_cmd == 1]` → **TAKEOFF**

### State 2: TAKEOFF

```
TAKEOFF
───────────────────────────────
entry: motors_enabled = 1;
       mode = 2;

during: z_setpoint = -min(1.0, 0.2 * (t - t_takeoff_start));
        % ramp altitude setpoint at 0.2 m/s up to 1.0 m in NED (negative)
```

**About `t_takeoff_start`:** Store the entry time using a local variable. In Stateflow:
```
entry: t_takeoff_start = t;
       motors_enabled = 1;
       mode = 2;
```

**Transition out:** `[z_setpoint <= -0.95]` → **HOVER**
(z = −0.95 means we have climbed to ~95% of target altitude, close enough)

Alternatively use a time-based guard: `[after(6, sec)]` → **HOVER**
(simpler; the ramp at 0.2 m/s reaches 1.0 m in 5 s, so 6 s gives a buffer)

### State 3: HOVER

```
HOVER
───────────────────────────────
entry: z_setpoint = -1.0;
       mode = 3;

during: z_setpoint = -1.0;   % hold constant
```

**Transition out:** `[after(30, sec)]` → **LAND**
(hold hover for 30 seconds — this is the window your RMSE test runs in)

### State 4: LAND

```
LAND
───────────────────────────────
entry: t_land_start = t;
       mode = 4;

during: z_setpoint = max(0.0, -1.0 + 0.2 * (t - t_land_start));
        % ramp setpoint back to 0 at 0.2 m/s
        if z_setpoint >= -0.05
            motors_enabled = 0;    % disarm when near ground
        end
```

**No transition out of LAND** — the simulation ends (or loops back to IDLE if you add a reset).

---

## Step 3 — Transition Diagram

Draw the transitions in Stateflow as arrows between states:

```
        arm_cmd == 1
IDLE ─────────────────► TAKEOFF ──── after(6, sec) ───► HOVER ──── after(30, sec) ───► LAND
                                          or
                                   [z_setpoint <= -0.95]
```

Each arrow must have a **condition** (in square brackets) or an **event** (keyword `after`). No unconditional transitions between states.

---

## Step 4 — Connect Stateflow Outputs to Controller

At the top level, replace the **Constant** blocks for setpoints with the Stateflow chart outputs:

| Old block | Replace with |
|---|---|
| `z_setpoint = Constant(-1.0)` | `z_setpoint` from chart output |
| `motors_enabled = Constant(1)` | `motors_enabled` from chart output |

Connect `motors_enabled` to a **Product** block before the motor speed signal going to the plant:
```
motor_speeds × motors_enabled → plant input
```
This cuts motor thrust to zero in IDLE and LAND (after landing).

Add a **To Workspace** block on `mode` so you can plot the mode transitions after simulation.

---

## Step 5 — Configure the Stateflow Chart

Double-click the chart block → **Properties**:
- **Update method:** Inherited (same step as Simulink solver, 0.001 s)
- **Enable zero-crossing detection:** On
- **Chart initialisation at time zero:** On (ensures IDLE is entered at t = 0)

---

## Step 6 — Test the State Machine in Isolation

Before connecting to the full plant:
1. Connect only a **Clock** block and `arm_cmd = 1` to the chart
2. Add **Display** blocks on `z_setpoint` and `mode`
3. Run for 45 seconds
4. Verify mode sequence: 1 → 2 → 3 → 4 at correct times
5. Verify `z_setpoint` ramps from 0 to −1.0, holds, then ramps back to 0

---

## Step 7 — Full Integration Test

With Stateflow connected to the controller and plant:
1. Set `arm_cmd = 1` (Constant block)
2. Run for 40 seconds
3. Open Scopes on z-position and `mode`
4. Expect: drone lifts to 1 m, holds, then descends

**If the drone oscillates during takeoff:** The altitude ramp is commanding a fast climb. Reduce ramp rate from 0.2 m/s to 0.1 m/s in the TAKEOFF during action.

**If mode transition does not fire:** Check that `t_takeoff_start` is correctly set in entry action, and that the chart is executing (add a display block on `mode` and run a short simulation).

---

## Common Mistakes

| Mistake | Symptom | Fix |
|---|---|---|
| Default transition missing | Chart never enters IDLE at t=0 | Add a default transition arrow pointing to IDLE |
| `after(N, sec)` used in wrong scope | Transition fires immediately or never | `after()` resets when you enter a state — must be inside the source state |
| z_setpoint sign error | Drone dives into ground on takeoff | In NED: altitude 1 m = z = −1 m. Setpoint must be negative |
| `motors_enabled` not connected | Drone thrusts in IDLE state | Wire `motors_enabled` to a Product gate on motor outputs |
| Chart update method = discrete | Mismatch with continuous solver | Set to Inherited or Continuous |

---

## Checklist

- [ ] Stateflow chart added to top-level model
- [ ] IDLE, TAKEOFF, HOVER, LAND states defined
- [ ] Entry and during actions written for each state
- [ ] Correct transition conditions between all states
- [ ] Default transition to IDLE at simulation start
- [ ] `z_setpoint` output wired to Controller altitude PID
- [ ] `motors_enabled` output gating motor signals
- [ ] `mode` logged to workspace
- [ ] Isolation test: modes sequence correctly over 45 s
- [ ] Full integration test: drone takes off, hovers, lands
