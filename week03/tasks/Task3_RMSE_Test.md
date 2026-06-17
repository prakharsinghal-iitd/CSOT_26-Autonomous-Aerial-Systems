# Task 3 — Hover Test, RMSE, & Plots
### Week 3 · Autonomous Aerial Systems · CSOT 2026

**Estimated time:** 45 minutes

**Deliverable:** `hover_plots_wk3_YOUR_NAME.png` + `rmse_result_wk3_YOUR_NAME.png`

---

## What You Are Measuring

Root Mean Square Error (RMSE) is the standard metric for evaluating altitude hold performance. It measures how far the actual altitude deviates from the setpoint over time:

```
RMSE = sqrt( mean( (z_actual(t) - z_setpoint(t))² ) )
```

Your target is **RMSE < 0.3 m** over a 30-second hover. This directly tests requirement **R1**.

---

## Step 1 — Set Up the Simulation

Before running the RMSE test, configure the model:

1. **Enable wind:** Make sure the Wind Model subsystem from Week 2 is active (F_wind ≠ 0). This is important — the RMSE must be computed with wind active.
2. **Enable Stateflow manager:** arm_cmd = 1, so the drone sequences through Takeoff → Hover → Land.
3. **Log all signals:** Add `To Workspace` blocks on:
   - `z` — true z-position from plant (NED, negative is up)
   - `phi`, `theta`, `psi` — Euler angles from plant
   - (already logged from Week 2: `accel_meas`, `gyro_meas`, `baro_alt_meas`)

4. **Simulation time:** Set to **40 seconds** (6 s takeoff + 30 s hover + 4 s land buffer).

---

## Step 2 — Run the Simulation

Run the 40-second simulation. If the drone diverges (position or angles blow up):
- Check that the inner attitude loop is tuned before testing with wind
- Reduce outer loop Kp by 50% and retry
- Check NED sign conventions throughout

---

## Step 3 — Compute Altitude RMSE in MATLAB

After the simulation completes, run the following in the MATLAB Command Window:

```matlab
% ─── Extract altitude from simulation output ───────────────────────────
% True altitude (positive up) from plant NED z (positive down)
z_NED     = squeeze(out.z.Data);        % NED z, positive downward
alt_true  = -z_NED;                     % altitude (positive up)
t_vec     = out.z.Time;

% ─── Define Hover window (t = 6 s to 36 s) ─────────────────────────────
hover_start = 6.0;      % seconds — when TAKEOFF completes
hover_end   = 36.0;     % seconds — 30 s hover window
hover_mask  = (t_vec >= hover_start) & (t_vec <= hover_end);

alt_hover   = alt_true(hover_mask);
z_setpoint_altitude = 1.0;             % target altitude in metres (positive up)

% ─── Compute RMSE ────────────────────────────────────────────────────────
alt_error   = alt_hover - z_setpoint_altitude;
RMSE_alt    = sqrt(mean(alt_error.^2));
fprintf('\n========================================\n');
fprintf('Altitude RMSE (30 s hover, wind ON): %.4f m\n', RMSE_alt);
if RMSE_alt < 0.3
    fprintf('R1 STATUS: PASS (< 0.3 m)\n');
else
    fprintf('R1 STATUS: FAIL (>= 0.3 m) — retune gains\n');
end
fprintf('========================================\n\n');
```

**Take a screenshot of this MATLAB Command Window output.** Save as `rmse_result_wk3_YOUR_NAME.png`.

---

## Step 4 — Check Attitude (R2 Verification)

```matlab
% ─── Extract attitude signals ────────────────────────────────────────────
phi_deg   = squeeze(out.phi.Data)   * 180/pi;
theta_deg = squeeze(out.theta.Data) * 180/pi;

phi_hover   = phi_deg(hover_mask);
theta_hover = theta_deg(hover_mask);

max_roll    = max(abs(phi_hover));
max_pitch   = max(abs(theta_hover));

fprintf('Max roll  during hover: %.2f deg\n', max_roll);
fprintf('Max pitch during hover: %.2f deg\n', max_pitch);

if max_roll < 5 && max_pitch < 5
    fprintf('R2 STATUS: PASS (roll & pitch within ±5°)\n');
else
    fprintf('R2 STATUS: FAIL — exceeds ±5°\n');
end
```

---

## Step 5 — Generate Hover Plots

```matlab
% ─── Figure setup ────────────────────────────────────────────────────────
figure('Position', [100 100 1200 800]);
sgtitle('Week 3 — Closed-Loop Hover Performance (30 s, Wind ON)', ...
        'FontSize', 14, 'FontWeight', 'bold');

% ─── Plot 1: Altitude ────────────────────────────────────────────────────
subplot(2,2,1);
plot(t_vec, alt_true, 'b-', 'LineWidth', 1.5, 'DisplayName', 'True altitude');
hold on;
yline(1.0, 'r--', 'LineWidth', 1.2, 'DisplayName', 'Setpoint (1.0 m)');
yline(1.3, 'k:', 'LineWidth', 1.0, 'DisplayName', '+0.3 m bound');
yline(0.7, 'k:', 'LineWidth', 1.0, 'DisplayName', '-0.3 m bound');
xlim([0 40]); ylim([-0.5 2.0]);
xlabel('Time (s)'); ylabel('Altitude (m)');
title(sprintf('Altitude (RMSE = %.3f m, R1 = %s)', RMSE_alt, ...
              string(RMSE_alt < 0.3, 'PASS', 'FAIL')));
legend('Location', 'southeast'); grid on;

% ─── Plot 2: Roll angle ──────────────────────────────────────────────────
subplot(2,2,2);
plot(t_vec, phi_deg, 'g-', 'LineWidth', 1.2, 'DisplayName', 'Roll φ');
hold on;
yline(5,  'r--', 'LineWidth', 1.0);
yline(-5, 'r--', 'LineWidth', 1.0, 'DisplayName', '±5° bound');
xlim([0 40]); ylim([-15 15]);
xlabel('Time (s)'); ylabel('Roll (deg)');
title(sprintf('Roll Angle (max = %.1f°, R2 = %s)', max_roll, ...
              string(max_roll < 5, 'PASS', 'FAIL')));
legend; grid on;

% ─── Plot 3: Pitch angle ─────────────────────────────────────────────────
subplot(2,2,3);
plot(t_vec, theta_deg, 'm-', 'LineWidth', 1.2, 'DisplayName', 'Pitch θ');
hold on;
yline(5,  'r--', 'LineWidth', 1.0);
yline(-5, 'r--', 'LineWidth', 1.0, 'DisplayName', '±5° bound');
xlim([0 40]); ylim([-15 15]);
xlabel('Time (s)'); ylabel('Pitch (deg)');
title(sprintf('Pitch Angle (max = %.1f°)', max_pitch));
legend; grid on;

% ─── Plot 4: Altitude Error ──────────────────────────────────────────────
subplot(2,2,4);
alt_error_full = alt_true - 1.0;
plot(t_vec, alt_error_full, 'r-', 'LineWidth', 1.2);
hold on;
yline( 0.3, 'k--', 'LineWidth', 1.0);
yline(-0.3, 'k--', 'LineWidth', 1.0);
xlim([0 40]); ylim([-1.5 1.5]);
xlabel('Time (s)'); ylabel('Error (m)');
title('Altitude Error vs Setpoint');
legend({'Error', '±0.3 m bound'}); grid on;

% ─── Save ────────────────────────────────────────────────────────────────
saveas(gcf, 'hover_plots_wk3_YOUR_NAME.png');
fprintf('Plot saved: hover_plots_wk3_YOUR_NAME.png\n');
```

---

## Expected Results

| Metric | Target | Typical tuned value |
|---|---|---|
| Altitude RMSE | < 0.3 m | 0.05 – 0.15 m |
| Max roll during hover | < 5° | 1° – 3° under 0.22 N wind |
| Max pitch during hover | < 5° | 2° – 5° (wind acts in body X → pitch response) |
| Takeoff time | 5–7 s | Depends on ramp rate |

---

## Troubleshooting

| Problem | Likely cause | Fix |
|---|---|---|
| RMSE > 0.3 m, drone oscillates around setpoint | Outer Kp too high or Kd too low | Reduce outer Kp by 30%; increase Kd |
| Drone never reaches 1 m altitude | Outer Kp too low or feedforward wrong | Check `Fz_feedforward = m*g = 4.905 N` |
| Roll angle exceeds 5° | Wind force too high or inner Kp too low | Increase inner Roll Kp |
| Drone drifts laterally but altitude is fine | Expected — no horizontal position loop yet | This is OK for Week 3 |
| Stateflow mode stays in TAKEOFF | Altitude setpoint transition condition not met | Switch to time-based guard: `after(6, sec)` |

---

## Checklist

- [ ] Wind model active during RMSE test
- [ ] Simulation run for 40 seconds
- [ ] Altitude RMSE computed over 30 s hover window
- [ ] RMSE < 0.3 m (R1 PASS)
- [ ] Max roll < 5°, max pitch < 5° (R2 PASS)
- [ ] MATLAB command window screenshot saved as `rmse_result_wk3_YOUR_NAME.png`
- [ ] 4-panel hover plot saved as `hover_plots_wk3_YOUR_NAME.png`
- [ ] All subplots have titles, axis labels, legends, and grids
