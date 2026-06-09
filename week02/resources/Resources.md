# Week 2 — Resources & References
### Autonomous Aerial Systems · CSOT 2026 · AeroClub, IIT Delhi

---

## Primary Videos

| # | Title | Link |
|---|---|---|
| Video 1 | Modelling Environments & 3D Simulation | https://youtu.be/yPA0cmXuYB4?si=_qnmTntOfwHZ89pp |
| Video 2 | Modelling Virtual Sensors | https://youtu.be/di04jfUNRxE?si=skb1refyexA9RCzY |

---

## MathWorks Official Documentation

| Resource | What it covers | Link |
|---|---|---|
| IMU Sensor Simulation | imuSensor object, gyroparams, accelparams — the exact noise model used in this week | https://www.mathworks.com/help/fusion/ug/introduction-to-simulating-imu-measurements.html |
| IMU Sensor Fusion with Simulink | End-to-end example of IMU + AHRS in Simulink — closely related to what you build | https://www.mathworks.com/help/nav/ug/imu-sensor-fusion-with-simulink.html |
| Sensor Models Overview | Full list of MathWorks sensor simulation blocks | https://www.mathworks.com/help/nav/sensor-models.html |
| Band-Limited White Noise block | Explains the noise power parameter and how to set it correctly | https://www.mathworks.com/help/simulink/slref/bandlimitedwhitenoise.html |
| Zero-Order Hold block | How ZOH works for rate conversion | https://www.mathworks.com/help/simulink/slref/zeroorderhold.html |

---

## Reference Papers (Optional — Not Required)

These are real research papers. Reading them is not required to complete Week 2 but they give depth on what you are implementing.

### 1 — IMU Noise Model
Title: A Primer on IMU Error and Noise Analysis
Why read it: Explains ConstantBias, NoiseDensity, BiasInstability, and RandomWalk in plain language with diagrams. Directly maps to the parameters you set this week.
Source: MathWorks Documentation / IEEE Tutorials

### 2 — Multi-Sensor UAV Localisation
Title: IMU/Magnetometer/Barometer/Mass-Flow Sensor Integrated Indoor Quadrotor UAV Localization with Robust Velocity Updates
Authors: Remote Sensing, MDPI, 2019
Why read it: Shows exactly how the three sensors you model this week (IMU, barometer, magnetometer) are used in a real quadrotor localisation system. Section 3 (sensor models) is directly relevant.
Link: https://www.mdpi.com/2072-4292/11/7/838
Pages to read: Section 2 (System Overview) and Section 3 (Sensor Models) — approximately 6 pages

### 3 — Wind Disturbance Modelling
Title: Simulation of Wind Effect on a Quadrotor Flight
Why read it: Derives the wind force expression used in Task 2 from aerodynamic first principles. Shows how crosswind translates to body-frame force and torque.
Link: https://www.researchgate.net/publication/283649477

### 4 — Quadcopter with Sensor Model in Simulink (Practical)
Title: Modelling and Design of Flight Control for Quadcopter in Ballistic Airdrop Mission Under Wind Perturbation
Why read it: Builds almost exactly the same stack you are building — 6-DOF dynamics + IMU + barometer in MATLAB/Simulink. Very practical reference.
Link: https://www.researchgate.net/publication/354306331

---

## Noise Parameter Reference — Real MEMS IMUs

These are real sensor datasheets for context. Your simulation parameters are based on typical values from this class of sensor.

| Sensor | Type | Accel Noise Density | Gyro Noise Density |
|---|---|---|---|
| MPU-6050 | MEMS (budget) | 0.004 m/s^2/sqrtHz | 0.0005 rad/s/sqrtHz |
| ICM-42688 | MEMS (mid-range) | 0.0025 m/s^2/sqrtHz | 0.00025 rad/s/sqrtHz |
| ADIS16470 | MEMS (industrial) | 0.00049 m/s^2/sqrtHz | 0.00009 rad/s/sqrtHz |

Your simulation uses values close to ICM-42688 — a sensor used on many commercial drones.

---

## Noise Power Conversion 

This is the most common source of errors in Week 2:

  NoiseDensity [units/sqrtHz] × sqrt(SampleRate [Hz]) = sigma [units]
  sigma^2 = Noise Power (what you enter in Band-Limited White Noise block)

Example (Accelerometer at 200 Hz):
  NoiseDensity = 0.003 m/s^2/sqrtHz
  sigma = 0.003 × sqrt(200) = 0.003 × 14.14 = 0.0424 m/s^2
  Noise Power = 0.0424^2 = 0.0018 m/s^4   ← enter THIS in Simulink

Do not enter 0.003 directly. The block will produce noise that is 14x too small.

---

## Getting Help

GitHub Issues: tag week2-question

AeroClub contact: CSOT 2026 communication channel
