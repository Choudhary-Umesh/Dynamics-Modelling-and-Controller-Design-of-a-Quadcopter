# Quadrotor Dynamics Modeling & Control Design

MATLAB/Simulink model of a quadrotor UAV — full 6-DOF nonlinear plant, first-order motor/ESC
actuator dynamics, and a cascaded PID flight controller (altitude, attitude, horizontal velocity).

The model is parameterised two ways: a **Quan Quan** parameter set (Sunnysky A2212 / APC 10x4.5,
1.023 kg airframe) used as the primary configuration, and a **Ferry** parameter set (0.284 kg
airframe with an explicit DC-motor electrical model) kept for comparison.

---

## Contents

| File | Purpose |
|---|---|
| `quad_dynamics.slx` | Main Simulink model: 6-DOF plant + cascaded PID controller |
| `Quadcopter_Plant.slx` | Standalone plant-only model (no controller) |
| `Quad_Constant.m` | **Primary** parameter script — Quan Quan Table 11.3 / 11.5 / 11.6 |
| `Quad_Constant_Ferry.m` | Alternate parameter script — Ferry thesis, DC-motor model |
| `Thrust_Test.m` | Open-loop pure-thrust climb, 3D trajectory plot |
| `Roll_Test.m` | Open-loop roll imbalance (left/right pair), 3D trajectory plot |
| `Pitch_Test.m` | Open-loop pitch imbalance (front/rear pair), 3D trajectory plot |
| `Yaw_Test.m` | Open-loop yaw imbalance (CW/CCW pairs), helix trajectory plot |
| `Altitude_ferry.mat` | Logged altitude-response data |
| `Motor Dynamics and control/` | Brushless DC motor sub-study (see below) |

### `Motor Dynamics and control/`

| File | Purpose |
|---|---|
| `Motor_Dynamics.slx` | Electrical + mechanical DC motor model (R, L, K_t, K_emf, J_m, b_m) |
| `Motor_dynamics_init.m` | Loads Ferry Table 2.9 motor parameters |
| `Motor_Dynamics_TF.m` | Runs the model and formats `Voltage` / `Velocity` for System Identification |
| `system_Identification.sid` | Saved System Identification Toolbox session (motor transfer function fit) |

---

## Requirements

- MATLAB R2025b or newer (models were saved in R2025b)
- Simulink
- System Identification Toolbox — only for the motor sub-study

---

## Getting started

Open MATLAB, `cd` into the repository folder, then:

```matlab
Quad_Constant      % load parameters into the base workspace
Thrust_Test        % run a simulation and plot the 3D trajectory
```

Run `Quad_Constant` (or `Quad_Constant_Ferry`) **before** any of the `*_Test.m` scripts — the test
scripts use `clearvars -except ...` and rely on the constants already being in the workspace.

Each test script sets `StopTime`, pushes the four rotor speeds `Omega1_val … Omega4_val` into the
model via `Simulink.SimulationInput`, runs the sim, and plots the resulting path.

### Motor sub-study

```matlab
cd 'Motor Dynamics and control'
Motor_dynamics_init
Motor_Dynamics_TF        % prints the sample time to use in systemIdentification
systemIdentification     % load system_Identification.sid
```

---

## Model overview

### Plant

Standard 6-DOF rigid-body quadrotor in a **NED** frame (negative Z is up, so a 3 m hover is
`h_ref = -3`). Motors are in an X configuration at 45° from the body axes, arm length `l`.

Rotor speeds map to forces and moments through the thrust factor `b` and the drag/torque factor `d`:

```
U1 = b(Ω1² + Ω2² + Ω3² + Ω4²)      total thrust
U2 = b·l(Ω2² + Ω3² − Ω1² − Ω4²)     roll
U3 = b·l(Ω3² + Ω4² − Ω1² − Ω2²)     pitch
U4 = d(Ω2² + Ω4² − Ω1² − Ω3²)       yaw
```

Quan Quan's dynamometer coefficients are published per RPM², so `Quad_Constant.m` converts them to
rad/s² using the factor `(30/π)²` before they reach the model.

### Actuator

The primary configuration uses a first-order motor/ESC lag:

- `Tm  = 0.076 s` — motor/ESC time constant
- `CR  = 80.584` — throttle-to-speed slope (RPM/%)
- `w_b = 976.2` — base speed at zero throttle (RPM)

`Quad_Constant_Ferry.m` instead exposes the full electrical model (`R_m`, `L_m`, `Kt`, `Kemf`, `b_m`)
for use with `Motor_Dynamics.slx`.

---

## Controller design

The controller is a **cascaded PID structure** built around the timescale separation of quadrotor
dynamics: rotational modes are roughly an order of magnitude faster than translational ones, so
position is regulated by commanding attitude, and attitude is regulated by commanding rotor speeds.

Inside `quad_dynamics.slx`, the `Controller` subsystem contains four nested loops plus a control
allocation stage:

```
        ┌───────────────┐   ┌────────────────┐   ┌────────────────┐   ┌────────────────┐
X_d,Y_d │   Position    │ v │   Velocity     │φ_d│ Attitude Angle │p_d│ Attitude Rate  │ U2,U3,U4
───────▶│   control     ├──▶│   control      ├──▶│   control      ├──▶│   control      ├────┐
        │    (P)        │ d │  (PID → tilt)  │θ_d│      (P)       │q_d│    (PD)        │    │
        └───────────────┘   └────────────────┘   └────────────────┘   └────────────────┘    │
                                                                                            ▼
Z_d ───▶┌───────────────┐ U1                                          ┌──────────────────────┐
        │   Altitude    ├────────────────────────────────────────────▶│ Control allocation   │
        │  control (PID)│                                             │ (mixer + signed √)   │
        └───────────────┘                                             └──────────┬───────────┘
                                                                                 │ Ω1..Ω4
                                                                                 ▼
```

Loop rates run slowest at the outside (position) and fastest at the inside (angular rate).

### 1. Position loop (outer)

Proportional control on horizontal position error, producing a velocity setpoint:

```
v_d = Kp_pos · (p_d − p)
```

Kept proportional-only — integral action here fights the integrator already present in the velocity
loop and invites windup during large step commands.

### 2. Velocity loop

Full PID on horizontal velocity error, producing the tilt angles that generate the required
horizontal acceleration:

```
θ_d =  (1/g)·[ Kp_Vx·e_vx + Ki_Vx·∫e_vx + Kd_Vx·ė_vx ]
φ_d = −(1/g)·[ Kp_Vy·e_vy + Ki_Vy·∫e_vy + Kd_Vy·ė_vy ]
```

The `1/g` factor comes from the small-angle hover linearisation `ẍ ≈ g·θ`, `ÿ ≈ −g·φ`. Tilt
setpoints are saturated to keep the small-angle assumption valid and prevent the vehicle from
commanding attitudes that trade away too much vertical thrust.

### 3. Attitude loops (angle + rate)

Attitude is deliberately **split into two cascaded loops** rather than a single PID on angle. The
outer loop is proportional on angle error and produces an angular-rate setpoint; the inner loop is
PD on angular-rate error and produces the body moments:

```
ω_d = Kp_att · (Θ_d − Θ)                        ← Attitude Angle control (P)
U   = K_wp · (ω_d − ω)  −  K_wd · ω̇             ← Attitude Rate control (PD)
```

This structure gives independent authority over overshoot and damping, and lets the rate loop reject
disturbances (wind gusts, thrust asymmetry) without waiting for an angle error to build up. The
derivative term acts on measured rate rather than on the error signal, which avoids derivative kick
when the setpoint steps.

### 4. Altitude loop

PID on altitude error, run in parallel with the horizontal chain, producing total thrust `U1`:

```
U1 = [ m·g + Kp_z·e_z + Ki_z·∫e_z + Kd_z·ė_z ] / (cos φ · cos θ)
```

The gravity feedforward `m·g` means the integrator does not have to wind up to hold hover, and the
`1/(cos φ · cos θ)` term compensates the thrust lost to tilt so that altitude does not sag during
aggressive horizontal manoeuvres.

### 5. Control allocation

The four virtual controls `[U1, U2, U3, U4]` (thrust, roll, pitch, yaw moments) are mapped back to
individual rotor speeds by inverting the allocation matrix:

```
⎡U1⎤   ⎡  b     b     b     b ⎤ ⎡Ω1²⎤
⎢U2⎥ = ⎢ −bl    bl    bl   −bl⎥ ⎢Ω2²⎥          →   Ω² = M⁻¹ · U
⎢U3⎥   ⎢ −bl   −bl    bl    bl⎥ ⎢Ω3²⎥              Ω  = signum(Ω²)·√|Ω²|
⎣U4⎦   ⎣ −d     d    −d     d ⎦ ⎣Ω4²⎦
```

Because the inversion yields squared speeds, the model uses **signed square-root** blocks so that a
transient negative `Ω²` (possible when the loops demand more differential authority than the thrust
budget allows) degrades gracefully instead of producing a complex result and halting the sim. Rotor
speeds are then saturated to the physical range of the Sunnysky A2212 / APC 10x4.5 combination.

### Gain set

Gains follow Quan Quan Tables 11.5 and 11.6:

| Loop | Gains |
|---|---|
| Altitude (Z) | `Kp_z = 4.0`, `Ki_z = 2.0`, `Kd_z = 4.0` |
| Roll — angle | `Kp_roll = 2.0` |
| Roll — rate | `K_wp_roll = 2.0`, `K_wd_roll = 0.8` |
| Pitch — angle | `Kp_pitch = 2.0` |
| Pitch — rate | `K_wp_pitch = 2.0`, `K_wd_pitch = 0.8` |
| Yaw — angle | `Kp_yaw = 3.0` |
| Yaw — rate | `K_wp_yaw = 3.0`, `K_wd_yaw = 2.5` |
| Horizontal velocity (Vx, Vy) | `Kp = 0.5`, `Ki = 0.05`, `Kd = 0.2` |

Yaw carries noticeably higher gains than roll and pitch. That is a consequence of the inertia and
actuation asymmetry: `Izz` is about twice `Ixx`/`Iyy`, and yaw authority comes from the rotor drag
coefficient `d` rather than the thrust coefficient `b` — roughly two orders of magnitude less
control authority, which the gains have to make up for.

All gains are defined in `Quad_Constant.m` and pushed to the model through the base workspace, so
retuning means editing one script rather than opening subsystems.

### Tuning approach

Loops are closed from the inside out — angular rate first, then attitude angle, then velocity, then
position — with each inner loop verified before the loop enclosing it is closed. Open-loop plant
behaviour is characterised first via `Thrust_Test.m`, `Roll_Test.m`, `Pitch_Test.m` and `Yaw_Test.m`,
which excite each axis independently so the plant response can be checked against the expected
trajectory before any feedback is wrapped around it.

---

## Key parameters (primary configuration)

| Symbol | Value | Description |
|---|---|---|
| `m` | 1.023 kg | Airframe mass |
| `l` | 0.2223 m | Motor-to-CoM distance |
| `Ixx`, `Iyy` | 0.0095 kg·m² | Roll/pitch inertia (bifilar pendulum) |
| `Izz` | 0.0186 kg·m² | Yaw inertia |
| `Jr` | 6.5e-7 kg·m² | Rotor inertia |
| `cT` | 1.4865e-7 N/RPM² | Thrust coefficient |
| `cM` | 2.925e-9 N·m/RPM² | Torque coefficient |
| `Ax`, `Ay`, `Az` | 0 | Translational drag (zero for the hover baseline) |

Translational drag is deliberately zero in the baseline hover/low-speed case. The Ferry
configuration sets all three to 0.1 kg/s.

---

## References

1. Quan Quan, *Introduction to Multicopter Design and Control*, Springer — Tables 11.3, 11.5, 11.6.
2. Ferry, N., *Quadcopter Plant Model and Control System Development with MATLAB/Simulink
   Implementation* — Tables 2.3, 2.9; Figures 2.8, 2.9, 2.31–2.34; Table 4.3.

Figure numbers in the test-script titles (2.31–2.34) refer to the corresponding figures in Ferry.

---

## Notes

- Simulink build caches (`slprj/`, `*.slxc`) are excluded via `.gitignore` — MATLAB regenerates them.
- Reference PDFs are not included in this repository; see the citations above.
