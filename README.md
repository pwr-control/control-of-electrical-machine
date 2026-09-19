# control-of-electrical-machine

Simulink/Simscape studies on the control of electrical machines. The current content is a
set of sensorless control architectures for a permanent-magnet synchronous machine (PMSM,
`psm_sensorless_ctrl/`), all built on the same plant so that modulator and observer choices
can be compared: space-vector PWM with dq PI or model predictive current control on the inverter
side, and back-EMF observer, extended Kalman filter or nonlinear observer for rotor position
and speed. Induction-machine studies are planned for the same structure.

## Prerequisites

- MATLAB with Simulink, Simscape and Simscape Electrical.
- The companion [library](https://github.com/pwr-control/library) repository on the MATLAB
  path **with subfolders**: the models use its Simscape machine and device components, the
  C-Caller control code (current controllers, observers, PLL, modulators) and the machine
  data scripts (`n_sys_generic_1M5W_torque_curve`, `testroom_eq_psm_690V`, device data
  `infineon_FF1200R17IP5`, `danfoss_SKM1700MB20R4S2I4`).

## How to use

Each subfolder of `psm_sensorless_ctrl/` is a complete bench: run `init_model.m`, simulate
the `.slx`, then use `plotting_results_inv.m`, `power_losses_calculus.m` and `spectrum.m`
on the logged results.

## Repository layout

| Folder | Content |
|---|---|
| [`psm_sensorless_ctrl`](psm_sensorless_ctrl) | Five PMSM sensorless benches: SVPWM or MPC × back-EMF observer, EKF or nonlinear observer |
| [`literature`](literature) | Reference material (predictive control in power electronics) |

Each folder has its own README.
