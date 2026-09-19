# PMSM sensorless control benches

Common plant: AFE + DC link + inverter + PMSM, 690 V grid through a 1.6 MVA emulator
transformer (`grid_emulator.m`), machine data from the library script
`n_sys_generic_1M5W_torque_curve` (six paralleled systems, `Pnom = 250 kW` per inverter,
external inertia `5*Jm`, inverter filter `LFi = 230 µH`), speed-controlled operation at
`rpm_sim = 17.8` with MTPA and field-weakening limits. The inverter shares the AFE switching
frequency: `2 × 2.5 kHz` for the SVPWM benches, `6 × 2.5 kHz` for the MPC benches (the MPC
runs at six times the maximum PWM frequency), Simscape step `ts/100`.

Every bench has the same file set:

- `init_model.m` — live-script style initialization: switching frequencies, grid emulator,
  AFE settings (DC-link control, resonant PI, DDSRF-PLL, FHT, LVRT reactive-current logic),
  inverter settings (Luenberger position/speed observer, rotor-speed observer with load
  estimator, speed and current PI gains, MPC gains, back-EMF observer gains, EKF), device data
  (`infineon_FF1200R17IP5` IGBT, `danfoss_SKM1700MB20R4S2I4` MOSFET, ideal switch),
  heatsink, gate drivers, lithium-ion battery, C-Caller settings. Flags
  `use_observer_from_simulink/ccaller_*` and `use_current_controller_from_simulink/ccaller_*`
  choose the implementation per module.
- `grid_emulator.m` — positive/negative sequence set points and transformer data of the grid
  emulator.
- `kalman_psm.m` — 6-state EKF of the PMSM in the stationary frame (currents, back-EMF
  components, speed, position): `A*_tilde_ekf`, `B_tilde_ekf`, `C_ekf`, `Qkalman`, `Rkalman`.
- `plotting_results_inv.m` — grid currents/voltages, inverter output current, MOSFET Q1/Q2
  losses and temperatures, device current/voltage, DC-link voltage; EPS output.
- `power_losses_calculus.m`, `spectrum.m` — loss balance and FFT of the output current.

## Benches

| Folder | Model | Modulator / current control | Position and speed estimation |
|---|---|---|---|
| `svpwm_bemf_obs` | `psm_sv_bemf_ctrl.slx` | Space-vector PWM, dq PI | Back-EMF observer (`emf_fb_p`, `emf_p`) + Luenberger speed observer |
| `svpwm_ekf_bemf_obs` | `psm_sv_ekf_bemf_ctrl.slx` | Space-vector PWM, dq PI | EKF (`kalman_psm.m`) on the back-EMF model |
| `svpwm_nonlinear_obs` | `psm_svpwm_nonlinear_ctrl.slx` | Space-vector PWM, dq PI | Nonlinear observer |
| `mpc_bemf_obs` | `psm_mpc_bemf_ctrl.slx` | Model predictive current control at `6 × 2.5 kHz` | Back-EMF observer |
| `mpc_ekf_bemf_obs` | `psm_mpc_ekf_bemf_ctrl.slx` | Model predictive current control at `6 × 2.5 kHz` | EKF on the back-EMF model |

The init scripts differ only in the model name, the switching frequency and the observer
gains recomputed for it, so results are directly comparable across benches.
