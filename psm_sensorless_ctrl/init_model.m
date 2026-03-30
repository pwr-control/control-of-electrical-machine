%[text] ## Settings for simulink model initialization and data analysis
close all
clear all
clc
beep off
pm_addunit('percent', 0.01, '1');
options = bodeoptions;
options.FreqUnits = 'Hz';
% simlength = 3.75;
simlength = 2;
transmission_delay = 125e-6*2;

% model = 'psm_sv_bemf_ctrl';
model = 'psm_sv_ekf_bemf_ctrl';
% model = 'psm_svpwm_nonlinear_ctrl';
% model = 'psm_mpc_bemf_ctrl';
% model = 'psm_mpc_ekf_bemf_ctrl';


load_step_time = 0;
time_start_motor_control = 0.025;
%[text] #### local time allignment to master time
kp_align = 0.6;
ki_align = 0.1;
lim_up_align = 0.2;
lim_down_align = -0.2;
%[text] ### Enable one/two modules
number_of_modules = 1;
enable_two_modules = number_of_modules;
%[text] ### MOTOR Selection from Library
n_sys = 6;
run('n_sys_generic_1M5W_pmsm'); %[output:98950895] %[output:9dd00217]
run('n_sys_generic_1M5W_torque_curve');

% n_sys = 1;
% run('testroom_eq_psm_690V');
% run('testroom_torque_curve_690V');

b = tau_bez/omega_m_bez;
external_motor_inertia = 5*Jm;
% external_motor_inertia = 1;

%% inverter filter
% LFi = 40e-6;
% RLFi = 5e-3
%% inverter filter
LFi = 230e-6;
LFi_0 = 20e-6;
RLFi = 5e-3;
%[text] ### Settings for speed control or wind application
use_torque_curve = 1; % for wind application
use_speed_control = 1-use_torque_curve; %
use_mtpa = 1; %
use_psm_encoder = 0; % 
use_load_estimator = 0; %
use_estimator_from_mb = 0; %mb model based
use_motor_speed_control_mode = 0; 
use_advanced_pll = 0; % advanced pll should compensate second harmonic
use_dq_pll_ccaller_mod1 = 0; % only module 1
use_dq_pll_ccaller_mod2 = 0; % only module 1
%[text] ### Settings for CCcaller versus Simulink
use_observer_from_simulink_module_1 = 1;
use_observer_from_ccaller_module_1 = 0;
use_observer_from_simulink_module_2 = 1;
use_observer_from_ccaller_module_2 = 0;

use_current_controller_from_simulink_module_1 = 0;
use_current_controller_from_ccaller_module_1 = 1;
use_current_controller_from_simulink_module_2 = 1;
use_current_controller_from_ccaller_module_2 = 0;

use_moving_average_from_ccaller_mod1 = 0;
use_moving_average_from_ccaller_mod2 = 0;
mavarage_filter_frequency_base_order = 2; % 2 means 100Hz, 1 means 50Hz
dmavg_filter_enable_time = 0.025;
%%
%[text] ## Grid Emulator Settings
grid_emulator;
%%
%[text] ## AFE Settings and Initialization
%[text] ### Switching frequencies, sampling time and deadtime
% fPWM_AFE = 6*2.5e3; % in case of mpc controller run 6 times the maximum pwm
fPWM_AFE = 2*2.5e3; % in case of sv controller run 2 times the maximum pwm
% fPWM_AFE = 2*4e3; % in case of sv controller run 2 times the maximum pwm
tPWM_AFE = 1/fPWM_AFE;

dead_time_AFE = 3e-6;
delay_pwm = 0;
delayAFE_modB=2*pi*fPWM_AFE*delay_pwm; 
delayAFE_modA=0;
delayAFE_modC=0;

ts_afe = 1/fPWM_AFE; % Sampling time of the control AFE as well as INVERTER
tc = ts_afe/100;

s=tf('s');
z_afe=tf('z',ts_afe);

% t_misura = simlength - 0.025;
t_misura = 1/omega_bez*2*pi*5;
Nc = ceil(t_misura/tc);
Ns_afe = ceil(t_misura/ts_afe);

time_gain_afe_module_1 = 1.0;
time_gain_inv_module_1 = 1.0;
time_gain_afe_module_2 = 1.0;
time_gain_inv_module_2 = 1.0;
wnp = 0;
white_noise_power_afe_mod1 = wnp;
white_noise_power_inv_mod1 = wnp;
white_noise_power_afe_mod2 = wnp;
white_noise_power_inv_mod2 = wnp;

trgo_th_generator = 0.025;

afe_pwm_phase_shift_mod1 = 0;
white_noise_power_afe_pwm_phase_shift_mod1 = 0.0;
inv_pwm_phase_shift_mod1 = 0;
white_noise_power_inv_pwm_phase_shift_mod1 = 0.0;

afe_pwm_phase_shift_mod2 = 0;
white_noise_power_afe_pwm_phase_shift_mod2 = 0.0;
inv_pwm_phase_shift_mod2 = 0;
white_noise_power_inv_pwm_phase_shift_mod2 = 0.0;
%[text] ### 
%[text] #### Reactive current limits for grid support

i_grid_pos_eta_lim = 1;
i_grid_neg_xi_lim = 0.5;
i_grid_neg_eta_lim = 0.5;

%[text] #### Reactive current Limits - Red. Dyn. grid support
i_grid_pos_eta_red_lim = 0.1;
i_grid_neg_eta_red_lim = 0.1;
i_grid_neg_xi_red_lim = 0.1;

%[text] #### Grid voltage derivate implemented with double integrator observer
Aso = [0 1; 0 0];
Asod = eye(2)+Aso*ts_afe;
Cso = [1 0];
omega_rso = 2*pi*50;
p2place = [-1 -4]*omega_rso;
p2placed = exp(p2place*ts_afe);
Kd = (acker(Asod',Cso',p2placed))';
l1 = Kd(2) %[output:8bf67124]
l2 = Kd(1) %[output:45e82fc7]

%[text] ### Current sensor endscale, and quantization
adc_quantization = 1/2^11;
Imax_adc = 1049.835;
CurrentQuantization = Imax_adc/2^11;
%%
%[text] ### Voltage sensor endscale, and quantization
Umax_adc = 1500;
VoltageQuantization = Umax_adc/2^11;
%%
%[text] ### DClink, and dclink-brake parameters
Vdc_ref = 1070; % DClink voltage reference
Rprecharge = 1; % Resistance of the DClink pre-charge circuit
Pload = 250e3;
Rbrake = 4;
CFi = 900e-6*8;

%[text] #### 
%[text] #### DClink Lstray model
Lstray_dclink = 100e-9;
RLstray_dclink = 10e-3;
C_HF_Lstray_dclink = 15e-6;
R_HF_Lstray_dclink = 22000;
Z_HF_Lstray_dclink = 1/s/C_HF_Lstray_dclink + R_HF_Lstray_dclink;
Z_LF_Lstray_dclink = s*Lstray_dclink + RLstray_dclink;
Z_Lstray_dclink = Z_HF_Lstray_dclink*Z_LF_Lstray_dclink/(Z_HF_Lstray_dclink+Z_LF_Lstray_dclink);
ZCFi = 7/s/CFi;
sys_dclink = minreal(ZCFi/(ZCFi+Z_Lstray_dclink));
% figure; bode(sys_dclink,Z_Lstray_dclink,options); grid on
%[text] ### LCL switching filter
LFu1_AFE = 0.5e-3;
RLFu1_AFE = 157*0.05*LFu1_AFE;
LFu1_AFE_0 = LFu1_AFE;
RLFu1_AFE_0 = RLFu1_AFE/3;
CFu = (100e-6*2);
RCFu = (50e-3);
%%
%[text] ### DClink voltage control parameters
Vdc_nom = Vdc_bez;
Vdc_norm_ref = Vdc_ref/Vdc_nom;
kp_vs = 0.85;
ki_vs = 35;
%%
%[text] ### AFE current control parameters
%[text] #### Resonant PI
kp_afe = 0.6;
ki_afe = 45;
delta = 0.015;
res = s/(s^2 + 2*delta*omega_grid_nom*s + (omega_grid_nom)^2);

Ares = [0 1; -omega_grid_nom^2 -2*delta*omega_grid_nom];
Bres = [0; 1];
Cres = [0 1];
Aresd = eye(2) + Ares*ts_afe;
Bresd = Bres*ts_afe;
Cresd = Cres;
%%
%[text] ### Grid Normalization Factors
Vgrid_phase_normalization_factor = Vphase2*sqrt(2);
pll_i1 = 80;
pll_p = 1;
pll_p_frt = 0.2;
Vmax_ff = 1.1;
Igrid_phase_normalization_factor = 250e3/Vphase2/3/0.9*sqrt(2);
ixi_pos_ref_lim = 1.6;
ieta_pos_ref_lim = 1.0;
ieta_neg_ref_lim = 0.5;
%%
%[text] ### Double Integrator Observer for PLL
Arso = [0 1; 0 0];
Crso = [1 0];
omega_rso = 2*pi*50;
polesrso = [-1 -4]*omega_rso;
Lrso = acker(Arso',Crso',polesrso)';
Adrso = eye(2) + Arso*ts_afe;
polesdrso = exp(ts_afe*polesrso);
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:3873a036]

%[text] ### PLL DDSRF
pll_i1_ddsrt = pll_i1/2;
pll_p_ddsrt = pll_p/2;
omega_f = 2*pi*50;
ddsrf_f = omega_f/(s+omega_f);
ddsrf_fd = c2d(ddsrf_f,ts_afe);
%%
%[text] ### First Harmonic Tracker for Ugrid cleaning
omega0 = 2*pi*50;
Afht = [0 1; -omega0^2 -0.05*omega0] % impianto nel continuo %[output:2f8df18e]
Cfht = [1 0];
poles_fht = [-1 -4]*omega0;
Lfht = acker(Afht',Cfht',poles_fht)' % guadagni osservatore nel continuo %[output:45ef9943]
Ad_fht = eye(2) + Afht*ts_afe % impianto nel discreto %[output:20f05bf9]
polesd_fht = exp(ts_afe*poles_fht);
Ld_fht = Lfht*ts_afe % guadagni osservatore nel discreto %[output:0445503e]
%[text] ### Reactive current control gains
kp_rc_grid = 0.35;
ki_rc_grid = 35;
kp_rc_pos_grid = kp_rc_grid;
ki_rc_pos_grid = ki_rc_grid;
kp_rc_neg_grid = kp_rc_grid;
ki_rc_neg_grid = ki_rc_grid;
%%
%[text] ### Settings for First Order Low Pass Filters
%[text] #### LPF 50Hz in state space (for initialization)
fcut = 50;
fof = 1/(s/(2*pi*fcut)+1);
[nfof, dfof] = tfdata(fof,'v');
[nfofd, dfofd]=tfdata(c2d(fof,ts_afe),'v');
fof_z = tf(nfofd,dfofd,ts_afe,'Variable','z');
[A,B,C,D] = tf2ss(nfofd,dfofd);
LVRT_flt_ss = ss(A,B,C,D,ts_afe);
[A,B,C,D] = tf2ss(nfof,dfof);
LVRT_flt_ss_c = ss(A,B,C,D);
%[text] #### LPF 161Hz
fcut_161Hz_flt = 161;
g0_161Hz = fcut_161Hz_flt * ts_afe * 2*pi;
g1_161Hz = 1 - g0_161Hz;
%%
%[text] #### LPF 500Hz
fcut_500Hz_flt = 500;
g0_500Hz = fcut_500Hz_flt * ts_afe * 2*pi;
g1_500Hz = 1 - g0_500Hz;
%%
%[text] #### LPF 75Hz
fcut_75Hz_flt = 75;
g0_75Hz = fcut_75Hz_flt * ts_afe * 2*pi;
g1_75Hz = 1 - g0_75Hz;
%%
%[text] #### LPF 50Hz
fcut_50Hz_flt = 50;
g0_50Hz = fcut_50Hz_flt * ts_afe * 2*pi;
g1_50Hz = 1 - g0_50Hz;
%%
%[text] #### LPF 10Hz
fcut_10Hz_flt = 10;
g0_10Hz = fcut_10Hz_flt * ts_afe * 2*pi;
g1_10Hz = 1 - g0_10Hz;
%%
%[text] #### LPF 4Hz
fcut_4Hz_flt = 4;
g0_4Hz = fcut_4Hz_flt * ts_afe * 2*pi;
g1_4Hz = 1 - g0_4Hz;
%%
%[text] #### LPF 1Hz
fcut_1Hz_flt = 1;
g0_1Hz = fcut_1Hz_flt * ts_afe * 2*pi;
g1_1Hz = 1 - g0_1Hz;
%%
%[text] #### LPF 0.2Hz
fcut_0Hz2_flt = 0.2;
g0_0Hz2 = fcut_0Hz2_flt * ts_afe * 2*pi;
g1_0Hz2 = 1 - g0_0Hz2;
%%
%[text] ### Settings for RMS calculus
rms_perios = 1;
n1 = rms_perios/f_grid/ts_afe;
rms_perios = 10;
n10 = rms_perios/f_grid/ts_afe;
%%
%[text] ### Online time domain sequence calculator
w_grid = 2*pi*f_grid;
apf = (s/w_grid-1)/(s/w_grid+1);
[napfd, dapfd]=tfdata(c2d(apf,ts_afe),'v');
apf_z = tf(napfd,dapfd,ts_afe,'Variable','z');
[A,B,C,D] = tf2ss(napfd,dapfd);
ap_flt_ss = ss(A,B,C,D,ts_afe);
% figure;
% bode(ap_flt_ss,options);
% grid on
%%
%[text] ## INVERTER Settings and Initialization
%[text] ### Mode of operation
motor_torque_mode = 1 - use_motor_speed_control_mode; % system uses torque curve for wind application

%[text] ### Switching frequencies, sampling time and deadtime
fPWM_INV = fPWM_AFE;
% fPWM_INV = 2500;
dead_time_INV = 3e-6;
delayINV_modA = 0;
pwm_out_lim = 1;

ts_inv = 1/fPWM_INV;
t_measure = simlength;
Ns_inv = floor(t_measure/ts_inv);
s=tf('s');
z=tf('z',ts_inv);

%[text] ### Simulation parameters: speed reference, load torque in motor mode
% rpm_sim = 3000;
rpm_sim = 17.8;
% rpm_sim = 15.2;
omega_m_sim = omega_m_bez;
omega_sim = omega_m_sim*number_poles/2;
tau_load_sim = tau_bez/5; %N*m
b_square = 0;

%[text] ### Luenberger Observer
Aso = [1 ts_inv; 0 1];
Cso = [1 0];
% p2place = exp([-10 -50]*ts_inv);
p2place = exp([-40 -160]*ts_inv);
Kobs = (acker(Aso',Cso',p2place))';
kg = Kobs(1) %[output:04a70309]
kw = Kobs(2) %[output:69e958f5]

%[text] ### Rotor speed observer with load estimator
A = [0 1 0; 0 0 -1/Jm_norm; 0 0 0];
Alo = eye(3) + A*ts_inv;
Blo = [0; ts_inv/Jm_norm; 0];
Clo = [1 0 0];
p3place = exp([-1 -5 -25]*125*ts_inv);
Klo = (acker(Alo',Clo',p3place))';
luenberger_l1 = Klo(1) %[output:8e3d2c7e]
luenberger_l2 = Klo(2) %[output:95bd65ef]
luenberger_l3 = Klo(3) %[output:7d37a44a]
omega_flt_fcut = 10;
% phase_compensation_omega = -pi/2-pi/12; % for motor mode
phase_compensation_omega = 0; % for generator mode
%[text] ### Control settings
id_lim = 0.35;
%[text] #### rotor speed control
kp_w = 2;
ki_w = 8;
iq_lim = 1.4;
%[text] #### current control
kp_i = 0.25;
ki_i = 18;
kp_id = kp_i;
ki_id = ki_i;
kp_iq = kp_i;
ki_iq = ki_i;
CTRPIFF_CLIP_RELEASE = 0.001;
%[text] #### Model Predictive Control 
kp_i = 0.25;
ki_i = 18;
%[text] #### 
%[text] #### Field Weakening Control 
k_kalman = 0;
%[text] #### BEMF observer
emf_fb_p = 0.2;
emf_p = emf_fb_p*4/10;

emf_fb_p_ccaller_1 = emf_fb_p;
emf_p_ccaller_1 = emf_fb_p_ccaller_1*4/10;

emf_fb_p_ccaller_2 = emf_fb_p;
emf_p_ccaller_2 = emf_fb_p_ccaller_2*4/10;

% omega_th = 0.25;
omega_th = 0;
%[text] #### EKF BEMF observer
kalman_psm;
%[text] #### Speed obserfer filter LPF 10Hz
fcut_10Hz_flt = 10;
omega_flt_g0 = fcut_10Hz_flt * ts_inv * 2*pi;
omega_flt_g1 = 1 - omega_flt_g0;
%[text] #### Motor Voltage to Udc Scaling
motorc_m_scale = 2/3*Vdc_bez/ubez;
inv_m_scale = motorc_m_scale;
%%
%[text] ## Power semiconductors modelization, IGBT, MOSFET,  and snubber data
%[text] ### HeatSink settings
heatsink_liquid_2kW;
%[text] ### DEVICES settings (IGBT)
igbt.data = 'infineon_FF1200R17IP5';
run(igbt.data);

igbt.inv.Vth = Vth;                                  % [V]
igbt.inv.Vce_sat = Vce_sat;                          % [V]
igbt.inv.Rce_on = Rce_on;                            % [Ohm]
igbt.inv.Vdon_diode = Vdon_diode;                    % [V]
igbt.inv.Rdon_diode = Rdon_diode;                    % [Ohm]
igbt.inv.Eon = Eon;                                  % [J] @ Tj = 125°C
igbt.inv.Eoff = Eoff;                                % [J] @ Tj = 125°C
igbt.inv.Erec = Erec;                                % [J] @ Tj = 125°C
igbt.inv.Voff_sw_losses = Voff_sw_losses;            % [V]
igbt.inv.Ion_sw_losses = Ion_sw_losses;              % [A]
igbt.inv.JunctionTermalMass = JunctionTermalMass;    % [J/K]
igbt.inv.Rtim = Rtim;                                % [K/W]
igbt.inv.Rth_switch_JC = Rth_switch_JC;              % [K/W]
igbt.inv.Rth_switch_CH = Rth_switch_CH;              % [K/W]
igbt.inv.Rth_switch_JH = Rth_switch_JH;              % [K/W]
igbt.inv.Rth_diode_JC = Rth_diode_JC;              % [K/W]
igbt.inv.Rth_diode_CH = Rth_diode_CH;              % [K/W]
igbt.inv.Rth_diode_JH = Rth_diode_JH;              % [K/W]
igbt.inv.Lstray_module = Lstray_module;            % [H]
igbt.inv.Irr = Irr;                                % [A]
igbt.inv.Cies = Cies;                              % [F]
igbt.inv.Csnubber = Csnubber;                      % [F]
igbt.inv.Rsnubber = Rsnubber;                      % [Ohm]
% inv.Csnubber = (inv.Irr)^2*Lstray_module/Vdc_bez^2
% inv.Rsnubber = 1/(inv.Csnubber*fPWM_INV)/5

igbt.afe.Vth = Vth;                                  % [V]
igbt.afe.Vce_sat = Vce_sat;                          % [V]
igbt.afe.Rce_on = Rce_on;                            % [Ohm]
igbt.afe.Vdon_diode = Vdon_diode;                    % [V]
igbt.afe.Rdon_diode = Rdon_diode;                    % [Ohm]
igbt.afe.Eon = Eon;                                  % [J] @ Tj = 125°C
igbt.afe.Eoff = Eoff;                                % [J] @ Tj = 125°C
igbt.afe.Erec = Erec;                                % [J] @ Tj = 125°C
igbt.afe.Voff_sw_losses = Voff_sw_losses;            % [V]
igbt.afe.Ion_sw_losses = Ion_sw_losses;              % [A]
igbt.afe.JunctionTermalMass = JunctionTermalMass;    % [J/K]
igbt.afe.Rtim = Rtim;                                % [K/W]
igbt.afe.Rth_switch_JC = Rth_switch_JC;              % [K/W]
igbt.afe.Rth_switch_CH = Rth_switch_CH;              % [K/W]
igbt.afe.Rth_switch_JH = Rth_switch_JH;              % [K/W]
igbt.afe.Lstray_module = Lstray_module;              % [H]
igbt.afe.Irr = Irr;                                  % [A]
igbt.afe.Cies = Cies;                                % [F]
igbt.afe.Csnubber = Csnubber;                        % [F]
igbt.afe.Rsnubber = Rsnubber;                        % [Ohm]
% afe.Csnubber = (afe.Irr)^2*Lstray_module/Vdc_bez^2
% afe.Rsnubber = 1/(afe.Csnubber*fPWM_AFE)/5

%[text] ### DEVICES settings (MOSFET)
mosfet.data = 'danfoss_SKM1700MB20R4S2I4';
run(mosfet.data);

mosfet.inv.Vth = Vth;                                  % [V]
mosfet.inv.Rds_on = Rds_on;                            % [V]
mosfet.inv.Vdon_diode = Vdon_diode;                    % [V]
mosfet.inv.Rdon_diode = Rdon_diode;                    % [Ohm]
mosfet.inv.Eon = Eon/3*2/3;                            % [J] @ Tj = 125°C
mosfet.inv.Eoff = Eoff/3*2/3;                          % [J] @ Tj = 125°C
mosfet.inv.Erec = Err;                                 % [J] @ Tj = 125°C
mosfet.inv.Voff_sw_losses = Voff_sw_losses*2/3;        % [V]
mosfet.inv.Ion_sw_losses = Ion_sw_losses/3;            % [A]
mosfet.inv.JunctionTermalMass = JunctionTermalMass;    % [J/K]
mosfet.inv.Rtim = Rtim;                                % [K/W]
mosfet.inv.Rth_switch_JC = Rth_mosfet_JC;              % [K/W]
mosfet.inv.Rth_switch_CH = Rth_mosfet_CH;              % [K/W]
mosfet.inv.Rth_switch_JH = Rth_mosfet_JH;              % [K/W]
mosfet.inv.Lstray_module = Lstray_module;              % [H]
mosfet.inv.Irr = Irr;                                  % [A]
mosfet.inv.Csnubber = Csnubber;                        % [F]
mosfet.inv.Rsnubber = Rsnubber;                        % [Ohm]
% inv.Csnubber = (mosfet.inv.Irr)^2*Lstray_module/Vdc_bez^2
% inv.Rsnubber = 1/(mosfet.inv.Csnubber*fPWM_INV)/5

mosfet.afe.Vth = Vth;                                  % [V]
mosfet.afe.Rds_on = Rds_on;                            % [V]
mosfet.afe.Vdon_diode = Vdon_diode;                    % [V]
mosfet.afe.Rdon_diode = Rdon_diode;                    % [Ohm]
mosfet.afe.Eon = Eon/3*2/3;                            % [J] @ Tj = 125°C
mosfet.afe.Eoff = Eoff/3*2/3;                          % [J] @ Tj = 125°C
mosfet.afe.Erec = Err;                                 % [J] @ Tj = 125°C
mosfet.afe.Voff_sw_losses = Voff_sw_losses*2/3;        % [V]
mosfet.afe.Ion_sw_losses = Ion_sw_losses/3;            % [A]
mosfet.afe.JunctionTermalMass = JunctionTermalMass;    % [J/K]
mosfet.afe.Rtim = Rtim;                                % [K/W]
mosfet.afe.Rth_switch_JC = Rth_mosfet_JC;              % [K/W]
mosfet.afe.Rth_switch_CH = Rth_mosfet_CH;              % [K/W]
mosfet.afe.Rth_switch_JH = Rth_mosfet_JH;              % [K/W]
mosfet.afe.Lstray_module = Lstray_module;              % [H]
mosfet.afe.Irr = Irr;                                  % [A]
mosfet.afe.Csnubber = Csnubber;                        % [F]
mosfet.afe.Rsnubber = Rsnubber;                        % [Ohm]
% afe.Csnubber = (mosfet.afe.Irr)^2*Lstray_module/Vdc_bez^2
% afe.Rsnubber = 1/(mosfet.afe.Csnubber*fPWM_AFE)/5
%[text] ### DEVICES settings (Ideal switch)
silicon_high_power_ideal_switch;
ideal_switch.Vth = Vth;                                  % [V]
ideal_switch.Rds_on = Rds_on;                            % [Ohm]
ideal_switch.Vdon_diode = Vdon_diode;                    % [V]
ideal_switch.Vgamma = Vgamma;                            % [V]
ideal_switch.Rdon_diode = Rdon_diode;                    % [Ohm]
ideal_switch.Csnubber = Csnubber;                        % [F]
ideal_switch.Rsnubber = Rsnubber;                        % [Ohm]
ideal_switch.Irr = Irr;                                  % [A]
% ideal_switch.Csnubber = (ideal_switch.Irr)^2*Lstray_module/Vdab2_dc_nom^2
% ideal_switch.Rsnubber = 1/(ideal_switch.Csnubber*fPWM_DAB)/5
%[text] ### GATE drivers settings
positive_voltage_rail = 12;
negative_voltage_rail = 0;
dead_time = 3e-6;
use_deadtime = 1;  
use_deadtime_cmos_based = use_deadtime;
%[text] ### Lithium Ion Battery
ubattery = Vdc_nom;
Pnom = 250e3;

typical_cell_voltage = 3.6;
number_of_cells = floor(ubattery/typical_cell_voltage)-1; % nominal is 100

% stato of charge init
soc_init = 0.85; 

R = 8.3143;
F = 96487;
T = 273.15+40;
Q = 50; %Hr*A

Vbattery_nom = ubattery;
Pbattery_nom = Pnom;
Ibattery_nom = Pbattery_nom/Vbattery_nom;
Rmax = Vbattery_nom^2/(Pbattery_nom*0.1);
Rmin = Vbattery_nom^2/(Pbattery_nom);

E_1 = -1.031;
E0 = 3.485;
E1 = 0.2156;
E2 = 0;
E3 = 0;
Elog = -0.05;
alpha = 35;

R0 = 0.035;
R1 = 0.035;
C1 = 0.5;
M = 125;

q1Kalman = ts_inv^2;
q2Kalman = ts_inv^1;
q3Kalman = 0;
rKalman = 1;

Zmodel = (0:1e-3:1);
ocv_model = E_1*exp(-Zmodel*alpha) + E0 + E1*Zmodel + E2*Zmodel.^2 +...
    E3*Zmodel.^3 + Elog*log(1-Zmodel+ts_inv);
figure;  %[output:49e44520]
plot(Zmodel,ocv_model,'LineWidth',2); %[output:49e44520]
xlabel('state of charge [p.u.]'); %[output:49e44520]
ylabel('open circuit voltage [V]'); %[output:49e44520]
title('open circuit voltage(state of charge)'); %[output:49e44520]
grid on %[output:49e44520]
%[text] ## C-Caller Settings
open_system(model);
Simulink.importExternalCTypes(model,'Names',{'mavgflt_output_t'});
Simulink.importExternalCTypes(model,'Names',{'dsmavgflt_output_t'});
Simulink.importExternalCTypes(model,'Names',{'mavgflts_output_t'});
Simulink.importExternalCTypes(model,'Names',{'bemf_obsv_output_t'});
Simulink.importExternalCTypes(model,'Names',{'bemf_obsv_load_est_output_t'});
Simulink.importExternalCTypes(model,'Names',{'dqvector_pi_output_t'});
Simulink.importExternalCTypes(model,'Names',{'sv_pwm_output_t'});
Simulink.importExternalCTypes(model,'Names',{'global_state_machine_output_t'});
Simulink.importExternalCTypes(model,'Names',{'first_harmonic_tracker_output_t'});
Simulink.importExternalCTypes(model,'Names',{'dqpll_thyr_output_t'});
Simulink.importExternalCTypes(model,'Names',{'dqpll_grid_output_t'});
Simulink.importExternalCTypes(model,'Names',{'rpi_output_t'});

%[text] ## Remove Scopes Opening Automatically
open_scopes = find_system(model, 'BlockType', 'Scope');
for i = 1:length(open_scopes)
    set_param(open_scopes{i}, 'Open', 'off');
end

shh = get(0,'ShowHiddenHandles');
set(0,'ShowHiddenHandles','On');
hscope = findobj(0,'Type','Figure','Tag','SIMULINK_SIMSCOPE_FIGURE');
close(hscope);
set(0,'ShowHiddenHandles',shh);

%[text] ## 

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":35.4}
%---
%[output:98950895]
%   data: {"dataType":"textualVariable","outputData":{"name":"tau_bez","value":"     1.455919822690013e+05"}}
%---
%[output:9dd00217]
%   data: {"dataType":"textualVariable","outputData":{"name":"vg_dclink","value":"     7.897123558639406e+02"}}
%---
%[output:8bf67124]
%   data: {"dataType":"textualVariable","outputData":{"name":"l1","value":"  67.668222262819981"}}
%---
%[output:45e82fc7]
%   data: {"dataType":"textualVariable","outputData":{"name":"l2","value":"   0.283130953403918"}}
%---
%[output:3873a036]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.283130953403918"],["67.668222262819981"]]}}
%---
%[output:2f8df18e]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:45ef9943]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:20f05bf9]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000","0.000200000000000"],["-19.739208802178720","0.996858407346410"]]}}
%---
%[output:0445503e]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.311017672705390"],["54.332172227996914"]]}}
%---
%[output:04a70309]
%   data: {"dataType":"textualVariable","outputData":{"name":"kg","value":"   0.039461503083742"}}
%---
%[output:69e958f5]
%   data: {"dataType":"textualVariable","outputData":{"name":"kw","value":"   1.254711180325163"}}
%---
%[output:8e3d2c7e]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l1","value":"   0.606931756868082"}}
%---
%[output:95bd65ef]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l2","value":"     3.449190983162578e+02"}}
%---
%[output:7d37a44a]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l3","value":"    -4.186041938481557e+02"}}
%---
%[output:49e44520]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAATEAAAC4CAYAAACLrdvMAAAAAXNSR0IArs4c6QAAIABJREFUeF7tXQt0VcXV3kQIQQ3laVMIFuQlbygPkT9\/VaSgroZKtVCEqhgKFhChvAVCkDcKgoSXVnmshSkFkUbUpUUUGyU+edYHAlJBoGIRCFBAhH998\/fEk5tz751zztybw9171sqCJHvmnPvN7C979uzZu9ylS5cuUUDaqVOnaOvWrZSXl0evv\/461ahRg\/r27Uvdu3enWrVqUbly5crkTa+77jrat2+f8WcPzvuE8t4\/osZd2LsJ9W6fZvwZQR7wiy++oHr16gX5FS\/bd+OEbbkgkBjIKycnh\/Lz86lSpUp011130W9+8xtq3LgxJSUllflCigWJfXnsLLWeukV9tmurpdC2CTeW+eeM9wtwUjTBNnYIBILEvv32W3ryySepW7du9LOf\/YySk5Nj94k9jBwLEuNuhWEahMQ8LEbNLpywDQSJac5LmYmZJjGxwv5\/KjkpWrwXLydshcQ0VpdpEoMfDJYYV1+YBTknRdNYZkZFOGErJKaxdEySGKyw7ou2Ev7l6gsTEtNYdD5FhMR8Apho3U2SWMGe44rE0HAaiVNJro2TosV7jjlhGwhLDKeT69evpy5dulBaWskwgyNHjtDGjRvpzjvvpKuvvjrea0E9zySJdV+4lQr2Hlfj5g9qQxkNqpTJZwrCQzkpWrzx5oRtmZLYuXPnaMeOHfTNN9\/QkiVL6J577qHatWuXmO\/9+\/fTiy++SIsXL6Zq1arFey0YJTG7Qz+jfhXKH9ymTD5PUB7KSdHijTknbMuUxI4dO0ZZWVm0ffv2sHNcoUIFGjZsGP3+97+n8uXLx3stGCWxWa\/up1mvfiFW2H9nkZOixXvhcsK2TEnMmliQ2YgRI+iRRx6hhg0bxnu+oz7P1HbS2kpyd+hbgHNStKiLzLAAJ2wDQWKG58\/4cCZITBz6paeFk6IZX5RRBuSEbZmRGKL0Z8+eTQMGDKAqVapQdnY2HTx40HFq0tPT6dFHH6WqVavGey0Y207aI\/S5O\/TFEov9MhYSiz3GZD+RxKnjX\/7yFzpx4oTjk3\/0ox9Rz549L9vTSYnQd15QnBQtDipV4hGcsC0zSyzek+rneX63k\/at5Jhu9WhMt7p+Xidh+nJStHhPGidsA0FiVqgF\/nVqFStWpJYtWxL+LYvml8RkKymWWLzXrZBYnBGHfyzUJ3by5El1QRghFrfeeitNmzbtsvWJVfvjGwpROZUsubA4KVqcVYrV5fpAWGLhJvg\/\/\/kPrVixgn784x9Tjx494r0Oip\/nxxKTU8nw0yYkFrslzQnbQJMYpvjo0aM0Z84cFUNWuXLl2M16hJH9kJgEuAqJlcWiFRIrC9TDPBMkhvCKyZMnX5bXjiTAVUisLNRJSCzOqIdz7MNXtmbNGpVfH0QWKeMrZBcsWECrVq1Sb9+nTx966KGHHP1o2KYuX75c3dfE\/zt27KjSY8PicmpeLTG5Kxl5IXFStDirlPjE4g24k2PfeocWLVrQoEGDSmW3sL8jap3k5ubS+fPnaciQIXThwgW1BUVwLL4PLTCycuVK2rZtG02cOJFSU1Np7dq1VFBQQDNnznSMRfNKYvbkhxLgWnpVCYnFTtM4YRs4nxiICBbXxYsX1ZfOpW+cZI4cOZKGDh1KzZs3VysDpAQrDsSE4iNWg9UH\/xoOCjIyMtSPkSlj1KhRNH36dMe7m15JzJ52B4VAcDop7QcEOClavOedE7aBIbEDBw7QhAkTqGnTpjRmzBjCpfCBAwdS69at1bbQjVP\/9OnTyhKrW7cu3XvvvSXWD6y2oqIiSklJKd6ebt68WVly2I6G5jND59BtJsrJRWuHTl6gzBX\/f42qVuXy9OJ96dG6sPs9rpnhSpk08whwwjYQJIbt36xZs+jMmTOKsEAksMJgISE+rF27dvTggw9q1Z3EOEuXLqX27dsrKyxSXUM8AwkXcWiALBqwzpxqW3qxxCRKP7picrIWoqNhVoITtoEgMfjEsJ0bPXo0NWrUqMRs7tq1S5Vze\/zxx7WtMZDihg0bVIwZkik6WVc7d+6kqVOnKh\/Y8OHDqVmzZmFJ0guJSWhFdKXkpGjR0TArwQnbwJPY7t27lTX2xBNPhA2xwITB+gIJWtlfEZoB6wrO+9AcZYWFhTRu3Dj1hZTY0Qr0eiExCa2IrpScFC06GmYlOGEbCBKztpPIp4\/rRzVr1lQzCt\/W\/Pnz6ezZs8pfFi7EAj4uZH\/t2rWrqhz+3Xff0erVq+m9994rdeKIkAo49jMzM6lz585aK8ctiUlohRasrMIA9BAxJyUkZg5L7ZFgOY0fP175qODHguN9z5491LZtW+Uvq1OnTsSx9u3bp2K9YGXBrwVCGzt2rMrZD+LC\/zt16qSIq3\/\/\/oTtpL0hFm3ZsmVGTifFH6Y37ZwUTQ8Rc1KcsA2EJWZNHSyyTz\/9lD766CNC2ESHDh2oVatWZZa9wnovt5aY+MP0lJGToukhYk6KE7aBIDFYYcjyiqDWSKeJ5qbY3UhuScweH3Zs7i3uHsZImpOixXtaOWEbCBJDAOqMGTOUnwrbx6A1tyQmqXf0ZpCToukhYk6KE7aBIDHEa8H\/hZAHFMm95pprSszm5ZQU0e7UlyyukZWSk6KZoye9kThhGwgSi1Z\/En6xZ5555rLIYmF36st9SSExPcoxLyUkZh7Ty3pEN9tJcerrTzUnRdNHxYwkJ2zLzBLDFhIVj6688koVbIpKR7jX6NQQMoGKR9GCUs1Mf+lR3JCYBLnqzwInRdNHxYwkJ2zLjMTsVb+rV69OWVlZtH37dscZvFy2kxLk6k4BOSmaO2T8S3PCtsxILBEtMQlydad8nBTNHTL+pTlhW2YkFjpN2EoeOnSIkJIHmVax1UTiQlhhSFxYlk13Oyn+MHezxEnR3CHjX5oTtoEhsVdeeUVd4B4wYIBKx4PMFkh0+Nlnn6k00layQ\/\/T634EXRKTJIjusOWkaO6Q8S\/NCdtAkJh1Kfv222+nX\/ziF8UpcbDlzMvLU\/cco+XY9z\/t4UdwS2JSX1JvNjgpmh4i5qQ4YRsIErM7+UPT5nz++ecqbTQytVppdsxNtd5IOiQmTn09LO1SnBTNPTr+enDCNhAkBv8Xskwgjc5NN91UYvY+\/PBDWrRokcon5iZFtb8lULK3DomJU9894pwUzT06\/npwwjYQJIbpWrdunboEDp8Y0uUgfuzdd9+luXPn0v33369y5TuljvY31Xq9dUhMnPp6WIol5h4nLz2ExLyg5rMP\/F8vvfSSSmJ4+PBhNRrix1A0BPcpdaoe+XyFsN11SGxw3ieEEm1oUtlIbyY4KZoeIuakOGEbGEvMPn3YXiI7a1lG6dvfR4fEJFLfvQJyUjT36PjrwQnbQJKYv+kz31uHxKz0Oxn1q1D+4DbmXyIBR+SkaPGePk7YColprK5oJCbpdzRAdBDhpGjeEPLeixO2QmIa6yQaidlPJhf2bkK926dpjCoinBQt3rPNCVshMY3VFY3E5GRSA0SxxLyB5LGXkJhH4Px0w+kksli8\/fbbyqlvb3Dw9+zZUxW6NdFwpWnBggW0atUqNVyfPn3UVaeqVas6Dh+NxCSnvrdZ4aRo3hDy3osTtoGwxHD5G\/cjEdBav359Va7N3tLT09W1o3Ak42aq8azc3Fw6f\/48DRkyhFBhCbcBMDa+d4pF0yUxuW7kZiZI6k66g8uVtJCYK7j8C8MyGjx4MA0cOLBUxL7\/0UuOgFJwuFg+dOjQ4kvlBQUFtGbNGhWjVqlSpVKPjERict3I+wxxUjTvKHnryQnbQFhiIDFksMBX6N1Jb1Oo3wtVxmGJ1a1bV90KcGogMXt7\/fXXi789dPICZa44qL4f2KEKDbihiv7DmUsePHiQYGVLM48AJ2wDQWLWdhIXvOH7itf1IlQWX7p0KbVv315ZYeFqXkayxORk0rsCcrIWvKPkrScnbANBYqg7CYc+nO1NmjShG2+8sYT\/K5Yl2+AT27BhA61YsYIWL15MaWmlwyMikZicTHpTMvTipGjeUfLWkxO2gSAxbCezs7MJJrBTM+nYx+TC+sLW1UrtgwrkI0aMoIkTJzpuZyORmJxMelMyITHvuOn0FBLTQekylSkqKqJhw4ZR165dVeofhHOsXr2a3nvvPbWldArj0CExOZl0vyA4KZp7dPz14IRtICwxTBf8YthSokguYsaQXwy5xDp06ECNGjXyN6Mhvfft20c5OTlUWFio\/G8gNDyvdu3ajs+JRGJyZ9L71HBSNO8oeevJCdvAkNimTZvokUceodtuu00FvU6bNo3eeustRWqIH8vIyPA2mwZ6hSMxuTPpD1xOiuYPKfe9OWEbCBKzcuxnZmZS69atlX8KhNagQQP661\/\/Sm+88UbYGC730+u+RzgSk2yu7rG09+CkaP6Qct+bE7aBILHQQroWiSFmLMg59uVk0r1yCYn5w0y3t5CYLlKG5Kwc+\/369VOxWnYSC3KOfTuJSTZX94uBk6K5R8dfD07YBsISw3Qhxz5OCQcNGkRPP\/00Pfzww3TmzBmaN2+eSk8dxBz7ks1VFM0fArHrLSQWO2zDjoyg0\/Xr1xOi6P\/9738ruQoVKqjsEllZWY53GuP1muF8YkJi\/maAk6L5Q8p9b07YBsYSs6YJ2SWwvUTDZWynC9nup9Rfj3AkJuEV\/nDlpGj+kHLfmxO2gSKxL7\/8UlX8\/uCDD1TQ6S233EKoCl6zZk33s2iwhxOJSXiFf4A5KZp\/tNyNwAnbwJAY0uEMHz5chVW0a9dOzRii6BGYijuVHTt2dDeLBqWdSEzCK\/wDzEnR\/KPlbgRO2AaCxLB9RI4vWF72LBaI3F++fLkKfg2X68vd1HqTdiIxCa\/whqW9FydF84+WuxE4YRsIErPHiYXmEwtqnJiEV7hTKidpTormHy13I3DCNhAkhlQ8SD\/961\/\/mtq2bVtithAnBmtsxowZxnLsu1sORE6WmGSvcItiaXlOiuYfLXcjcMI2ECSGbSPI6qmnnqK+fftS06ZN6YorrlD5pubPn69CLFq0aKFmERe2TeTad7MkIpGYZK9wg2RJWU6K5h0lbz05YRsIEsN2EkQF31e01qpVK3rhhReiiRn9vROJSXiFf4g5KZp\/tNyNwAnbQJAYLLETJ06odDzRWhAsMXt4BQrlomCuNPcIcFI09+j468EJ20CQmDVd8I3t2LGDtm3bRlWqVFEZLVDCLSkpyd+M+uwdaolJeIVPQP\/bnZOimUFMfxRO2AaGxL7++msVZvHuu++qWLGzZ88qn9gdd9xBkyZNKtOA10gklj+oDWU0kApH+ur1gyQnRfOCj58+nLANBInh3iTuTH7\/\/fcqg8VVV12l5u\/IkSMqOSLKqeFCePny5f3Mq+e+oSQmMWKeoSzRkZOimUFMfxRO2AaCxFAoZNSoUap4R2gq6l27dtFjjz2mslnE+1TSWjKhJDY47xPKe\/+I+rWk4NFXrFBJTormHSVvPTlhGwgSw+kkrhyNHz++FIkFMdhVsld4UywhMTO46YwiJKaDkkEZZK6YOnWqsrSQesfaNuLUEhfCEXoxZcoUQv3JsmihlpiQmJlZ4KRoZhDTH4UTtoGwxDA1u3fvpiFDhqgg15tvvlkRGS6Af\/XVV7RkyRJq3ry5\/gwalgwlMYkRMwMwJ0Uzg5j+KJywDQyJYXpQEzI\/P582btyoTidxIbxHjx5aJ5MoNoLrSSA8\/B9ZL1CWDQQU2lAsFz62tWvXql+5KdkmMWL6ihRNkpOiRcPC9O85YRsoEvMzkStXrlTxZajinZqaqggK6X1CC+IioBYHBfgXRXRh+SEt9s6dO9X9zeTk5FKvYbfEJEbMzyyV7MtJ0cyhpjcSJ2wTgsQQJIsSb7DarPqU+\/fvVyee06dPJ3tmDJBXbm4unT59WoVtIJD22WefJRwujBkzxjGMw27NXajRmE5ljFYrKadLDcpscrXeqhKpUggcPHiQ0tPTBZkYIMAJ24QgMRATtqIpKSnFltTmzZsVWSGhYlpaWollgpAOEBa2rWi4jwlZnQrgEiNmTuM4WQvmUNMbiRO2CUFi9mnFiSbIafLkySpwFtYZ7ltazQqsxc+sAFoQ2OHDh9UJqFNOf\/t2UvKI6SmRjhQnRdPBw6QMJ2wTisTg10KoBvLzI+6sWbNmJQgMiwTbxqFDh9LYsWOLTzwPHDigrjyhb2hSRvSxk5jkETOnapwUzRxqeiNxwjZhSKywsJDGjRunvrp06RL20rgTicEKg08Njv06depEdOxLjJieEulIcVI0HTxMynDCNiFIDCEVIKHMzEzq3LlzxLUQek8T8WjPPfcc7dmzh7Kzsx0Dau2WWOupWwhhFhn1q1D+4DYm1x27sTgpWrwnlxO2CUFiiPvq37+\/CpOwt1q1atGyZcvUCRi2j506daJevXrRyZMn1Ykkfod4tD59+qibAuHuZtpJTAJdzakjJ0Uzh5reSJywTQgS05tW71IWiUmtSe8YOvXkpGhmkYs+GidshcSir4dix74EumqA5UKEk6K5gMWIKCdshcQ0loxlidlJTJIhagAXRYSTovlHy90InLAVEtNYGxaJSaCrBlguRDgpmgtYjIhywlZITGPJOJGYJEPUAE4sMf8geRxBSMwjcInazSIxCXQ1O8OcFM0sctFH44StWGLR10OxY18CXTXAciHCSdFcwGJElBO2QmIaS8ayxKxAV6n6rQGahggnRdOAw6gIJ2yFxDSWjkViEuiqAZYLEU6K5gIWI6KcsBUS01gyILE3P\/iYYImhSdVvDdA0RDgpmgYcRkU4YSskprF0QklsTLd6NKZbXY2eIhIJAU6KFu+VwAlbITGN1QUSW\/naR9R90VYlvbB3E2WNSfOHACdF84eU+96csBUS01gfILFpq98hFM1Fk2h9DdA0RDgpmgYcRkU4YSskprF0QGIDF2+iWa9+ISSmgZeuCCdF08XElBwnbIXENFZNKIlJtL4GaBoinBRNAw6jIpywFRLTWDogseYjnqeCvceV9LG5t2j0EpFoCHBStGhYmP49J2yFxDRWj53EJNBVAzBNEU6KpgmJMTFO2AqJaSwbkFjlB1aptNRCYhqAaYpwUjRNSIyJccJWSExj2YDEjt\/5jJKU3PoagGmKcFI0TUiMiXHCVkhMY9nUbd6BTnadpSQlWl8DME0RToqmCYkxMU7YColpLBs7iUm0vgZgmiKcFE0TEmNinLBNGBJD2bbly5fTkiVLCP\/v2LEj5eTkqDQ6oe3SpUv08ssv09y5cwmTDVkUznWSRd9rO9xOpzJGq2GExIzpmcK+Xr165gaUkYoR4IRtwpDYypUradu2bTRx4kRKTU2ltWvXUkFBAc2cOVNVBLe3rVu30pw5c2jKlCn005\/+lDZs2ECvvfaao2woiUm0vjmm4KRo5lDTG4kTtglBYufOnVPFc3v06EEZGRlqlvfv30+jRo2i6dOnU8OGDYtn3iqe27JlS1VsFw0\/O3PmjCK7pKSkUquk1h3D6ez13dXPry6YTZtXL9JbSSIVEYGDBw+qmqDSzCPACduEIDFsD4uKiiglJYWSk5PViti8eTPl5ubSggULKC3th8vaKJyLQro33HAD\/fnPf6a9e\/dSly5daPz48VS7dm3H1WQnMYnWN6dwnKwFc6jpjcQJ24QgMfu0Xrx4kTZu3EiTJ0+mESNGKOusXLlyxSLHjh2jrKwsZQFkZ2dT9erVac2aNcpHNm\/ePMcq4Gl3T6Xz1\/6PGkOi9fWUSEeKk6Lp4GFShhO2CUViO3fuVA56bAuHDx9OzZo1K0FgioT+S2IgOGvrefToURo5ciRNmDChxNbTWlTX3LuELtRoLIGuJrWMSBz7hvG0DyckFkNwYzV0YWEhjRs3Tn1he+jk28KzT58+rbaTsNA6d+6sXgckNnr0aGWZOZ2WCYnFZtY4KVpsEAw\/KidsE8ISQ0gFHPtw1FvEFGnRrFu3jjZt2kSTJk0q3k7u2LFDkVjFihVLda3x4Bq6eGUNidY3rImcFM0wdFGH44RtQpAYLKn+\/fsTtpP2VqtWLVq2bJnyf8H66tSpE\/Xq1UudRubn56swi2+++YbuvvtuGjZsGNWsWdNxcUiBkKg640mAk6J5AshHJ07YJgSJ+ZjrqF1x6VsKhESFyZMAJ0XzBJCPTpywFRKLslDsJCbR+j60yqErJ0Uzi1z00ThhKyQWZT0U7DleXCBESCy68riR4KRobnAxIcsJWyExFyQmVY5MqNcPY3BSNLPIRR+NE7ZCYlHWQ977R6TKUXSd8STBSdE8AeSjEydshcSiLJRZr+6XKkc+lClSV06KFiMIww7LCVshMRckJvcmzaoiJ0Uzi1z00ThhKyQWZT2gYC62lGhybzK68riR4KRobnAxIcsJWyGxKCum+8KtqlSbFAgxoVolx+CkaObRizwiJ2yFxITE4q1fxc\/jpGjxBpkTtkJiUVYXovUR8CpVjsyrISdFM4+eWGIWAkJiUVaX3JuMnfoJiQm2JhAQEouAotybNLHEwo8hJBY7fDlhKySmSWJy5ci8wnFSNPPoyXZStpMaq0ruTWqA5ENESMwHeFG6csJWLLEoltiQvE\/onZ17aEHW\/6rq39LMIcBJ0cyhpjcSJ2yFxDTWBIrq7tu3T0NSRNwgwEnR3OBiQpYTtkJiGitGSEwDJA8inBTNAzy+unDCVkhMY6kIiWmA5EGEk6J5gMdXF07YColpLBUhMQ2QPIhwUjQP8PjqwglbITGNpSIkpgGSBxHB1QNoml04YZswJIaybcuXL6clS5YQ\/t+xY0fKyckhTGakVlBQQLm5ubRo0SKqVq2aoyinBaGpI0bEBFcjMLJfswlDYitXrqRt27bRxIkTKTU1ldauXUsgqJkzZ6qK4E7tq6++ogEDBlBycjI988wzQmKx0yn2ihZnaNUfby4n6glBYufOnVPFc1HVOyMjQ62X\/fv306hRo2j69OnUsGHDUmsI1tqUKVOobt26tGXLFlWDMpIlFu9FKM8TBAQBPQQSgsQuXbpERUVFlJKSoqwqtM2bN6tt4oIFCygtrWSQKuRhqcES69atG82ePTsiielBKVKCgCBQFggkBInZgbt48SJt3LiRJk+eTCNGjFDWWbly5Upgu3v3blq8eDFlZ2erCuCw1iJZYmUxMfJMQUAQ0EMgoUhs586dNHXqVOUDGz58ODVr1qwUgcFimzRpEj3wwAPUvHlz+vzzz4XE9NaKSAkCgUQgYUissLCQxo0bp766dOlCSUlJjoCDtPr160eHDh0q9fvnnntOnWpKEwQEgcsHgYQgMTjp4djPzMykzp07u0JfLDFXcImwIBA4BBKCxI4ePUr9+\/cnbCftrVatWrRs2TJKT0+nsWPHUqdOnahXr14lZITEArcm5YUEAVcIJASJufrEIiwICAIJhYCQWEJNp3wYQYAfAkJi\/OZcPrEgkFAICIkl1HTKhxEE+CEgJBZmzk+dOkVPPvkkrVixgipXrkx\/+MMf6N5776Xy5cvzWyU+PvH777+vgor37t0b8VI+blG89dZb6q7rZ599RvXq1aM\/\/vGPdMcdd5SK9fPxOgnVVRdb+4f++uuvaejQoTRs2LCECScSEnNY1lAoZMPAqSei\/r\/\/\/nsaP3483X777UqppOkhgPurOBUGhm3btlVXwVatWkWPPfYYVa1atcQgn3zyiQqTwX3Wpk2b0scff6wu8+M2RZMmTfQeyEjKDbYWLBcuXFB4IttLIsVECok5LHxcRYIVMHr0aBXVj\/bSSy\/Rpk2b1CKoWLEiI3Xx\/lFXr15dTEawYE+ePElDhgyhhx56iNq3b19i4BdeeIFAZAhWxjUxXOoHiYH8QsNivL9R4vR0g631qdetW0f\/+Mc\/1J3h+++\/XyyxxFkOpT+JU+zY1q1b1VYHdy7DZbtIZEy8fLZZs2apLCEWCZ0\/f15tLdu0aVOKmBCwDIvXSpt05MgRevjhh2nQoEF00003eXl8Qvdxgy2AwH1huEdGjhxJTzzxBPXu3VtILJFXCK4wYZHYc4xJUKy7GQcpOQUYhyqf06hIrYzt+7XXXksTJkwImw\/O3RsljrRbbOHfxZ1i\/DG5\/vrr1bwIiSXOenD8JOEsMfhyFi5cWMqfk+BweP544ayFG264QWUXCW3ffvstzZs3TyWzhBWGNEmydXeGXxdb+HeRMBQNB1Nnz54VEvO8oi+jjocPH1Y+MaTzadSoUbFP7O9\/\/zs9+uijxTnLLqOPVCav+uyzzxK2hZafCz4xZBeBXwxbSnuzDlGQeQS\/v+qqq8rknS+Xh+pia1ltL774YqmPNnDgQBozZszl8pHDvqc49h2gwSkOLC781cJRNBz9yBKLSRf\/jP6aR1gFti74Y4BtzJo1awh\/CJxShsNRjdAKnFBKGEt0jN1gax\/NIjXZTkbH+LKXgNWArLAwxWvUqKHCBLp37y4K5mJmsZVBLBPyt0HpkCIJvq7atWurUUBc77zzDs2YMUNtI\/\/0pz+VGh2\/k9PJ0qDrYos\/GJUqVSoeQEjMxQIWUUFAEBAE4oGAbCfjgbI8QxAQBGKGgJBYzKCVgQUBQSAeCAiJxQNleYYgIAjEDAEhsZhBKwMLAoJAPBAQEosHyvIMQUAQiBkCQmIxg1YGFgQEgXggICQWD5TlGYJAgiBw7NgxFbz8yiuvqBhKNyUOcZ0PmWFmz55NDRs2NIaIkJgxKGUgQcAsAk6JCCI9AQSTlZWlrhK5IRc3b413AonNnz9fZShJTk7W7i4kpg2VCAoCiYFAUEksNMOLLtqmSezixYsq95xYYrozIHKCQAwQwAV5XG9bu3atGr1r167qvumBAwfonnvuKX4iMrG2a9dObeMWLVqk7plWr16d+vTpQw888ADhvi+ssO3bt6s+1uVuZAbB+M8\/\/7y6C4zsIUhKaV39Cv1IofJ4H9wbRlokXBPDZX6rOWWHxXvk5+eru8dIqYQL\/chh9vOf\/5zTp\/r3AAAEbUlEQVT27NmjtpN437y8PAJJN27cWCVVsJJkIqMvrqC9+eabiqBuvfVWdeWvfv36ZFmav\/vd71SC0iuuuELdwxUSi8HClCEFAR0EkL0WCoyMHSj+jIb7o\/\/617\/Uz3HvFNs2JDME6aA4NLLd5uTkUIMGDVT6dORbQ+X7vn370j\/\/+U+VPx9pjKztJGSRxPO+++6jChUqKCLCuHPmzKGaNWuWeE3kHcM915SUFHrwwQdVGqT169erPk899RShGPXbb79d4p3s20nc5wQZI7U77ryiTsLf\/vY3lZcPP0Pr168f\/eQnP1HvjfHgV8OYGB+khHcHcSENPMYDHl9++SU9\/vjjhKSaIGrca8b4SCpQVFQkJKaz2ERGEIgFArB6BgwYQHfffTf17NlTWR5QUGy7kBYd2YTtWzeQD1JL\/\/KXv1SJCKDkUGYoP\/xgoT4xq6YBCADFbtDwzMGDByvS7Ny5c4mPBcsI5Pn0008XW2rWhfFWrVopCyrSFhfZXvB5QFQgVjT0hyMfOeRgTYEc8T3SjqMh4ywsPbxjamoqvfrqq6qvlT0ZueVAuCBCNJAY3sMaHwQrllgsVqeMKQhoIAAS2rBhgyIOWEVI84TtW4sWLRRJhRIG5FEgBOmM8C9IDtsvKLUTieXm5tLcuXMd38QpO4iVVSQ08wWIFA3PiERiIF9sHZE81MrDZ3+4k08s9Gcg4i1bttCnn35Ku3btUl\/p6eklSMx+cIF3ExLTWGwiIgjEEgFsk1DAA8qL5IWoBIVtJNIXWZYYfoatGraW8Am1bNlSWTawmsJZYrBgMAbSH9nT8eCz4PvQnzmRWKi1F43EIoVQRCMxPAulETMyMhShX3fddco3CCK2W2JCYrFcjTK2IOACATi+oaDIIgz\/ERr8XNiSITkkCqdYJAbCgcP\/xhtvpN\/+9rdK1qoIBQe\/kyWGCl3wNS1dupTS0tKKt5PTpk1TRIgtor3BWQ7yDJXH9vNXv\/qVyusWicSQERnbPVhj1lbVvn2tU6dOqTgxO7F99NFHKjU5LEErsy9IHVlsw5EYrE2xxFwsOhEVBEwiAEc6FB4nf\/BRwScGRzjK1+FEEaQAKwo+JJAcanKePn1aEUFSUpLKlIs6nvCR4WffffedSu2N5J233Xab8kfBUQ7\/GsYH6Vn1VGGlwQdlbyAcyINsLMc+rDNYiDgxvOaaayKSmFWvFWSIwiSwHtEfjnt8nuPHj0ckMfjHQOrWocAHH3ygKjOBwNEflbBC4+BQ5k9IzOSqlLEEAZcIwFEPy+O1115TjnqcKsLRDb8YSAUhDfgdwhlALrCiNm7cqLaSKGcHokDdAlhud955p7JaQHqWnwwhHLDmXn75ZXXqeNddd6kQi9DixdZrhwv5sEIyosWugThRnBfvceLECbr55puVlYj3jbadhO8Lfa2TTBQ2gQMfnxnZlUH4+Mz27SQwExJzuehEXBAQBIKDAEhTSCw48yFvIggIAi4R+PDDD4XEXGIm4oKAIBAgBHD16P8AZn0bQ2IlA0MAAAAASUVORK5CYII=","height":184,"width":305}}
%---
