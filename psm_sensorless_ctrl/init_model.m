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
% model = 'psm_sv_ekf_bemf_ctrl';
model = 'psm_svpwm_nonlinear_ctrl';
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
use_torque_curve = 0; % for wind application
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
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAATEAAAC4CAYAAACLrdvMAAAAAXNSR0IArs4c6QAAIABJREFUeF7tXQe4VcW1XhqRoqC0hxQpISBNCCKIEcGEFiUCGhBFFBF8iBRFpIqCgNINoQWkaFQEGwiIUoQgYiJP8QoYQZoXLDQRBAtRxPf9g3Ocu88us8+Z3eKa7+O73Htmz6z9z8x\/1lqzZs0ZP\/7444\/EhRFgBBiBhCJwBpNYQkeOxWYEGAGBAJMYTwRGgBFINAJMYokePhaeEWAEmMR4DjACjECiEWASS\/TwsfCMACPAJBbgHHjrrbeoY8eONHr0aOrQoYPxnnbs2EFdunSha6+9lgYOHGi8fbXBb7\/9lgYNGkR79+6lOXPmULFixQLtz67xZ599lqZMmUKPP\/44ValSJfT+M+1QjhOe9yN7HDBX31m+x4QJE6hhw4aZwmH8OSYx45D+3GDYJPbFF19Q165dqXz58jRmzBgqWLCgsbezLig0HFRfdkL7WUB+cQhynFTcpk6dKsZE9wsgbiSGcRk7diwtXbrUFxkbm4QODSWWxADmzJkzU6\/1zDPPiG8HOfD4oE2bNtStWzdRx6oNqc\/XqVMnpV3ICf3AAw\/Qe++9JwYMmo4bKUBDGDx4sOinTJkyqQG2Lg70ib\/dcccd1KtXL5IyywX62Wef5ZFV\/r1evXqifxRVGzp8+HBKE0ObIJVNmzaJenYyW+WxWyReuEITGz9+PPXv3z+tL7SnytC9e\/c8GqJsGxih3siRI\/OMi5RPDqrERy4efC61QCk7xgdFjqEkVysOTrJZ+5TzxE0W61qSpCn7lLKAsDBeUkZ1bqhtWJ+XuKnjI+eMdS5bcVDHXb4DyHPWrFlifUBjV+cbLIXjx4\/n0bCd5iP6DlP71yW\/RJKYShrqi2LSYwKpE8f6OQbSulDVRY+JiIG1FieT0DrZ1QW1ffv2POakHUFUrVo1z8JXF3Dx4sUFSZkiMblY5GS2aixLlixJkbETrk4k9uCDD9KIESNSC1Y+L3FzGjMn0lC\/EICDVetzGsMBAwaILwiVxNxkq1ChQp7xhjzWv1m\/nFRsrCQiP8M8BHmMGzfOlcScnoccrVu3tp3LKhna4SBJ0Do38Xfrl50qL74g5Bej\/EK1fqHIOYO\/R+VWsK7NxJGYBPHQoUMpjUcuEAxSnz59xMBv3LjR9nM5iNLkktoNvi1BgiggMfmNJknNTrNRJ6DUGuSkUtuSC1V+phKiKju+JeXEw7tcf\/312iSGZ3XMKKkNYgKqJNu8eXNBFF64Sp+Y1HgkjlLTsftdJRHrF42KjWqmqLhIHHr37i18i1YNUmo8csytpOcmGzRcOcY6slh9j3K85Pywzif5perkS7Q+\/8knn6TG3A03FUfZtsRB\/i7H1047s85vqT2uWrVKfJHJ+ez0fur60tWYgqqXOBKzmlgYOPXbQX77ATBpAqrPwLzs0aMHWb9pUN+OeOz6k4Ph9a1kZ05a\/Qkq6VmdpX7MSV0SU53j7777bspRjndStT4nXJ1ITPW\/qVoXFovUjtRvbxUbJ40D9UHmTZo0SdsgcTJ78YwbmVtls5KYlyxWEpPtqV9M6heR\/FJ1IjG75+X8cjP3VVMb9VWtSxKS1RJAPWt\/1j5gdqpuGimLlaSZxLKgZFMk5rSjZyWe\/zYSk+8DrQs+P0n2qgYgyd\/uy8GJxKwaiDSToZnpkpjfhe7lF3PStlXZnEhMdxc2ahJTtXtJwFZNzE7zl3+zIzE3x73XF3cWSzvjRxOniemak9I8VH1gqk\/Ayab3Q2JyAshvpXLlyqV8GE7mpHWCWBeBnRlVsmRJ4X+Q\/grr75KQdcxJdeMDk\/3GG28UJpourk4kZjXJVDNE9VO5mZPQAKwahqplyIVn954SNzzvRlJ4V6uJZGdOOsliXWmmzEmrHwtjKs1JlVBVzV36TOX4yy8oOT\/cNDEvc9LJB8yO\/Yy5Nu+DQTj2nVRwN01MVc9VCa0TxMnXgmesO0H4m3TcSt+OdFLLPqSs6u6kak6ivtuOqsTPulumg6uVxGRfTma6lMNp08DNsS9NfLlYpU8Mf7dzaNvhhv69ZFM3c5wc+1IWq8nv5tjHF4\/VT2UNr7DuTMoxtvN54VmVxJw2hdzMSaf+rHPK6m5R57C6S2xoSWfVTOI0Mfm2OqEATtvS1kXgFhbhRWLWtuzCNdxIzOrPsC4W1dchwz4kkVhJTF1QbiTm9k46uKqLUw1BmTx5svCnAM9p06bR3LlzUxssqpYKjOTY2PmS5Bi7aV525GENs9GVDf3BH4iF60SqbgHLTiEWIB2dWC+nkAYdn5j6xYP3f\/3118UY2FkCEle3OQWZrfJILdFPrF5WrOTz4cBI7KuvvqJhw4ZR+\/btbaN716xZk4rhgszq4vf5Dnmq60yabNrnZ\/0j4LWLqxP9jcW6YMGC2Gzr+0chHk+47YaaDpAO640DITHkWXzhhRfo\/vvvpyeffNKWxDAh8e3cqFEjo+\/KJGYUTmON2ZmqTsGfdp1KLUA1KY0J9wtqyMmctAYmJwmSNBI7cuSIcChit0qnLFq0KK3arl27aPbs2fTDDz+IWCe7b9pJkyZRs2bNqFatWjrdcB1GgBFgBGwRSCMxMDWCRa+77joqXLiwK2w4rnD11VenmXPwH9xwww20bNkyEeNjJbH\/\/Oc\/wtSEbwc7e7\/+9a9Fn40bN6YzzjiDh4oRYAQYAW0EbDUxBIzi6EbRokVdG4LWZq2zcOFCOnjwoAhSxJELOxI7duwYTZw4kdq2bSt8YTk5OTRq1Ch65JFHqHr16nn6BMFxYQQYgfgjsHr1aqpUqVLogqaRGBzycP5ddtllnpqYVdrc3FxxXgy+MJAbdrrsSMz6HHxo0N4qV66clrIGJLZ79+7QgTHRIctuAkX\/bTDu\/jEz8URUuNtqYnfffTdt2LBBxBq1a9eOLr30UjrrrLM839PuMDQesjoNP\/roI2FGIsL47LPPJkli0MpatWqVpokxiXlCb7xCVBPSxIuw7CZQ9N9GVLjb7k6eOnWK9uzZQ4sXL6bnn39evE2nTp0E6WBHSddv5aSJ7d+\/X6QEgcmKDA0gNJiX2OKtWLHifw2JgayjUK\/9T7\/0J1h2Eyj6byPJuMeKxFToT548KbIdIGTixRdfpGrVqolAxauuuspTO1NJTG7tgrzg6N+yZYswIaG91axZU+yI1q9fP23UowLG\/\/RjIjCBmYk2kkwESZY9qrWqHScGkw8RzUhkd+DAgdCCDqMChhcTa5Em5oDfNpjE\/CKmce8kdiBfe+01EbSKkAjk2sKuInKcn3nmmf579PkEk5hPwAxVT\/JiYtkNTQKfzUS1Vm01McR\/rVu3jp5++mmRruXKK6+kW265RTj4TeZt18EoKmB0ZPOqw4vJC6FgPmfcg8HVq9Wo1qptsCtyTZ04cYI6d+5MTZs2JaT2iKpEBYyJ9+XFZAJF\/20w7v4xM\/FEVGs1jcQQTQ+fF841hmEueoEXFTBecul8zotJByXzdRh385jqtBjVWrXVxB566CFxLMjrainsOHrV0Xl5tzpRAZOt3HieF5MJFP23wbj7x8zEE1GtVVsSu+eee0QohZf\/CxkjhgwZYuL9HduIChgTL8WLyQSK\/ttg3P1jZuKJqNaqrTm5efNmglmpU0yn0rH2GRUwOu\/uVYcXkxdCwXzOuAeDq1erUa1V7TgxrxcI6vOogDHxPryYTKDovw3G3T9mJp6Iaq0yiZkYPYc2eDEFCK5L04x7NLgziTngHhUwJqYBLyYTKPpvg3H3j5mJJ6Jaq6yJmRg91sQCRNF\/00xi\/jEz8QSTGGtiJuaRsTaYCIxB6auhJOMeSxJDgkRcw4VLHiAgwilwHOn222\/3zPrqa+RcKkcFjAn5kzwhWXYTM8B\/G0nGPaq16mhOImvFjBkz6NChQ+KyD9xlBxLDxSCID0PiRJ1Eif6HMe8TUQGTrdx4PskTkmU3MQP8t5Fk3KNaq44kJm89AnHlz5+fZBQ\/4seQC3\/EiBGhaGNRAeN\/+qU\/keQJybKbmAH+20gy7lGtVd8khmvYkMwQt1F7XSTifwjTn4gKGBOyJ3lCsuwmZoD\/NpKMe1Rr1dOcxP2TuH5t1qxZwoRcvnw5IdsrLjFlc9J9kiZ5QrLs\/gnIxBNJxj12JIYBge9rzpw5NGXKFPr+++8FaSHXPs5WFilSxMSYebYRFTCegmlUSPKEZNk1BjiAKknFfe8XJ6hhp8FC0bmp\/gVUvliBANCxb1IrTgyaFxIlnnPOOeJ2ojALk1iYaP\/cV1IXE2+oRDNf5r+9n3rO3yo6X3JXXWr0m\/NDE8SRxBBe8dxzz9GXX35pK0zVqlWpQYMGgSdMZBILbS7k6YhJjHH3g0BsSWz69Om0ZMkSat68uTAfP\/\/8c1q5ciW1aNFC7FgiZgz594MsTGJBouvcNpMY4+4HgbErcmnsio\/ipYkhlGLChAnUpk0bqlWrVup9Nm3aRC+\/\/DLdd999tGPHjjyf+Xlp3bpMYrpIma3HJGYWT93Wkoq7SmLvDb08Hj4xZG21y\/Cq\/h0Dk21mV5ityCLbvn17cR+ltTCJ6U5\/s\/WSupjYJ2Z2Hui2FksSgyaGgNZmzZqJi3Jx6zei+GFCLly4kB5++GHatWsX1alTR\/c90+qhPVzKe\/\/994sr4ZjEMobS+INMYsYh1WowqbjDqQ+\/GEpsNDEIs3v3bho8eLDwhcEnhhuQEMkPM7NUqVJiO\/WVV17RGhy7SiDB2bNnEwJocbSJSSxjKI0\/mNTFxJqY8amg1WDraTm0ftdRYUaCxMIsniEWCK8AmR08eFCEWODS3HPPPVcEvIJ84ODPpCAGDZH\/CKRdtmwZNWnSxJHE0P7q1asz6SbSZxAojFujklhY9mhGLYm441rHYy3G0qlCJeJHYjD3EB8GwpIFIReTJk2i4cOHZ3XsCCYpiLF79+40btw4VxIDiSaxsDYTzagx7uHijkDX3476l+h0YMtKNLBlxVAFcNXEQDSDBg3KQ2KFCxem2267je66666MtbDc3FyaOnWq8IXh\/OXYsWOZxEIddu\/OmAi8MQqiRhJxjzK8AmPgGuwKkrnjjjvEWK1du1YQFwJg4R+75pprMh7Dt956izp27Jj2PLSygQMH5vk7705mDHNWDyZxMckXZtmzGnpfD0MLaz09h\/DzzG8+p89ntPf1vInKjiSmhlJ8\/fXXNHfuXBowYAAdPXpUaFFw+MM3ZqKwJmYCRbNtMBGYxVO3taThrmphZ+99k\/a\/MFT3VY3Vc9XE4Pe65ZZbqHz58iKkAqYlDoLj\/wi\/yDZGTL4Fk5ix8TTWUNIWk\/riLLuxaeDakEpg2JU8NvdmsQkYdnH1ia1fv14kQIQjH7uDiOn67rvv6I9\/\/KMw+zgVj\/tw8WIKezqf7o9xDxZ3mI6ICZPHjEBgU2+sTre2uCR+JAYoEEaBgh3KDz74QPy\/Ro0aGTv1\/cLLPjG\/iJmpz0RgBke\/rcQZdyt54d0kgSFrRVRr1dWcfPXVV+nqq6\/O4\/vCMSEc+m7ZsiUVLFjQ7xj5rh8VML4FtXkgzhPS6\/1Ydi+Egvk8jrjbkZckMKTdkbnDolqraSSG40abN28WUfo4CnTrrbcSwipkgcMfl4VMnDjRmE\/MbTpEBYyJKRrHCan7Xiy7LlJm68UFd0lcb+48IiLx1QLSuql+6bR4sKjWahqJyQtC4KCT0cNqIsQCBQpQly5dRHoenKcMukQFjIn3isuEzORdWPZMUMv+mahwB2m9uesord95JHUG0vo2TuQl60W1Vn2bk9kPk78WogLGn5T2taOakCz7R1SpUiUTMITeRhhzBoSFfx8fOeFKWtJkvKLy+ULz8srWGtVaTSOxU6dOiWyuOHLkVKCBnXfeeXTmmWcGPshRAWPixcKYkCbktGuDZQ8KWfd2TeMuNSz0Ci3r4y9OpJmHdhqXLnGpz0a1VtNIDD6vrl27EpIfOhWk38EFIqbixNyGNSpgTExh0xPShEy6bbDsukiZrZcJ7iAqFJiDfshKSg4zEaTV6DdFxSUfmZao1qpnFotMX8jUc1EBY0L+TCakiX5NtMGym0DRfxtOuEuiUs1AaFV7j5w2DXWLSlgXFi3gaSLqtot6Ua1VVxKDSbl9+3ZatWoVHTp0SKSVQchFmOllogLGz+A51WUiMIGi\/zaSiLskov\/74CP6Pn9RQUx7v\/hWmH9+iQqIgazKFy1AFxYDURUl04RlNypRrVWtiH1cDFKhQgV65513xEFwHBNq1KiR\/9mVwRNRAZOBqGmPJHExyZdg2U3MgJ\/bsNOk8GmmJCWJCj9hCqKERVZOyES1Vj3PTiLtjnpRyPvvv09PPPGEyCdm6gC423SJChgTU5iJwASK\/tsIG3erTypbLUp9YxlIaiUqoWmFeEGtzihEtVa1slioDnynC0R0XjKTOlEBk4ms1mfCXkwmZGZNzFl7kuSUrQblRlIXnXdS3OUahvlncr6grajWqiOJIX30gw8+KNJH169fP\/W+rInpDz2TmD5WJmu64a6adYKMfnKMw\/9kkpxUc0\/6psoXKyi0JxCUkyaV5DkTOxLDIGzcuFHcL1m7dm2qWLGiOIrEPjH95ZbkCZk02VVyenf7x0KbQVyUJCb8tB6f0R\/J9Jqq4\/w0Yf1MUPjdKzDUqe+k4a6+RyxJDALCfARx7dmzh4oXL064FKBs2bLZjL+vZ6MCxpeQDpWTPCGjlt3qCAfEVnPONDF5aU\/y8yB9UVHjns28j2qtOpqTuCAE17FhF7JMmTKhnJO0AzAqYLIZTPlskiekadnVWCYZ66QSE3bpxO8+4550xkmSjjTr7DSnuDjKTeOug4+pOlGtVdfdyRkzZtBTTz1F1apVow4dOohD32pGC1Mv79ZOVMCYeLckT0gv2e18S2GRkp3GpBJTvv8coQsuKO3odzIxtkG14YV7UP2aaDeqteoZsY9kiAh4ffHFF2n58uXixu\/OnTsLZz+fnXQf+iRNSKv5huDmb39VWARcogSpKamkJP7\/U5Am\/o\/YJxQ4w3XNuSThbp1BSZY9tiQmQUb0Pvxif\/nLX2jv3r18dlLjqyuqCemlJUlSgukmtSeN1\/Fdxc2Mk8QUhBkXFe6+AbJ5IMmyx5LEkNECxLV48WJaunSpgBxmZevWralUqVKh+MmiAiYuE1KXkAQZBeBPkjiohCQI6KdAS7krFyQp+R2LJBNBkmWPaq06mpNIjtijRw\/aunWruCOybdu2VKVKlVBMSHXSRgWM34VjV986IeNCSKr5ZufsxufsVzIxA\/y3wSTmHzPXYNddu3YJp76fW42QEXbo0KGEC3Jr1qwpAmbVYFkp4po1a6hbt24piZ3S+8SdxOxCAWR80vbPjlLBAgUC1ZCshKRqSZn4k+SAJHkxsez+icDEE1GtVU\/Hvp+XwyUiuDUcWlvjxo0JV77NnDmTJk+eTCVKlMjT1IIFC0Q2DK+D5FEBo\/qK1DxN+Hs2h3a98LTzI50mqtPBlKrZphKYV7t+P2ci8IuYmfpJxj2qtWqUxDAAjz76qLhct0iRIiJQtl+\/fjRkyBBhiqoFd1k2a9Ysz+Fyu2kQFjDWlL06GTC9pm2ZImcJLRYmm6ohxdGPZH2XJC8mlt1rZgbzeVhr1Sq9URJTG8du5rp162jevHkidU\/Roqe3ylFwo9KwYcPELieONuHlcbs4tDfr5SNBAiNT987\/v32+jqR4HTmRO268mIJZLF6tMu5eCAXzeZBr1U1iRxID0Wzbtk34xPLnz5+HgOD3gmbl5CvbsWOHuBHp8OHDNHjwYLExoNY9duyYuPINZid8YTk5OTRq1Chx23j16tXzyAtgUHADuYny2bGTtPHTE7R061fip1OBFlWv7GkNCj\/xe+nCZ4mfukXeFqVbP071WPZoRiOJuOMooizghrCL40UhR48epfHjx1P\/\/v3FpSCy4BA4TEYQjleOfQzIgAEDqHv37tSkSRPHd4PWNnr0aKpcubII4VCLSXaXV69b0\/lKzeqK3xT9Kdf46SRz2RbWCLJFMLPnGffMcMv2KZNr1Y8svi8KyZcvH\/Xq1UuEX1g1MWhUK1eupL59+5K8qxKmJDJgqOSESQYzEvFmqCdJDFpZq1atjJMYSKvX\/K1pJmMjXEXVoHRWlyO4gc2Lyc9UNFeXcTeHpZ+WYkNiUmjEiU2dOlUQlurPcnup3Nxc4dtCiEWNGjUIk+mhhx4SJqVqJu7fv58GDhwo2q5Xr54gNJiXY8aMEYRnUhNbv\/MotZ6ek2pSXpSgc4+enwG0q8uLKVsEM3uecc8Mt2yfig2JIUofoRKFChUiZLKwu3\/S6d5J6cwHGX344Yd00UUXpRz2IEVcBQfyatiwIW3ZskWYkF7xZNkAM3ZFLo1d8VEeApt6Y\/WMcz35HWReTH4RM1OfcTeDo99WslmrfvtS69uak9Ce+vTpIxIi2t0\/mYR7J60ENrBlJRrYMq+Wlw1wOs\/yYtJByXwdxt08pjotxobEdIQNs04mwMCB33P+ViEmzEeYjmETGPrmxRTmTPm5L8Y9GtwzWasmJA0sTsyEcGjDLzBw4v921L8iJzAmMVMzwH87TGL+MTPxhN+1aqJPtOF62xF8WEkyJ0FgcOLLEIooTEh1YHgxmZqm\/tph3P3hZap27EjM+mJw2iN4FWl5EDf25z\/\/OXapeFQ\/2E31L6BpN+UNnDU1WLrt8GLSRcpsPcbdLJ66rcWexOSLYOdy2rRpIk4M5yODLrrAWM3I94ZeHrRonu3zYvKEKJAKjHsgsHo2qrtWPRvyWcG3TwyhEhMmTBA7l7rxYz5lylNdF5jW03JSwaxL7qobWhiF27vxYspm5DN\/lnHPHLtsntRdq9n0Yfesa1JE5ALD0SG14NhRu3btqGfPnr7yjGUquA4wakArovCX9KybaXdGn+PFZBRO7cYYd22ojFbUWatGO\/ypMdcD4Js3bxYZJ9SCtNQ44xjGJSHoVweYOGphkJ0XUxBT1rtNxt0boyBq6KzVIPp1NSfhzD9w4IDIYgHTcd++fXTuueeGem2bFzBx1cKYxIKYrnptMonp4WS6ltdaNd2fbM+VxBYuXEgjR46kuXPnUt26dcVlIThShEPdXhlZTQnsBUxctTAmMVMzwH87TGL+MTPxhNdaNdGHXRuOJIacX8jI2rt3b3EGUhbEjeFC3eHDhwutLOjiBkwcdyRVPHgxBT077Ntn3KPBPXYkhtTSOEOJDKxq3jCnvwcFmxsw6vEixIQhNixOhRdTNKPBuEeDe+xI7LvvvhMkVrt2bWrfvr1w5MNHtnbtWlq2bJkwMwsWLBg4Wm7ASFMS5yPjEBdmBYMXU+DTw7YDxj0a3GNHYoABqWaRCww\/cTMRwitAXMgzVrVq1VCQcgJGNSXjFFbB5mQo08K1EyaxaMYgliQGKE6ePClI7ODBg2JXEskNZdbWMKByAkY9YhSX4FbWxMKYEd59MIl5YxREjdiR2KFDh2jEiBHi8tuSJUsG8c5abToBE3dTEi\/Hi0lriI1XYtyNQ6rVYOxIDBrYjBkzxC3eV111VSiHve2QsgMmCaYkk5jWvA+kEpNYILB6Nho7EsMZSdzmjavS4A9TD3vjd2hpUZ2dVANc47grKUebF5PnvA+kAuMeCKyejcaOxHDcyO7YEd4EEfzYtVTvo\/R8wwwr2AET5wBXduxnONAGH2MSMwimj6ZiR2I+ZA+0qh0wxe79h+gzrqEVrIkFOiU8G2cS84QokAqxITF5VVunTp3EJbnWLBZ4+yjNSdUfFnXmVq+ZwIvJC6FgPmfcg8HVq9XYkBjMyG3btlGlSpXEtWvWLBZRm5NJCK1gTcxrugf7OZNYsPg6tR4bElMFDDKLBWLPcMmu33snVX\/YF4\/+PprR0uyVF5MmUIarMe6GAdVsLpYkFlQWC6S4xs5n27ZtqXHjxrR+\/XqaOXMmTZ48mUqUKJEHMiswuMkIJmXc\/WF4CV5MmrPfcDXG3TCgms3FjsSCzGKBSQZ\/28MPPyxCN3CovF+\/fiJrRpUqVRxJLCnxYWxOas76gKoxiQUErEezsSOxsLJYwGRdt24dzZs3T+Qps8aeARgUxKtt\/PQE\/e\/C\/eL34c1K0LXVg08FlM10wKYINkGSWFj2aEYtibg3bdo0BRbcRGEXx3xiYWSx2LFjB3Xp0kVcBYeD5h07dkzL26+ye5Kc+mxOhj2Vf+6PNbFosI+dJgYYwspigW+fAQMGUPfu3alJkyaO5mQSzkuqwvNiimYxMe7R4B5LEgMUQWSxyMnJoZUrV1Lfvn1TGTFgSlasWJE6dOjgSGIyyDWuqXesU4cXUzSLiXGPBvfYklgQcOTm5tKgQYNEiEWNGjXELh4SMMKkRKoftUhgkhTkKuXnxRTE7PFuk3H3xiiIGr8oEpPOfFw6goBa5PAHqSHc4owzzrAlsaQc+mZzMojl4a9NJjF\/eJmq\/YsiMT+gSWCS5tRnx76fUTZbl0nMLJ66rTGJOSAlgUlSpD6bk7rTPph6TGLB4OrVaixJ7Ouvv6Y33niDtm7dmkf+8847j2644YZQr2xL2s4ka2JeUz64z5nEgsPWreXYkRjixEaNGiV8Vg0aNBC3HckSNomtfecDwnEjlKTsTDKJRbOQGPfocI8diSFiH\/n1ccaxdOnSkSEDYFQSi3v6HXbsRzZVUh2zJhbNGMSOxHBIe\/z48XTnnXdGTmJPrnyXWk\/PESMT15uN7KYNL6ZoFhPjHg3usSOxU6dO0WuvvSZS5eBo0DnnnJNCBmEQMClVEzMo2ABM97+tobErPmISCwpkm3aZCEIEW+kqybjHjsRgTnbt2pU2bdqUNpp16tShOXPmULFixQIfaQBTq9+LtH7XUdFX3HOIsTkZ+JTw7CDJRJBk2WNHYp4zJaQKKoklIYcYk1hIE8OlmyQTQZJljyWJIbJ+586dtHbtWvrVr35FzZs3F2cpccbRGlkf1NQFMEfbzhHNJ2lnEvImeUKy7EHNaPd2k4x7LEkMGVeHDx8uQixJvXqDAAAPSUlEQVRkpgmEXdx66610zTXXhDLKFWs1oGMtxoq+krQzySQWyvSw7STJRJBk2WNHYtidBIHddtttVKZMGXFAe9iwYXTgwAFxMziysp57bvBJCcs3uJq+ajSASSxkTkjyYmLZQ54sP3UXOxJTM7tCRkli6v\/DcOyXu+oW+uaS2wVMSQqvYE0smoXEuEeHe+xIDFe1jRgxgpo1a0YXX3wxjRw5UmhiH3\/8Mc2fP1\/8v2DBgoEjVuaavnSiWmsmscCRztsBazMhA\/5Td0nGPXYkBky3b99Od999N+EIEi4OufDCCwmX606YMIHq1asXyij\/z60z6GSJi0RfSQqvYI0glOnBPrHoYE7rOZYkBilxCHzDhg3073\/\/myDk5ZdfHkp8mERIkljSwiuYxKJbXUnWZpIse2xJDFMRmhjILKwofXX6l7jzeTpVqETiwiuYxJjEMkGAScw\/ao63HaGpQ4cOCb\/YihUrRHxYvnz56Oabb6bevXunXa3mv2vvJ5J2z6T1jZI8IVl27\/kZRI0k4x47TUym4ilQoAD16NGDzj\/\/fHG12mOPPUYIgh04cGDa9WqmBzWJefVVDJI8IVl207NZr70k4x47EkOIBXYgkY6nZMmSqRHYt2+fiBGDhhZ0iEUS8+oziekt1iBrJZkIkix77EgMwa7jxo0Td0GWLVs2D4kh2LV\/\/\/6BB7vOf3s\/9Zx\/Oqts0mLE2CcWJE25t51kIkiy7LEjMaTi2bhxIz3zzDN0\/fXXU7Vq1Wjv3r30xBNPiCNHl112mZhJQWpjSbwchDWx6MhL9pxkIkiy7LEjMcSDwZTEmUm3smjRojwf79+\/X5ib2AyAGTpkyBBBetYD42vWrKFu3bqlnrVL7wMtDNoYStJixFgTi47MkkwESZY9diSWyRSUUf61a9em9u3b0549e8SFuPgHklLLggULqFy5ctSoUSPHrpJ4OQhrYpnMHLPPJJkIkix7LEkMYRWvvvoqPf300yJSH76x22+\/na644grbrK52efnHjh0rUvd06NAhz0ydNGmSONJUq1YtxxmMy0GwQ5nEQFfWxMwSk5\/WkkwESZY9liS2cOFCeuWVV6hz5850wQUXiKj9J598kjp16iT8ZNYC0vvmm2+Ewx+pq48fP079+vWjG2+8kf7whz+kqkNjw84nfGzwu+Hl7W4AL3bvP8QzScsjxr4ZP5Rjvm6SiSDJsseOxHBWcujQoXTvvfcKTUqW3NxcevTRR8V1bkWKFHGcgQiUHTNmjPgcGTDUtD1oe+LEidS2bVthZubk5Ij2HnnkEapevbp4Ro0RO3vvm\/Sv8Tebn+0Btwh\/IkzmJBaWPZpRSyLuTZs2TYG1e\/fu0IFzjNhXU\/GoO5BOf5eSQxt76aWXaMqUKSJ5IrS2\/Pnzu74YgmdHjx5NlStXTpmdSQ90xQtH9c1kYhax7CZQ9N8G4+4fM0cSQ8Q+NKNLLrmE\/vSnPwnzEGSDVNW4BQk7l1ZywucvvPACrVq1SgTDwgS1K1CZYUa2bt2azj77bNEuSAxaWatWrdI0sULvziVoY1wYAUYg3gjEShMDVMgdhuNF+FmiRAmRjge59qdOnUpVq1ZNQxPhFSAvXLirBshaK6Ie2u3Vq5dI6QNCg3kJ81M1XaVZCcc+F0aAEWAE7BBwPQCOB2Aegl0PHjxIhQsXFj4raE925f333xd3VOKMpVqgZeGSEVwBB\/Jq2LAhbdmyRWhfuNeyZs2aQrOrX78+jxIjwAgwAr4Q8CQxX61xZUaAEWAEQkaASSxkwLk7RoARMIsAk5hZPLk1RoARCBkBJrGQAefuGAFGwCwCTGJm8eTWGAFGIGQEYkliiBvDcSfEqSGv2Z133iku8Q3jijg\/+OvKibRGy5YtEyEkOMnQsmXLtGSTfvo1UVdXdrUv3Ai\/ePFiEUYT9Vj4kX\/Tpk3i9AmOzWFnHKdDEFQaVdGVHcfzcG558uTJdOLECZENBllh1CSlUb2DXb\/ywm3kIKxSpUpoosWSxLZu3SoSMmKyIS027rzEofNrr702NGB0OtKVE8eqcOAdpIx4O+RoQ8AvFpZTuIpO\/9nU0ZVd9oEQmz59+lCpUqUEGUdNYrryIyYRcYt9+\/YV4UHInoKjc2GkV3caH13ZX3\/9dRE8jtRWCCzHHDrrrLPEeeQ4FXxJI2Tq5ZdfppUrV9Ls2bOZxObOnSvGCBkzUBBLpg5mXAZQV85nn31W3BYl32fHjh2C0BDgG2RSSTecdGVHG4gVRDZfFEkKUZOYrvzQgBHniMBq5LSTSQoQ82jNcRfWvNKVXZIYNN9ChQoJEitdurQ4zhenAg3sueeeE3PjzTffFHL+ojUxTDJoXk2aNEllvojDordOGj9yYpBx0kEufCSEhEaAg\/TqwfiwJqYf2eWXCI6btWnThubNmyc0myhJzI\/8+KJAwamQd955R5jykN\/pSFzQY+BHdpiTSGWFbMooyL2HOQNtPo4F56qhJcLk\/UWT2LfffivS8tx0003Cf4ESRxLLRE6o3Th3ipMKIGq3hJBBTlI\/ssOHB60RN8F\/\/\/339Pe\/\/z1yEvMjP0gA5AX3BG6wh1b8wQcf2J79DRJz2bYf2eEXxj+krcKX3fTp04VvLEpT2A0jJrGf0MHBcxxBwqFzuchBYpiE+Fe0aNEw5ppnH37l\/PTTT0VKIun8tDt76tmpoQq6ssMBPXPmTCpfvrxwKmMc4kBiuvIDLpBYhQoVRE47FPgi4WsdP358JKa8ruyy3pVXXplKirB9+3axBiB7XNaBOiWZxBQ0cMAcPgDVJ7Z06VLxjRSVI9yOP3TlhFMct0Ndd911InMHMoJEXXRkl1oDsFdLmTJl6PHHHw\/VZLDipSM\/nrH6n0Bi0IRBblERgY7s0uxs0KBBHhKDvwkbK265\/KKaW0xiCvLq7g0csNBgsPjhJ4tT0ZUT2XCh1cAhG5Uz2Yqbruzqc3HRxCCTrvyoB9LCHAL5SlJD2E5UY6Eru2pOgrQk+UUpO5uTmgxkjaOBPwYEgO3lOBU3OfFNjwJHJ\/xfTz31VB7R7W53CvPddGSH7yWuJKYrP+qtW7dOaC\/YpcQ86t27d6SajK7s0MaWLFkidrE\/\/\/xzkSYeYS5RbAbpzE3WxHRQ4jqMACPACFgQiGWwK48SI8AIMAK6CDCJ6SLF9RgBRiCWCDCJxXJYWChGgBHQRYBJTBcprscIMAKxRIBJLJbDwkIxAoyALgJMYrpIcT1GgBGIJQJMYrEcFhaKEWAEdBFgEtNFKsJ6OLQsc2B5ieGnrldb6uc4roNASyQT9JNPDEG\/uEu0Q4cOfroLtC5SO3Xs2FFE8Ad1fAqnG3B9IUpQfQQKUoIaZxJLwGD5ISY\/df28Ohb+\/PnzfREY2jdBYohwxz9TZ07xLsjVZT2R4AcPnbo4e4qEhoi0DzM1jY5s\/011mMRiMprIboE0xCAhpFuRR0xwITG0BhRktsU5wLfffpsmTJgg0i1XqlSJevbsKc6WIuWMWhcaExLV4RnkA6tcubJIc9S4cWPbc4PqhcYXXXRRqu6GDRvS2lXzialHe3bt2iVSKA0fPlxobSAxHOZHBoYVK1bQpZdemkoPLY\/VTJs2TWSXwCXK9913n5Bv586doh7OyyIh5l\/\/+leREBAYIZMG2ka6JrQr85shgy7eGfhAhsGDB9PFF1+cNsIqiclD7r\/73e9o9erVAqdmzZo53mJvJWU3cmcSC2dxMYmFg7NnL8hA+s9\/\/lOkIUJRs3iq2hXO0EGDwJnMGjVqiEX8wAMPiMULQlPrIhcY6sGsARns2bNHLGw8X7du3TwywVwFwaE+iAa\/q3XdFivIA4sbZIm0N88\/\/7zIxos8ZCAdmc8LnyHBH2RGtlLIg9QyIDyYdsgKOmvWLPHueE\/I3aNHD2GKIqkksst+8803dNddd4ncZnjuhx9+EO\/+ySefiGR8yHQCXJAEEWcO8RnMWbXYkdiBAwdEfaTfBoZvvPGG+KKwnlNkEvOcyqFXYBILHXL7DkFiWOAgMeSCVw+7q8Qk0ytjceH\/IBssXJACTBa1LtqE2YQD6MjRjmJNAyOlWbRoEb333nuCEGXfyPhw5MgRQWxuJIbFj7Q2MnXSsWPHCBoZyASEhIst1LRK0iwFMUHrRKYS\/MSFHpAPxHf48GGReQK5s6CBQQ6kMxowYADJXGyQCe8LspwzZ44wOWUaapmPq0WLFqkMwfJdnTQx6bfbt2+f6Au4Ws1AJrGYLBhFDCaxmIwJUhHj2x9pe7CYcTEKyAMmoEpMMFFAdrgFB2mKsciQ2kXmNVfr4v\/QpqwFt9FY\/UF2viu1LScSU80xO+e926IHcU2ZMkWYmdDEypUrR5999hn97W9\/EySm3kMARznMS2hH8rYfNTUQiA8JHK0F2qFVLjsSUzMJu2VjYBKLyYJhEovfQHz44YdUvHhxQUzQsJCIEFekYSG\/9NJLqd1JaFe4Ng3aDzQY64JTicdOu7Lm+5dIIOf\/3r17hQaCPFvQaqAFQR5oUU4kJpP3QXuU2VPhh0Ma7rZt2xL8XerupNoOtCfIDzMQ2p+ahtxKYjCNoYVBU4XZjAJ8YLrCZETqZlUbhPzHjx+nAgUKpCXStCMxaGytWrUS7UITA8lDE5R9SZysJIbxwA0\/dju27BMLZ52xJhYOzp69QAODFoLcafny5RPaF4gN17phoUDbglaFRHkgMRAMstxiEcPRDe0FZhaIS9aFz0n1cyFFNpzg99xzT5pPDM+ATGB6wsG+bds28TtIFATlZk7CZIUWJEkPZijICUQAsnUjMZAT+oH5ByJCP4899hgdPXo0jyYGTRUmM26HgskIkxW\/Y8cSBAJ5pV8O\/WHTA3+H9qbjE0P7eHeZfBDygzClGS4HEOauNFtxgxXGRMpgvTyFScxz2hupwCRmBMbsG5G7kyAkLBL1Vh7s3MEEBJmA1OD8ljt00Jw2b94sfF\/QbL788stUXSxi7PLBpAI5qDuO1qym6BO7evADgTzVnULUdSMxa\/K+du3aCaKE2edmfuGdQURy1xLP4OovOO27desmbvZRr7UD4cn6eBdgBO0N7wmNSyY\/hPx+dyfr1atHy5cvT92IBAKD\/CCzrl27CkJGm0g1DtLFmJQtW5aQPlrKANJS6zKJZb8udFpgEtNBietEjoCdeQhzEhe2gtj83L3g5RMz9bJMYqaQdG+HSSwcnLmXLBGQu42I+4ITXt4a9fvf\/973zfBMYlkORswe\/38D4r4iBIWo9gAAAABJRU5ErkJggg==","height":184,"width":305}}
%---
