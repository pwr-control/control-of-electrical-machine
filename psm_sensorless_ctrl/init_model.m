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
use_observer_from_simulink_module_1 = 0;
use_observer_from_ccaller_module_1 = 1;
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
% open_scopes = find_system(model, 'BlockType', 'Scope');
% for i = 1:length(open_scopes)
%     set_param(open_scopes{i}, 'Open', 'off');
% end

% shh = get(0,'ShowHiddenHandles');
% set(0,'ShowHiddenHandles','On');
% hscope = findobj(0,'Type','Figure','Tag','SIMULINK_SIMSCOPE_FIGURE');
% close(hscope);
% set(0,'ShowHiddenHandles',shh);

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
%   data: {"dataType":"textualVariable","outputData":{"name":"l1","value":"     6.766822226281998e+01"}}
%---
%[output:45e82fc7]
%   data: {"dataType":"textualVariable","outputData":{"name":"l2","value":"     2.831309534039184e-01"}}
%---
%[output:3873a036]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["2.831309534039184e-01"],["6.766822226281998e+01"]]}}
%---
%[output:2f8df18e]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Afht","rows":2,"type":"double","value":[["0","1.000000000000000e+00"],["-9.869604401089359e+04","-1.570796326794897e+01"]]}}
%---
%[output:45ef9943]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Lfht","rows":2,"type":"double","value":[["1.555088363526948e+03"],["2.716608611399846e+05"]]}}
%---
%[output:20f05bf9]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000e+00","2.000000000000000e-04"],["-1.973920880217872e+01","9.968584073464102e-01"]]}}
%---
%[output:0445503e]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["3.110176727053895e-01"],["5.433217222799691e+01"]]}}
%---
%[output:04a70309]
%   data: {"dataType":"textualVariable","outputData":{"name":"kg","value":"     3.946150308374179e-02"}}
%---
%[output:69e958f5]
%   data: {"dataType":"textualVariable","outputData":{"name":"kw","value":"     1.254711180325163e+00"}}
%---
%[output:8e3d2c7e]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l1","value":"     6.069317568680815e-01"}}
%---
%[output:95bd65ef]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l2","value":"     3.449190983162578e+02"}}
%---
%[output:7d37a44a]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l3","value":"    -4.186041938481557e+02"}}
%---
%[output:49e44520]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAmgAAAF0CAYAAACXE8U0AAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQmYFcW1xw+KIDqsgkEWAVF0orIY16hEo0LiFjQKiIhKUGDQlzwEQYMxGtSHIBEX9ocIKEtccA9oogERQQygT0EFIQZZBGUVgoi875Spm56m7+2u7uo+faf\/\/X18ozNV51T\/6tyq\/6210r59+\/YRHhAAARAAARAAARAAgdQQqASBlpq6QEFAAARAAARAAARAQBGAQEMggAAIgAAIgAAIgEDKCECgpaxCUBwQAAEQAAEQAAEQgEBDDIAACIAACIAACIBAyghAoKWsQlAcEAABEAABEAABEIBAQwyAAAiAAAiAAAiAQMoIQKClrEJQHBAAARAAARAAARCAQEMMgEDCBGbPnk29evWi6tWr06RJk6hVq1YJl4Box44d1KNHD1q4cCF16NCBhg8fnisDl2\/w4ME0Y8YMql+\/fqJlW7p0KXXr1o22b99Oo0ePpnbt2in\/urxt27alsrKyRMvk56xv3740c+bMcuX1y6P\/buO9Ro4cSXPmzKHx48dTSUlJUNfW0jnrjI02atQoUuwUik1rhU7AkOZSWloqVjcJvCZcxEgAAi1GuDANAl4E0izQuLMfNmxY5E42bM17CbT169dTx44dac2aNdSvX79UCTRd3po1axqLEhvvpcXhqaeeKiICnGJK13nUslQUgeZ8D+eXjbCfDeTLHgEItOzVOd4YBPISgEAzCw7Nyz0KGcRKRRNoYRh4caooAo3fLUp8BIkhpKnYBCDQKnb9Ft3b6dElXXCvaUDnqEGnTp3olltuyb1nvhEWnUcndKdzdwo\/+9nP1DRkvvT5wDo73Xx5vUbQnO\/Upk0bGjNmjMruLKdu7LVd91RSPnHlZKq\/ybvf9+67785NeTrfLd9oSL6RG+f7u0cNgtStewSNy+KsB102p2133XIarxEL91Qcp1mxYoXniGGQaTuTd+UyOQWMm4Xpe3nFmdtHkHco1ED41ZfbftDPilc+r+lsPf0e5LPo\/my4Pzv8\/0E+Y85Y4ti\/99576brrrvMcvfVrU9inflf+b6nlDEXXCaDAOQIQaAiG1BDw6mi9Gv1C6dydlNcUjLbp7DALpYvS8Xj5KiTQnJXhFKf53tmZJkmBlm+aVv\/eLR6D1q2pQCtk19np5xNEXmI3X1r3lwU\/Bl4fLB1zfgLN771atmyZm\/Z1+vGzH3TdY5D6CiPQCtWD\/jIS5LPorFsvcRa03dA8mjdv7vkFxck2SPnco4g2RklT00CjIIkTgEBLHDkcehFwNtzOUSPdUeQTK84G0Sutbryd+b06VnenoDsAZydUaG2NM79TnHiVyU+guUf3vNg4y6UZRBFoepNA0ClOr44n39SUSd2arEHzGp1wlktzyVc3znLpOuPY1OvdvPI74y0fK6\/RRa84zNd5B30v96iQ3iTgx8BvKtKkvkymI53l0p8lfge9WUXXAW900L\/jv+vPotd7eY1iOsvk\/Mw6RWeQz5i7TdB5grYpXHYTPugZQMBNAAINMZEKAkGmzHQDqdO6R2ncHR7vBvTaqehsNL2+FbuFWJCF2Pl2H3rtiCwk0Lx2wHn599r9l6RA8xIHK1eu9NyBaVK3JgLNHbjukRQtRJw23R2zO5bee+89zx22XiOD+d7LKQQKiaGgoyv53iufQPMb2fPbZWlSXyYCJF+53LtQ8wmsfO\/rjAP3CJ2XQCv0GXP\/zR07Jm2KLleQ9iMVjTAKkToCEGipq5LsFahQI+\/1t3wNnjvtrbfe6jkN5DUlVKgMQRrYfAKt0Gih16ig1yhdEP\/sJ2mB5h7pmTdv3n7ruUzr1lSgFZre8hJo7rVpbmbPPPOMeod8j9eUmFuEeU39eU0tFhJoQd4rn2AplJfzFJrmNK0vGwLNzdr0s1ho2tRLoJl8xtx1dPnllwduU\/R7BR2Vzl7Ljzf2IwCB5kcIf4+dgGmnAIHmfdZV0gLNWW89e\/akxYsX73eummndmgg0Z8esRccPfvCD\/aYoC4nnOASa\/sB4CQfnCE0+gRb0vSDQxpNz1JZ5sAD\/8Y9\/nBs5h0CLvfmGgxgJQKDFCBemgxMwmVaJOsXpVSrTb+1uG\/mm0fTvhw4dmjt01W8Xp\/vA0Xzvy79fu3Zt7vyrpAWac9SuYcOGtG3bNoXFvVvNpG5NBJp+X2cn7LVOycYUp8koj1d8eZUhn0AL+l75BFq+qcSgn0aT+gozgqaFlD6EmMvbv3\/\/XNyYfBbfeustNSXtFL5+a9AKjaCFneIsxNarPoPWBdJlmwAEWrbrPzVvb7IwOd8an6CbBLxEgEmn4HVae76F6M7pJj29ZirQvNh4LbjWnRVXql5r5T6OId8xG6abBHTguKfzvDo\/k7oNI9C8drJy+WxtEsgnhAqtDeQjIvyEo59A83uvfOXyEqn50no1ACb1ZSLQvGKWP0vuz61zR6V7+tjN3Bnz7s8Xv1vQETSvdzbZJFBolDboEoXUNMYoSGoIQKClpipQkEJb+\/3OvXLSczbq+c6LcjfeUQUa28t37IDbl6lAc3auXlHiteM0XzT5CbRCi6y9bObrxNxpg9atn3jWdvk9eDpTXwvlVbYg544dd9xxtHz58nIjMIXWcHkd7+AedSm0Jsoputzs9MiSyXvl20AQ9B3yxUnQ+jIRaOyrEBu\/XbNeIpP96123Xu8SVKB51QXb0yPDfPVYvi89Tr\/uLyimfNALgICTAAQa4iFVBNwNuN9Btf\/1X\/9FvXv3Vnc38hP0oFr3N3MbAo39ewlCr7su3XdxBvmW7e50vdh4jWg5D\/P1E2jujspvx5+z0\/U7YytI3RbaDet1cLDbZtDDZ3VZvTY2eIntQqw5vXta1ysOvFi6y6\/jN+h7uf04BYI7Fvzqx90QBKmvMALE64uM83Nr+ll022NbRx999H67cYN8xtyj886NRvl2AGtuXjt2Cx1mnKqGF4VJJQEItFRWCwpViECQhhYEQSAIgSg77ND5BiFc3GmCHoWS7y29zrQrbiIofZIEKrRA48Z32rRpvpcY55uainrpb5IVmSVfEGhZqu3o7+rsZPMtJvc7vDVfKbTAC5s\/+tvBgg0CztFp50hZ1A0XiA8btZNdGxVWoGnRVbNmTV+B5t5FlN1wKI43h0ArjnpKUykLranicnrd3Rmk\/CbtTBB7SCNDoND6US5RmC\/rzqnasPElQwNe00KgQgq0fDvqCn0LnjNnTu64grRUDsrhTQACDZERhoDXwnnTdVlefnU8ohMOUyvpyRPkAnqT0mrRV1pair7FBBzS5ghUSIGmrw5p06YNvfTSS74jaNzA8qOPGkB8gAAIgAAIgAAIgIAkgQon0JzTlbxDy28Nmv7WxOfx8HZ7\/WBNiWRYwjcIgAAIgAAIZJtAhRJoWmx17tyZysrK1N2EfgJND0Ofd955uRE0badBgwa+Q9Nr1qzJdgTh7UEABEAABEAgIQK80ScrT4USaF5X3\/gJtHwVrXfvFFpXwuKMryhZsGBBVuIF7wkCIAACIAACYgROO+004qvzsiDUKoxA89qJGWQELV+U6ZE1vgSaR+O8nrfffpu6dOmigoVPnMaTHAEWxSNGjAD75JCX8wT+QuCJ1BdCxD74yxGQ86xjnzf1QaDJ1YOxZ79t9PlOmLch0LISLMaVEmMGLY7BPkbIBUyDvwx39gr2cuzBH+yTJFBhRtC8oAUZQcs3lRnkbDQ0lEmGanlfYC\/HHp0U2MsSkPWOtkeOf9bYZ16g6TPT1q5dmzuOw2vjQKEpToziJP+BXb16NU2cOJEGDRpElStXTr4AGfcI\/nIBAPZy7Nkz+Mvxh0CTY2\/ds9cIGv9uzJgx+11u7J4iDTIlmrVgsV5BEQz+61\/\/onXr1lHjxo0h0CJwDJsV\/MOSi54P7KMzjGIB\/KPQi5Y3a31uhR5BixYK\/rmzFiz+RJJLgUYyOdZensBfjj\/Yy7Fnz+Avxz9rfS4EWoRYy1qwREBlPSsaSetIjQyCvxEuq4nB3ipOY2Pgb4zMWoas9bkQaBFCJ2vBEgGV9axoJK0jNTII\/ka4rCYGe6s4jY2BvzEyaxmy1udCoEUInawFSwRU1rOikbSO1Mgg+BvhspoY7K3iNDYG\/sbIrGXIWp8LgRYhdLIWLBFQWc+KRtI6UiOD4G+Ey2pisLeK09gY+Bsjs5Yha30uBFqE0MlasERAZT0rGknrSI0Mgr8RLquJwd4qTmNj4G+MzFqGrPW5EGgRQidrwRIBlfWsaCStIzUyCP5GuKwmBnurOI2Ngb8xMmsZstbnQqBFCJ2sBUsEVNazopG0jtTIIPgb4bKaGOyt4jQ2Bv7GyKxlyFqfC4EWIXSyFiwRUFnPikbSOlIjg+BvhMtqYrC3itPYGPgbI7OWIWt9LgRahNDJWrBEQGU9KxpJ60iNDIK\/ES6ricHeKk5jY+BvjMxahqz1uRBoEUIna8ESAZX1rGgkrSM1Mgj+RrisJgZ7qziNjYG\/MTJrGbLW50KgRQidrAVLBFTWs6KRtI7UyCD4G+GymhjsreI0Ngb+xsisZchanwuBFiF0shYsEVBZz4pG0jpSI4Pgb4TLamKwt4rT2Bj4GyOzliFrfS4EWoTQyVqwREBlPSsaSetIjQyCvxEuq4nB3ipOY2Pgb4zMWoas9bkQaBFCJ2vBEgGV9axoJK0jNTII\/ka4rCYGe6s4jY2BvzEyaxmy1udCoEUInawFSwRU1rOikbSO1Mgg+BvhspoY7K3iNDYG\/sbIrGXIWp8LgRYhdLIWLBFQWc+KRtI6UiOD4G+Ey2pisLeK09gY+Bsjs5Yha30uBFqE0MlasERAZT0rGknrSI0Mgr8RLquJwd4qTmNj4G+MzFqGrPW5EGgRQidrwRIBlfWsaCStIzUyCP5GuKwmBnurOI2Ngb8xMmsZstbnQqBFCJ2sBUsEVNazopG0jtTIIPgb4bKaGOyt4jQ2Bv7GyKxlyFqfC4EWIXSyFiwRUFnPikbSOlIjg+BvhMtqYrC3itPYGPgbI7OWIWt9LgRahNDJWrBEQGU9KxpJ60iNDIK\/ES6ricHeKk5jY+BvjMxahqz1uRBoEUIna8ESAZX1rGgkrSM1Mgj+RrisJgZ7qziNjYG\/MTJrGbLW50KgRQidrAVLBFTWs6KRtI7UyCD4G+GymhjsreI0Ngb+xsisZchanwuBFiF0shYsEVBZz4pG0jpSI4Pgb4TLamKwt4rT2Bj4GyOzliFrfS4EWoTQyVqwREBlPSsaSetIjQyCvxEuq4nB3ipOY2Pgb4zMWoas9bkQaBFCJ2vBEgGV9axoJK0jNTII\/ka4rCYGe6s4jY2BvzEyaxmy1udCoEUInawFSwRU1rOikbSO1Mgg+BvhspoY7K3iNDYG\/sbIrGXIWp8LgRYhdLIWLBFQWc+KRtI6UiOD4G+Ey2pisLeK09gY+Bsjs5Yha30uBFqE0MlasERAZT0rGknrSI0Mgr8RLquJwd4qTmNj4G+MzFqGrPW5EGgRQidrwRIBlfWsaCStIzUyCP5GuKwmBnurOI2Ngb8xMmsZstbnQqBFCJ2sBUsEVNazopG0jtTIIPgb4bKaGOyt4jQ2Bv7GyKxlyFqfC4EWIXSyFiwRUFnPikbSOlIjg+BvhMtqYrC3itPYGPgbI7OWIWt9LgRahNDJWrBEQGU9KxpJ60iNDIK\/ES6ricHeKk5jY+BvjMxahqz1uRBoEUIna8ESAZX1rGgkrSM1Mgj+RrisJgZ7qziNjYG\/MTJrGbLW50KgRQidrAVLBFTWs6KRtI7UyCD4G+GymhjsreI0Ngb+xsisZchanwuBFiF0shYsEVBZz4pG0jpSI4Pgb4TLamKwt4rT2Bj4GyOzliFrfS4EWoTQyVqwREBlPSsaSetIjQyCvxEuq4nB3ipOY2Pgb4zMWoas9bkQaBFCJ2vBEgGV9axoJK0jNTII\/ka4rCYGe6s4jY2BvzEyaxmy1udCoEUInawFSwRU1rOikbSO1Mgg+BvhspoY7K3iNDYG\/sbIrGXIWp8LgRYhdLIWLBFQWc+KRtI6UiOD4G+Ey2pisLeK09gY+Bsjs5ZhyKxVNPTZRbRk0BnUqFEja3bTaggCLULNQKBFgBcxKxrJiAAjZgf\/iAAjZAf7CPAsZAV\/CxBDmrjk0cU0b+UWeq\/vMRBoIRlmJhsEmlxVo5GUY8+ewV+OP9jLsUfsy7KHQJPlX1TeIdDkqgudlBx7dFJgL0tA1jvaHjn+EGhy7IvOMwSaXJWhkZRjD4EG9rIEZL2j7ZHjD4Emx77oPEOgyVUZGkk59hBoYC9LQNY72h45\/hBocuyLzjMEmlyVoZGUYw+BBvayBGS9o+2R49968Hz67Kt\/YZOAXBUUj2cINLm6QiMpxx4CDexlCch6R9sjxx8CTY590XmGQJOrMjSScuwh0MBeloCsd7Q9cvwh0OTYF51nCDS5KkMjKcceAg3sZQnIekfbI8cfAi0m9lu3bqVRo0YR\/wz71KxZkwYOHBg2u\/V8EGjWkQY2iEYyMKpYEoJ\/LFgDGQX7QJhiSwT+saH1NQyB5osoXIL169dTx44dac2aNeEMEKmTg+fMmRM6v+2MEGi2iQa3h0YyOKs4UoJ\/HFSD2QT7YJziSgX+cZH1t1un7+sqEW4S8GdllEILtEGDBlG7du2M8nLi2bNn0+DBgyHQjMlVzAxoJGXrFfzl+IO9HHv2DP5y\/CHQYmK\/ceNGuummm9S\/s88+29jL3Llz6ZFHHqHp06cb540rA0bQ4iLrbxeNpD+jOFOAf5x0C9sGezn2EGiy7CHQYuK\/d+9e4n9VqlSJyUPyZiHQkmeuPaKTkmOPTgrsZQnIekfbI8cfAi0m9jzF2aVLF2rcuDGVlZXRySefTAceeGBM3pIxC4GWDGcvL2gk5dhDoIG9LAFZ72h75PhDoMXEnndv\/uEPf6AXX3yRvvnmG6pXrx5dd9111LlzZ6pdu3ZMXuM1C4EWL99C1tFIyrGHQAN7WQKy3tH2yPDnGwR4Fyc\/2CQQUx3s2LGDXn\/9dZowYQK99957ykvLli2pd+\/edM455xTVFCgEWkxBEsAsGskAkGJMAv4xwvUxDfZy7PHlRI49BFrC7HnjwFNPPUUTJ04k\/u+SkhK68sorqXv37tSwYcOES2PuDgLNnJmtHOikbJEMZwf8w3GzkQvsbVAMbwP8w7OLkhMCLQq9CHn37dtHH3\/8MY0bN47+\/Oc\/086dO6lZs2ZqVO3SSy9N7agaBFqESo+YFY1kRIARs4N\/RIARsoN9BHgWsoK\/BYghTECghYBmOwuvT5s\/fz7dcccdyvSMGTOofv36tt0UtDdy5EiaNm2ar28ItESrpZwzNJJy7Nkz+MvxB3s59oh9OfYQaHLs1cYBFmY83ck\/+UgOPtD23nvvJb7iKaln6dKl1K1bN+XTTxxCoCVVK\/v7QSclxx6dFNjLEpD1jrZHhv+bK7bQpSMXK+fYJJBAHbAIW7JkCU2dOjU3tSm5u5M3MPTo0YMWLlyorpWCQEsgCEK6QCMZEpylbOBvCWQIM2AfAprFLOBvEaaBqanvrKc+U5dBoBkwM07K6834Pk4eKXv++efpyy+\/VGvMLrzwQurZsye1aNGCKlWqZGzXRgae2uS7Ptu0aUMvvfQSBJoNqDHZQCMZE9iAZsE\/IKgYkoF9DFANTIK\/ASyLSYfMWkVDZq2GQLPINGdq165dNHnyZJo0aRKtXbtWibDS0lJ1FtrPfvYztYNT8uG7Pvv376\/KN2\/ePKxBk6yMAL7RSAaAFGMS8I8Rro9psJdjz57BX4b\/5AXr6NfTl0OgxYFfX5a+ZcuW1B2locvGh+byLQemmwSefPJJNSXKT9IbGuKoq2KwiUZStpbAX44\/2Muxh0BLnj33z\/z0fm4jzVu5BQItjirgEbTVq1dT8+bNU3dsRt++fdWo3vjx49VInqlAc\/LiDQbXXnttHAhh00Fg9+7d6uw8FsSVK1cGm4QJgH\/CwBH7csBdnhH7yVbF448\/rma2trUbQt8dUhcCLQ78epRq0KBBanem6cNTkIMHD1ZrxGw+zqnNVq1aKdOmAm3o0KG5g3VZMGAUzWYNedtiwb9hwwY1cgmBFj9vtwfwT5659gj2cuzZM\/gny5+1wzvLVlPPV\/flHGMXp+U6SKtA49GzmTNn5n3bfv36qWlPrwfHbFgOEgNzmOYxgBVDUvCPAWpAk2AfEFRMycA\/JrAFzDqP2OBkEGiW60ALNN7BGfbh0RLbI2heZTEdQeMy6TVoYd8N+cwIoJE042U7NfjbJhrcHtgHZxVHSvCPg2phm2+u2EyXjlyiEh2wcxMtGXRGJvrcSvv43IsEnq1bt9KoUaOIf4Z9+PDYgQMHhs0eOB8EWmBUYgnRSIqhV47BX44\/2MuxR+zLsL\/k0cW5DQKVNy2nv997CQSaTFXIe4VAk68DvxKgk\/IjFO\/fwT9evoWsg70cewi05Nk7r3hi7yVv3k9vPTMOAi35qiguj1iDJldf6KTk2KOTAntZArLe0fYky995gwBPb9aYPUAtdcrCsqLEpjiTrdJkvEGgJcPZywsaSTn2EGhgL0tA1jvanuT48+gZ37\/JP\/mpvOkjNYIGgZZcHRStJwg0uapDIynHHgIN7GUJyHpH25Mcf+foGXv97Y9206N3lEGgJVcFxesJAk2u7tBIyrGHQAN7WQKy3tH2JMe\/9eD5udGzI+scTKPPr0RdunSBQEuuCorXEwSaXN2hkZRjD4EG9rIEZL2j7UmG\/10vrqQRf\/0s5+z5sjbEOzgh0JLhr7zwKR+bN2+mvXv3Up06deiAAw6gPXv2pO46KC8kEGgJBorLFRpJOfYQaGAvS0DWO9qe+Pm7pzbPbF6LXujThrLW54ptEmBh9uqrr9Idd9yh7lTkHRkzZsygqlWr0q9+9Ss6+eSTiU\/xr1KlSvzRENJD1oIlJKZYsqGRjAVrYKPgHxiV9YRgbx2pkUHwN8JlnNh9awBPbfLoGf\/MWp8rJtD+9re\/Uc+ePemkk06iY445hl5\/\/XUl0Piy8ttuu41mzZpFfG8nXz6e1idrwZKmekAjKVsb4C\/HH+zl2GP0OF727jPPWJQ90rmUzjq6lnKctT5XRKDt3r2bevXqRd999x2NHj2a5s6dqy5CZ4HGF41\/8803xHdkbtq0icaPH69EWxqfrAVLmuoAnZRsbYC\/HH+wl2MPgRYfe\/e0Jnt69KpSuuqU+jmnWetzRQSavpfzxhtvpK5du9Ls2bPLCTSujSlTptDYsWNzoi2+sAhvOWvBEp6U\/ZzopOwzNbEI\/ia07KYFe7s8Ta2Bvymxwul51GzxZ9vo+kkflEvI05p65Ez\/IWt9rqhAu+aaa+iGG27wFGh83dJTTz1F06dPp3r16tmNCEvWshYslrBZMYNG0grG0EbAPzS6yBnBPjLCSAbAPxK+\/TI7j9LgP7qnNZ0Zstbnigg0PcW5c+dOGjdunJpXdk5xrlmzhq677jpq3LixmgLljQNpfLIWLGmqAzSSsrUB\/nL8wV6OPXsGfzv8J8z7nPo9\/XE5YyzOBrRvVm5aEwLNDm8jK4sXLyae4uQNAvzv5Zdfpt\/85je0YsUKevrpp9U6tDFjxtBPfvITI7tJJoZAS5J2eV9oJOXYo5MCe1kCst7R9oTnz9OZ81ZuoT5Tl+1nxLlbM5+HrPW5IiNoGv5bb72ljtlYtWpVufrgKc177rmHzj\/\/\/PCRkEDOrAVLAkgDu0AjGRhVLAnBPxasgYyCfSBMsSUCf3O0LMz4303TluVuBtBWCk1puj1lrc8VFWgMn89D492aH374IfHU5\/HHH692ch544IHmUZBwjqwFS8J4C7pDIylbG+Avxx\/s5dhj9Nic\/Sv\/t4munvC+Z0beoclTmizSgjxZ63PFBVqQSklrmqwFS5rqAZ2UbG2Avxx\/sJdjD4Hmz16Plg2ZtUpNZ3o9QaYzvfJlrc8VEWhbt26lUaNGEf\/0e+rWraumOk844YTUjaplLVj86irJv6OTSpL2\/r7AX44\/2Muxh0DLz56F2eS319IDr\/0jb6KwwkwbzFqfKyLQ+Gqnm266iT744APinZz8sBDjh6c7vZ6zzz6bHn74YapRo4bsp9PhPWvBkhrw2EklXhUQCXJVAPZy7CHQyrNnUfanv2+ge17+tKAou\/jEenTj2Y0CT2XmM5a1PldEoDH8hQsXUp8+fdRVTtdff33utgDevfmnP\/2J+By0Rx55hH74wx\/S888\/r47h4LS33HKL7KcTAi0V\/NFJyVYD+MvxB3s59lkXaHr6cuo764hP\/i\/0mCz+D1qjEGhBSUVIt2PHDurRowe1aNGC7rrrLqpUqVI5a7xx4M4776R\/\/OMf6hy0atWq0ZAhQ9R5ac8++2wEz3azZi1Y7NKLZg2dVDR+UXODf1SC4fODfXh2NnJmjT+Lso82fE0P\/fWzvGvKNFcWZSM6HkfN6laLPFrmVVdZ63NFRtD0VU88gtapUyfPzwzfIPDoo4\/mrnp65pln6MEHH6Q5c+bY+IxZsZG1YLECzZKRrDWSlrBZMwP+1lAaGwJ7Y2RWM1Rk\/izG+Hlz5RaaunCdryDjtCzK7r+8BR1X\/9BYRJmz8rLW54oItM2bN1P37t2pefPmdO+991KVKlXKfYB4mvP222+nlStX0oQJE6h27dpq\/RkfZvvKK69Y\/bBFMZa1YInCynbeitxI2mYVhz3wj4NqMJtgH4xTXKkqGv83V2yhXXv2Bhohc46UPdK5VAmyoEdk2KiPrPW5IgKNK4ovQh82bBhdeeWVasMAn33GD4+u8dqzadOmqfVmvXv3VuvVbrvtNnVGGgu1tDxZC5a0cOdyVLRGMk1sg5QF\/INQiicN2MfDNajVYuWvR8f459N\/30ArNu4MNEKmR8kuKD2MftHq8P0uMA\/KzUa6rPW5YgKNR8lYoD322GO0d+\/ecnXHh9TyxoF+\/fqpjpgvVF+7dq0SdaWlpTbq2YqNrAWLFWiWjBRrI2np9cXFjH8EAAAgAElEQVTNgL9cFYC9HPti+3LIo2OffPE1PbP4i8BiTAuyn7SoTVeeVD\/xUbJCtZu1PldMoOlK4CM3\/vKXv9DSpUvVr3iU7Nxzz6WGDRuq\/9+1a5caVWvQoEHqLk3PWrDINovlvaOTkq0N8JfjD\/Zy7NMo0PTIGJfthfc20p8\/2GQkxrQgu6RlPWr\/w7qpEmTums5anysu0GQ\/atG8Zy1YotGymxudlF2eptbA35SYvfRgb49lGEvS\/HlUjB\/TaUr9rrxmrHHtg9V0JS\/sP+voWmEwiOTJWp8rJtD4KI1PP\/2UZs6c6Xk4Ld\/L+fnnn9OIESNy69NEIqKA06wFS5r4SzeSaWIhURbwl6D+vU+wl2OfFH\/nerGX\/28Tvf\/5duNRMT0yxj\/5kNiWDaunenQsSK1mrc8VE2i8I7Nv377Ea9G8Ht7ZefLJJ6tNAbyLM41P1oIlTXWATkq2NsBfjj\/Yy7GPQ6Dpw1\/nf7qF5nyyOZQQc4qxK076AZ3Tok7RizGvWs5anysi0L7++mvq2bOnWvj\/0EMPUZMmTdRGgDZt2qjbBaZOnUoTJ05Uh9SeeOKJsp9GjKClkj86KdlqAX85\/mAvxz6KQPvPKfzr6bOvdoUWYlqM8TTlLRc0ocoHHFAhxRgEGpGIQNMH1fIRGzfffLOqh9\/\/\/vf0xRdfqClNfnh0rWrVqjR06ND9bhqQ\/Xj+x3vW1HxauEdpJNP0DsVcFogEudoDezn2+doeFl+8tit30OuKzeoqpH9u\/lfud2FKrdeL\/bh5LTr76NqZEWL5WGWtzxUVaL\/5zW\/o8ssvV3UxZcoUeuGFF2jcuHHqQnT+\/yeeeIImT56cu0g9TIDHmSdrwRInS1Pb6KRMidlND\/52eZpYA3sTWvbTvvZ\/62jLli20dV8JzXxvU2QR5hwRu7RVPSqtX5J5IQaB9j0BEYG2bds2NaV50kkn0YABA1RBXnvtNTWKxsKsadOmSqixOJsxYwY2CdhvY4reIjop2SoEfzn+YB8\/ez0d+dL7G+n\/1u6wIsKcQoyPtPjhERBipjWZtUEREYHGlfLAAw+oQ2pvvfVW6ty5s1qP1rVrV3WzwM9\/\/nMaOHAg8U5PfdWTaUUmkT5rwZIE06A+0EkFJRVPOvCPh2sQq2AfhJJ3Gj0Fqacj\/7XnO3r49c9o9ZfR1oQ5velpST7o9fRm3x9hkfSVSOEJpTtn1vpcMYHGo2i8\/ox\/sgirVasWDR48WG0OYGFWqVIluvPOO6lbt26pjZisBUuaKgKdlGxtgL8cf7D3Z69HwDglH9y6dM12a6NgbLNBjcrUrF4JHX34IfTLNj\/I3UeZ5L2U\/hQqXoqs9bliAo1Dh4XY9u3bqXr16kqQ8ZVPb731Fs2dO5fat2+vpkD592l9shYsaaoHdFKytQH+cvzBntTCez0a9uySL+jjDV9bFWB61It3SrY5sga1Kz0sNxJ2+CFE69ato8aNG1PlypXlAiGDnrPW54oKtELxxWJt69atVLNmTeK7OdP4ZC1Y0lQH6KRkawP85fhngb0WYMvXf02L\/7lNibGoOyLdNaZHu85qXosublmPSqpWDjQSlgX+ctFd2HPW+lwRgaaP2Rg0aBC1a9fOs0ZeeuklGjJkCDYJpPWTIlwuNJKyFQD+cvyLlb3zzkhNjw9nnbJgnfrfeSu\/v8LI1qMFGP88tWlN6nZ6g5zpKFORxcrfFldJOxBoMdHnq5t4+nLnzp1qi\/KDDz5Il112GbVq1Wo\/j5yW16Vx2unTp1O9evViKlU0s1kLlmi07OZGI2mXp6k18DclZi992tlrIfb3f26j1z78kv4Rw+gX09Qii6chT2tWU52e7\/x9FBFWqLbSzt9epKXPUtb63ERH0CZNmkR33XWXWnvm9\/C0Zv\/+\/dVxHGldh5a1YPGrsyT\/jkYySdr7+wJ\/Of5S7J3rvjbv3EOvLf+SPt24y\/rUoyarBdaZzWvReccdRic3qaGmOqV3RErxl4u49HjOWp+bqEDjezd5XdmGDRuoV69e9Otf\/5rOPvvs\/Wqf7+Hk+zfTKsx0gbMWLOn5mOLCaOm6QCclVwNxsHeKr8+3\/Ive\/nQrrdi4M3bxxaNfrRtXV4vwub13TkvKES7sOQ7+aX3XtJUra31uogJNVzZvAPjqq6\/UjQF8nVOxPlkLljTVExpJ2doAfzn+puz\/I7720Uvvb1IHr\/Jje82Xk4g+C6zFDw6h5vUOoYtPLL9MJa7pxyRqxZR\/EmXKio+s9bmJCTQtyvhn0IenOevUqYNdnEGBZSgdGknZygZ\/Of6a\/b5D66ljHliA8U5H3vEYx25H95tq8cU\/Wxx+KP2oSQ2VpBhGv2zUGmLfBsVwNiDQwnHzzaV3bq5Zs8Y3rU7QqFEj7OIMTCtbCdFIytY3+MfH3znduOKLnWqq0fZBq\/lK7xRfLRtVp+OPKFFJzzr6+xPx8WB5hWQMQKDFRH\/Xrl00b9484h2aQR+e\/jzzzDOpWrVqQbMkmi5rwZIoXB9nEAiytQH+5vydwqthrao0ecE6emf1VmUozulGtu\/c8ciL7c87rg4dWadaKhbdm5OUzYHYl+OftT43sSlOuSqNz3PWgiU+kuaW0UiaM7OZA\/y\/p6mPlNC7C1du3Elvr9pKb\/37TK+4hZcWX7zYvkmdg6ltizrUoOb363qzMuVoM66D2ELsB6EUT5qs9bniAm3z5s305ptv0rvvvku8e7NNmzZ0+umnq12caX+yFixpqg80krK1UZH5ex2o+tXOPTTl7XX0UQxXCuWrSeeo1\/mlh9GPjvx+rRdfNbRx40Y64ogj6KjDv5+CxJMcgYoc+8lRDOcpa32umEDjzQJjx46l4cOHqzs4nQ9vDujTpw+VlZUp0ZbWJ2vBkqZ6QCMpWxvFxt9LdDHB15Z9Se9+Fs9VQkGEF6\/tOuOoWnTAv+8cDjLqVWzsZSPVvnfwt880qMWs9bliAu2pp56iAQMGUNu2bemWW26hpk2bqjpavXo1PfDAA+rWgaFDh9Kll14atO4ST5e1YEkccAGHaCRlayMt\/LXwYmGj\/3vH7r006e219EECx0k4a8E54nXOsXXo1CY1yp3txWltHC+RFvayESjnHfzl2GetzxURaDt27KAePXpQSUkJPfzww\/ttAuANBTfffLMaWRs9enRqz0rLWrDIfSz394xGUrY24uTvJbr4bZ\/++wb660dfqRe3fXF2IZp6Z2OzutWoXkkVOvdYXmB\/cLksNoRX0BqNk33QMmQ5HfjL1X7W+lwRgaaP3Ljxxhupa9eunrU9ZcoUNQU6Y8YMql+\/vlxEFPCctWBJUyWgkZStDVP++USXxBSjcyRLLa4\/rBqd\/e+pRr3Y39ZoVxy1ZMo+jjJk2Sb4y9V+1vpcEYHGC1w7deqkLkvnkTKvh0fWnn32WVyWLvdZSLVnNJKy1cP83\/3on7mF6s41Xn\/7+CtasGprbJdk+4128d9ZeJ1+VE1qe0xtqkSVVJYg67tkqQbzjtgPximuVOAfF1l\/uxBo\/owip9izZw\/17duXli1bRuPGjaNmzZqVs7lq1Sp1SXppaanaRHDQQQdF9hmHgawFSxwMw9pEIxmWXOF8+RbT89Qin9mVxEn17hI613Ydffgh9NNj61DtQ75vEyqK6DKpTcS+CS37acHfPtOgFrPW54qMoHFlvP\/++9S9e3d1cO0FF1ygDqTlhw+zffXVV9W6M57i5GM30vpkLVjSVA9oJM1qw0t4vf7RV\/TU3zcoQ0mu6XIKKx7pYpHFOxkPPKCSGvlyrudKcm2XGVG51Ih9OfbsGfzl+GetzxUTaFzFH330Ed1+++20ZMkS2rdvn6r1SpUqUevWrenee++lY489Vi4SAnjOWrAEQJJYEjSS\/zkkVUPf\/e139OziL2juis2pEF0nHVmDDqlyIESX5U8FYt8yUENz4G8IzGLyrPW5ogIt17Hs3k18YC0\/fEAtj54Vw5O1YElTnVTURlJfB6RHjvhU+rc\/3UKffLFT4U\/iZHpnPTunF\/m\/T2lak6pWPoDqVdtH1fbuUGvQ+MJujHQl9+moqLGfHMFonsA\/Gr8oubPW54oINN4kMGHCBHXGWYsWLYgPpi3GJ2vBkqY6KqZG0n0H49OLN9DfPpYZ5eI6dIqupryD8ZjatPe7fUYjXcXEP01xa6MsYG+DYngb4B+eXdScWetzxQQa7+LkQ2kPO+wwJdSuu+46atSokZriLJYna8GSpnqRbCSd9y8yk3XbdtP8lf8Z5Up6PZdTdLH4anH4oXR+aR0qqVpZVVkca7ok+acpDiXKAvYS1P\/jE\/zl+GetzxURaFy933zzDc2fP58mTpyofvL\/825OffxGvXr15KIgoOesBUtALIkks91IOkXXnr3fqet\/5nyymXhppKTg4kXzzetVo8P4gNQW3x+QmoazumzzTyRoKogTsJetSPCX45+1PldMoDmr2C3W+BgOPmKDd3lefPHFqb2PM2vBIvex3N9zkEbSKbqWrtmuhJa+\/kdadLVqVJ3OOKom1ax2kOip9GHrNAj\/sLaRrzABsJeNEPCX45+1PjcVAs1Z3du2baPHHntMjaxVr14dNwnIfRZS6VmLrk+\/2EFvfvg5badq9OG6r1VZJRfQ81quExqW0PFHlMQ2rZimCkEnJVcbYC\/Hnj2Dvxx\/CDQB9nw35+uvv67EGFcAPy1btlQjaHxGWpUqVQRK5e8ya8HiTyR8CudC+nVbd6ujIlZt2pX49KJzAX3LhiV01tG1qfrB8a3lCk9MNic6KTn+YC\/HHgJNln3W+lyxETS3KOOL0XkN2vXXX08XXXSROm4jzMM3FMycOTOXlS9bb9euXUFTS5cupW7dutH27dvLpTv11FNp\/Pjx6lJ3rydrwRKmPnQePfL15dff0P\/OW0v\/+HJXIiNeTsHFZWn3w8OodePq+13\/w3\/DURHBaxgiITgr2ynB3jZRM3vgb8bLZuqs9bkiAk1flr5mzRrizQDXXHONupezYcOGkeqSxdmiRYty06KzZ8+mXr16Ub9+\/aisrCyvbU7Xv39\/mjRpErVq1SpwGbIWLH5g3lyxRSVZuHor8Sn1ca3zYiH17bff0hHVK9PBBx9MPz+hLrVsWD1XvCxe\/+NXN7b\/jk7KNtHg9sA+OKs4UoJ\/HFSD2cxanysi0PhQ2qlTp1L79u3pqKOOsnK0hh4FGzp0aLkRMxZta9euLTgSNnLkSJozZ07BNBhBK0+AR8SeW\/oFzf7wS6sjYSyueOcir+k699g6dHj176e33aILjWSwBi2uVOAfF1l\/u2DvzyjOFOAfJ93CtiHQ5NjH4jmIQOM0\/PDF7CZPFoJFrw174+OvaMGqrZHEmBZZPzmmNrU\/vi7VcK3tMpliRCNpEqn204K\/faZBLYJ9UFLxpAP\/eLgGsZqFPtfJQWQELUhF2EjDI2PDhg2jQuvQ9HQrrzNbvnx5zm2HDh18BZsOlieffFIdsstP\/fr1bRRdzMbabd9SgxqVaeir\/6C3Pt1mLMg4L4+Andm8llpgr0e\/TMRXkJdHIxmEUnxpwD8+tn6Wwd6PULx\/B\/94+bqtcx+tH14W1aVLFzXjpfvcZEuTrLcKKdD02jNG6Se09NToeeedlxNkWrQ1aNAg0CYBZ5XxZoNrr7022VqM6I1F2brt39JLy3fQcx\/uCGSNhRivAbuktEQJOv5v\/pnUs3v3buIrw1gQ812QeJIlAP7J8nZ6A3s59uwZ\/JPl\/\/jjj6v14c4HAi3ZOojFG+8U7dGjh1qDxkd4mIxuaZFXaPRNj6Dxuje9wYF9mPiJ5cUNjP7Pn1fRQ3P+8w0lX1YeAbvpJ43o6HrV1HqwJMWYV5l27dpFGzZsUN+iINAMKtxSUvC3BDKEGbAPAc1iFvC3CDOAKR4w0aNoCxYsoBEjRmAELQC3okiiR8h69uxZcCen+2WC5CvG+XC9pmzIrFUFpy9ZhHU6uT6dfXRtOuvoWqmra0wzyFYJ+MvxB3s59uwZ\/OX4F2OfG4WWyBTn1q1badSoUeoapxNOOMGz\/O+++y49+OCDatoxyr2cQYSWVwGC5CumYNHC7KZpy9Rdjl4Pi7JHOpeqETLba8aiBKlXXjSStoma2QN\/M142U4O9TZrmtsDfnJmtHMXU59p4ZxGBptd4DRo0yPMQWT60lhf3v\/TSS4GnJvOdZeY3VZnv70HORiuWYJn6znriETMvYcZCbGqPlnRolQNTL8qcAY9G0sbHP7wN8A\/PLmpOsI9KMFp+8I\/GL0ruYulzo7yjM29iAo0vRL\/99tvpmWeeCVz2n\/70p\/Twww9TtWrVfPPo9WacUJ\/+r0fB+OL1fDcCeK1T89o44FWAYgiWSx5d7DmVycJsQPtmdNUpxbnrFI2k70ci1gTgHyvegsbBXo49ewZ\/Of7F0OfapJOYQONCr1q1inhHBh9Uy3dvtm7d2vP2AL4kvU2bNnTWWWepC9NNHvdVT+5bBPjojTFjxux3a4BfvmITaDxq1mfqsv2Kracx07iuzKSe0Uia0LKfFvztMw1qEeyDkoonHfjHwzWIVQi0IJQipgmyBi2ii0SypzVYWJixQHM+FUWY6XdCI5lIiOd1Av5y\/MFejj1G0GTZp7XPjYtKoiNocb2ElN20BQuvMbt05OJya81YmPE0Jk9nVqQHnZRsbYK\/HH+wl2MPgSbLPm19btw0EhNoetSMX+jqq6+mJ554gvh3hZ6aNWtS7969iX+m8UlTsPBF5SzO3KNmz5e1KarF\/0HrGZ1UUFLxpAP\/eLgGsQr2QSjFlwb842PrZzlNfa5fWW38PTGBpnducqF54f\/NN99MfG1DoYcPITU9YNYGlKA20hIsvENzyKzV5Yp9aat6NPFa7yNMgr5fmtOhkZStHfCX4w\/2cuwxgibLPi19blIUEhNoSb1Qkn7SECxe4uzRq0qLdndm0PpDJxWUVDzpwD8erkGsgn0QSvGlAf\/42PpZTkOf61dGm3+HQItAUzpY3Ds1K9pGgEJVg0YyQuBayAr+FiCGNAH2IcFZygb+lkCGMCPd54YocqQsIgJNr0fDGrTwdeclzor5XDNTEmgkTYnZTQ\/+dnmaWAN7E1r204K\/faZBLUKgBSUVIZ1ej4Y1aOEg8m7N1oPn5zJnaeRMvzQayXCxYysX+NsiaW4H7M2Z2cwB\/jZpmtmCQDPjZTX1vn37aNOmTTRt2jR67rnn1F2c+e7qtOo4pDGpYGFx5ry2iXdqFvvBs6ZVgEbSlJjd9OBvl6eJNbA3oWU\/LfjbZxrUolSfG7R8ttOJTHH6vQQLtTvvvFPdOMCXpR900EF+WUT+nnSwsCjjQ2jnrdySe98sbAjwqlw0kiIhn3MK\/nL8wV6OPXsGfzn+Sfe5cm\/6vedUCjQu2PTp0+nRRx\/FMRuOCHGvO\/vv85rQHRcdJR1DIv7RSIpgh0CTxa68I\/ZlKwH85fhDoMmxz3nWF6t\/8MEHNHnyZKpbt24KSrV\/EZIMFq91Z0sGnZFKLkkUCo1kEpTz+wB\/Of5gL8ceAlmWfZJ9ruybCo6g+e3i\/PTTT2nRokXUrVs3+t3vfkeVKlVKA6v9ypBUsLinNrO4KcANH52U7EcC\/OX4g70cewg0WfZJ9bmyb\/kf7yJTnH67OKtUqUKXX345DRw4kGrUqJEWVmICzT21mdV1Z84KQCcl+7EAfzn+YC\/HHgJNlj0Emiz\/ovKeRLBgatM7JNBJyX5UwF+OP9jLsYdAk2WfRJ8r+4blvYuMoOUDwLs3t2\/fTtWrV0\/ttKaz7EkES4dRi2nOJ9\/v2uSpzYp6+bnphwKdlCkxu+nB3y5PE2tgb0LLflrwt880qMUk+tygZUkinZhA440Af\/zjH4k3AowcOZJKSkrUsRrdu3entWvX0m9\/+1u65JJLUi3U4g6WN1dsoUtHLs7Fwa9\/eiTdeXHzJOIi9T7QSMpWEfjL8Qd7OfYYQZNlH3efK\/t2+3sXE2hjx46lYcOGqbVmgwYNUgJt165dNGvWLBo\/fjytWLFCnYF24YUXpo1ZrjxxB8sljy7OnXmG0bPyYYBOSvZjAf5y\/MFejj0Emiz7uPtc2bdLiUDTI2XHHHMM3XPPPfsdRMuja3379lW3CrBYY\/GWxifOYHGPng1o35T4rk083xNAJyUbCeAvxx\/s5dij7ZFlH2efK\/tm3t5FRtD0Ls4+ffpQp06dPEv2zDPPqKueZsyYQfXr108jO4ozWNyjZ1k+88yr8tFJyX4kwF+OP9jLsYdAk2UfZ58r+2YpEmg8MnbNNdfQOeecQwMGDPAs2ZAhQ+iNN97I5EG17tGzLN616fdhQSflRyjev4N\/vHwLWQd7OfYQaLLsIdAS4M+7Ne+++2568cUX6f7771dCTR9Gy39jYXbrrbfSxRdfnMmDajF65h+E6KT8GcWZAvzjpFvYNtjLsYdAk2UPgZYQ\/zVr1tCNN95Iy5cvV4fR8tEa\/OzevVutPTvuuOOINxI0atQooRKZu4kjWNyjZzy1yRsE8JQngE5KNiLAX44\/2Muxh0CTZR9Hnyv7RoW9i6xB00XiXZu8xoz\/bdu2Tf2axVrHjh3Vv2rVqqWZXSxr0H4xagnN\/WSzem8WZlh75h0C6KRkPxrgL8cf7OXYQ6DJsodAS4A\/T2M+99xzdPzxxxPv5CzWx3awuG8NuOqU+sTXOuHZnwA6KdmoAH85\/mAvxx4CTZa97T5X9m38vYuMoG3cuFHt3rziiiuorKzMv5QpTWE7WF7\/6Cv65ZiludEz3BqQv+LRScl+KMBfjj\/Yy7GHQJNlb7vPlX0bf+8iAk3v4uSbAiDQvq8kHj3jWwP4Jz\/ntKhNz\/Rq7V+DGU2BTkq24sFfjj\/Yy7GHQJNlD4GWEP\/XXnuNfv\/736uRtIsuuogOOeSQ\/TwfeOCBVKdOHeKfaXxsBguO1jCrYXRSZrxspwZ\/20SD2wP74KziSAn+cVANZtNmnxvMo2wqkRE0fVAt7+Qs9PAOzqwcVIujNcw+CGgkzXjZTg3+tokGtwf2wVnFkRL846AazCYEWjBOkVLx7s158+apIzUKPVWrVqUzzzwztbs5bQWLe3MArnXyDy80kv6M4kwB\/nHSLWwb7OXYs2fwl+Nvq8+VewMzzyIjaGZFTG9qW8HinN7EpejB6huNZDBOcaUC\/7jI+tsFe39GcaYA\/zjpFrZtq8+VewMzzxBoZrzKpbYVLM7pzTOb16IX+rSJUKpsZEUjKVvP4C\/HH+zl2GMETZa9rT5X9i2Ce09MoOl1Z1y0hx9+mG6++WbCGrTvd2+2Hjw\/V2O4dzNY8KKTCsYprlTgHxdZf7tg788ozhTgHyddjKA5CSQm0LZu3UqjRo1Svq+++mp64okniH9X6KlZsyb17t2b+GcaHxtqfuo766nP1GXq9XBzQPBaRiMZnFUcKcE\/DqrBbIJ9ME5xpQL\/uMj627XR5\/p7SU+KxARael7ZXklsBItzevPso2vTc2U4+yxIDaGRDEIpvjTgHx9bP8tg70co3r+Df7x8C1m30efKld7cs5hA4+ueXnnlFZozZw7dcccddOihh6r7OPng2oMOOogGDhxIxx57rPkbJZgjarBgejN8ZaGRDM\/ORk7wt0ExnA2wD8fNVi7wt0XS3E7UPtfco2wOMYH28ssvU9++falVq1Y0evRoql27tpryvP\/+++mFF14gPmJj7Nix1KZNehfMRw0WTG+GD340kuHZ2cgJ\/jYohrMB9uG42coF\/rZImtuJ2ueae5TNISLQduzYQT169FDnm\/GGgZKSknIUNm\/eTL169VK3C7B4Y7GWxidqsDinN886uhbxBgE8wQigkQzGKa5U4B8XWX+7YO\/PKM4U4B8n3cK2o\/a5ciUP51lEoOkdnTfeeCN17drVs+RTpkxRI2gV9SYB9\/Tmo1eV0lWn1A9XixnMhUZSttLBX44\/2MuxZ8\/gL8cfAi0B9hs3blR3cF522WXquA2vZ+TIkfTUU0\/R9OnTqV69egmUytxFlGDB3ZvmvJ050EhG4xc1N\/hHJRg+P9iHZ2cjJ\/jboBjORpQ+N5xH2VwiI2h79uxR68+WLVtG48aNo2bNmpWjsGrVKrrhhhuotLSUhg8frjYNpPGJEiy4ezNajaKRjMYvam7wj0owfH6wD8\/ORk7wt0ExnI0ofW44j7K5RAQav\/L7779P3bt3p+3bt9NJJ51ETZo0USQ2bdpEc+fOperVq9OECRPoxBNPlCVUwHvYYMHdm9GrFI1kdIZRLIB\/FHrR8oJ9NH5Rc4N\/VILh84ftc8N7lM0pJtD4tT\/\/\/HO677776LXXXqNvvvlGkahSpQqdf\/75dNttt1HDhg1l6fh4DxssOF4jerWikYzOMIoF8I9CL1pesI\/GL2pu8I9KMHz+sH1ueI+yOUUFmuyrR\/ceNliGzFpFQ2atVgXA7QHh6gGNZDhutnKBvy2S5nbA3pyZzRzgb5Omma2wfa6Zl\/SkhkCLUBdhgwWXo0eA\/u+saCSjM4xiAfyj0IuWF+yj8YuaG\/yjEgyfP2yfG96jbE4ItAj8wwQLpjcjAHdkRSNph2NYK+Afllz0fGAfnWEUC+AfhV60vGH63GgeZXNDoEXgHyZYcLxGBOAQaHbgWbCCTsoCxJAmwD4kOEvZwN8SyBBmwvS5IdykJgsEWoSqCBMsWH8WATgEmh14Fqygk7IAMaQJsA8JzlI28LcEMoSZMH1uCDepyQKBFqEqwgQL1p9FAA6BZgeeBSvopCxADGkC7EOCs5QN\/C2BDGEmTJ8bwk1qskCgRagK02DB+rMIsF1Z0UjaYxnGEviHoWYnD9jb4RjWCviHJRc9n2mfG92jrAVRgcY3Bvz5z3+mzz77zJNCzZo1qXfv3sQ\/0\/iYBgvWn9mrRTSS9liGsQT+YajZyQP2djiGtQL+YclFz2fa50b3KGtBTKC9\/PLL6ronfUCtF4ZGjRpVqMvSsf7MXrCjkbTHMowl8A9DzU4esLfDMawV8A9LLnZ3vn8AACAASURBVHo+CLToDH0t7Nixg3r06EFffPEFPfTQQ3T88cdTpUqVfPOlLYFpsGD9mb0aRCNpj2UYS+AfhpqdPGBvh2NYK+Afllz0fKZ9bnSPshZERtDWr19PHTt2pGuuuUZdil6sj0mw4P5Nu7WMRtIuT1Nr4G9KzF56sLfHMowl8A9DzU4ekz7XjkdZKyICbfPmzeqi9AsvvDCzAu35sjZ01tG1ZGu\/iL2jkZStPPCX4w\/2cuzZM\/jL8YdAS4j92LFj6ZVXXiH+Wa9evYS82nVjEixYf2aXPRpJuzxNrYG\/KTF76cHeHsswlsA\/DDU7eUz6XDseZa2IjKDt2rWL3njjDXrsscdo+fLl1LZtW6pevfp+JCrSLs5fjl5Cr3+8Wb3jmc1r0Qt92sjWfJF7RyMpW4HgL8cf7OXYYwRNlj0EWgL89Rq0NWvWFPRWkXZxOjcIjOh0HF1z2hEJkK64LtBJydYt+MvxB3s59hBosuwh0GT5F5V3k2Cp0\/f13Lth\/Vn0akYnFZ1hFAvgH4VetLxgH41f1NzgH5Vg+PwmfW54L+nJKTLFmZ7Xj1aSoMGCA2qjcfbKjUbSPlMTi+BvQstuWrC3y9PUGvibErOXPmifa8+jrCVRgbZy5UoaPHgwzZ8\/nw4\/\/HB1KO0hhxxCAwcOpJ\/97Gd0ySWXJHY+Gh+aO3PmzFxtjB49mtq1a1ewdoIGCzYI2A9yNJL2mZpYBH8TWnbTgr1dnqbWwN+UmL30Qftcex5lLYkJtPfff18dtXHAAQdQ06ZNad26dUqgVa1aVf1+2bJl6hBbP5FkAx+Ls0WLFuVuLZg9ezb16tWL+vXrR2VlZXldBA0WHFBro5bK20AjaZ+piUXwN6FlNy3Y2+Vpag38TYnZSx+0z7XnUdaSiEDbs2ePuuaJNwnwMRuLFy9WI2ks0OrXr0\/btm1T56PxaBqPZLFoi+tZunQpdevWjYYOHVpODHL51q5dS+PHj6eSkhJP90GDpfXg+cQH1fIzoH1TGtC+WVyvkxm7aCRlqxr85fiDvRx79gz+cvyD9rlyJbTrWUSgbdy4kTp16kRXXHGFGqHiESunQONXHDduHE2ePFnsLk6bAg0bBOwGLRpJ+zxNLaKTMiVmLz3Y22MZxhL4h6FmJw8Emh2OBa3oYzZuvPFG6tq1q6dAmzJlihpd06NqCRQr52LkyJE0bNgwNXpXaIpVB8uTTz5JfCQIPzwC6HzeXr2DLh25OPerRQNPoSPrHJzk61RIX2gkZasV\/OX4g70ce3w5TJ496wX98Kxbly5daM6cObk+N\/kSJedRZARNX5Zet25dGj58uDq01muK86CDDqIxY8bQoYcemggRvfaMnXXo0EGVrdCjBZozDU+XXnvttblfjVmwhcYu3KL+v0GNyvTCtd8LOTzRCOzevZt4JJYFceXKlaMZQ25jAuBvjMxaBrC3hjKUIfAPhS10pscff5wmTZpULj8EWmicwTK+\/PLLahE+j6A1adKERo0apUasWCE\/8sgjapPAkCFD1DRo0o8WkLwGrdAInhZovH6tYcOGqpgsGJyjaFeO+z+at2q7+hvfIPCnX5Um\/ToV0h\/fRrFhwwb1LQoCLfkqBv\/kmWuPYC\/Hnj2Df7L8eQRNj6ItWLCARowYgRG0uKtg3759ShWzuNm5c2c5dwceeKDaRMBToPzfEo\/ePNCzZ8+8OzmDzIc7d3Big4C9msQ0jz2WYSyBfxhqdvKAvR2OYa2Af1hy0fMF6XOje0mPBZEpTufr847NhQsXqn\/ffPMN\/ehHP6KzzjqLateuLUrJlkDDBoF4qhGNZDxcg1oF\/6Ck7KcDe\/tMTSyCvwktu2kh0OzyTL01XnfWv39\/NZrXqlWrXHn1erRCGwX8gsV9g8CSQWdgg4CliEAjaQlkSDPgHxKchWxgbwFiBBPgHwFexKx+fW5E86nLLjqCxtOcH3\/8MT399NO0ffv367R448Dll19OzZolc1aYXm\/GvvWZZ3r0rLS0NNI5aE6Bxjs3WaDhsUMAjaQdjmGtgH9YctHzgX10hlEsgH8UetHyQqBF4xc4N09t\/va3vyXeLMBCzflUqlSJOnfuTHfeeSdVqVIlsM0oCd1XPfndIsC+\/IJl5pIvqPukD1SxINCi1M7+edFI2uVpag38TYnZSw\/29liGsQT+YajZyePX59rxkh4rIiNoLMh4c8CECROoT58+6liKGjVqKCos3PiQWj4D7ZZbblEbBdL6+AULrniKr+bQSMbHNohl8A9CKZ40YB8P16BWwT8oKfvp\/Ppc+x5lLYoItE2bNtE111xDp5xyCt111137XYjOAo5Hz\/i+ThZx0hsG8lWRX7Dgiqf4ghuNZHxsg1gG\/yCU4kkD9vFwDWoV\/IOSsp\/Or8+171HWoohA0zcJ8OgZX\/nk9bzwwgtqlE3iJoGgVVIoWPjuTRZo+nm+rA2ddXStoKaRzocAGknZEAF\/Of5gL8eePYO\/HH8ItATY80nMN998s1pfxqf1u9eZ8XEbt99+u5rufPjhh2O9LD3K6xYKFuzgjELWPy8aSX9GcaYA\/zjpFrYN9nLsIdBk2UOgJcSfbwzg9WWNGzemW2+9lZo2bUoHHHCAOjGYbxLgYy7uu+8+OuGEE3Il4kNr69Wrl1AJ\/d0UCpYhs1bRkFmrlRFsEPBnaZoCnZQpMbvpwd8uTxNrYG9Cy35a8LfPNKhFCLSgpCKk01OcLNJMHr7Wh+\/gSstTKFgmL1hHv56+XBWVr3h6oU+btBS7QpQDjaRsNYK\/HH+wl2OPETRZ9hBoCfDnu8zmzZtHPNVp8lStWpXOP\/98kyyxpi0ULNjBGSt6rAOJF6+vdYgEX0SxJQD72NAGMgz+gTDFkggCLRasFdNooWDBDs546xyNZLx8\/ayDvx+h+P4O9vGxDWIZ\/INQiicNBFo8XPNa3bx5M7355pv07rvvqs0Cbdq0odNPPz21R2s4XyRfsGAHZ\/xBhEYyfsaFPIC\/HH+wl2PPnsFfjj8EWkLs9+7dqw6j5V2c\/N\/OhzcD8BEcZWVlid0kEOa18wWLewcnjtgIQ7dwHjSS9pmaWAR\/E1p204K9XZ6m1sDflJi99BBo9lgWtPTUU0\/RgAEDqG3bturGAN7Fyc\/q1avpgQceoLfeekudg3bppZcmVCJzN0EEGnZwmnMNkgONZBBK8aUB\/\/jY+lkGez9C8f4d\/OPlW8g6BFoC7PUF5SUlJeqcs2rVqpXzypsI+Jw0HlkbPXp00Z2DNvWd9dRn6jL1ThBo8QQUGsl4uAa1Cv5BSdlPB\/b2mZpYBH8TWnbTQqDZ5elpTR+zweegde3a1TPNlClT1BRoMd4kgB2c8QcRGsn4GRfyAP5y\/MFejj17Bn85\/hBoCbDfuHGjuuLpsssuUyNlXg+PrD377LM0ffr0VB1O6yxrvmBxCrQB7ZvSgPbNEqCaLRdoJGXrG\/zl+IO9HHsINFn2EGgJ8N+zZw\/17duXli1bRuPGjaNmzcoLmFWrVtENN9xApaWlahPBQQcdlECpzF3kC5Y6fV\/PGXv0qlK66pT65saRoyABdFKyAQL+cvzBXo49BJosewi0hPi\/\/\/771L17d3VY7QUXXEBnnnmm8swH2L766qtq3RlPcfKxG2l9vIIFR2wkU1vopJLhnM8L+MvxB3s59hBosuwh0BLk\/9FHH6lL0ZcsWUL79u1TnitVqkStW7eme++9l4499tgES2PuyitYcEm6OccwOdBJhaFmLw\/422NpagnsTYnZTQ\/+dnmaWINAM6FlKS2PovGBtfzUrl07tbs23a\/rJ9Cwg9NSgHiYQSMZH9sglsE\/CKV40oB9PFyDWgX\/oKTsp4NAs8+0wlr0CpYhs1bRkFmr1TtDoMVX9Wgk42MbxDL4B6EUTxqwj4drUKvgH5SU\/XQQaPaZVliLXsGCIzaSqW40kslwzucF\/OX4g70ce\/YM\/nL8IdDk2BedZz+BhiM24qtSNJLxsQ1iGfyDUIonDdjHwzWoVfAPSsp+Ogg0+0wrrEWvYHEesbFk0BlqmhOPfQJoJO0zNbEI\/ia07KYFe7s8Ta2Bvykxe+kh0OyxrPCW3MGCIzaSq3I0ksmx9vIE\/nL8wV6OPXsGfzn+EGhy7IvOsztYcMRGclWIRjI51hBosqzd3hH7svUB\/nL8IdDk2Bed50ICDTs4461ONJLx8vWzDv5+hOL7O9jHxzaIZfAPQimeNBBo8XCtkFbdwYIjNpKrZjSSybHGCJosa4yggX+6CMiVBgJNjn3ReXYHS5+py2jqO+vVe5zZvBa90Ce911QVHWxXgSHQZGsQ\/OX4g70ce\/YM\/nL8IdDk2BedZ3ewOM9A4wvS+aJ0PPEQQCMZD9egVsE\/KCn76cDePlMTi+BvQstuWgg0uzwrtDV3sLQePJ94Jyc\/LM5YpOGJhwAayXi4BrUK\/kFJ2U8H9vaZmlgEfxNadtNCoNnlWaGtOYPlu0PqEgs0\/Txf1obOOrpWhX5\/yZdDIylJH9M8kvQR+5L0EfuS9CHQJOkXmW8INLkKQyclx549g78cf7CXY4\/Yl2UPgSbLv6i8O4Nl9b9K6NKRi3Plxy0C8VYlOql4+fpZB38\/QvH9HezjYxvEMvgHoRRPGgi0eLhWSKvOYJm7rjLxLk5+cAZa\/NWNRjJ+xoU8gL8cf7CXY48RNFn2EGiy\/IvKuzNYnvhgDw2ZtRoCLaEaRCeVEOg8bsBfjj\/Yy7GHQJNlD4Emy7+ovDuDpfdzG2neyi2q\/DgDLf5qRCcVP2OMoMkyzucdsS9bL+Avxx8CTY590XnOJ9AGtG9KA9o3K7r3KaYCo5GUrS3wl+MP9nLsMYImyx4CTZZ\/UXl3BsvFE\/+ZOwMNR2zEX43opOJnjBE0WcYYQQP\/dBKQKxUEmhz7ovOsg2XKzNnEAk0\/EGjxVyUEWvyMIdBkGUOggX86CciVCgJNjn3ReYZAk6syCDQ59pjmAXtZArLe0fbI8YdAk2NfdJ51sNz32PPEmwT0gzPQ4q9KNJLxM8YImixjjKCBfzoJyJUKAk2OfdF51sHS+4EZdN\/c7bnyfzX83KJ7l2IrMASabI2Bvxx\/sJdjj9FjWfYQaLL8i8q7DpaLfjuJ+Bw0fnBIbTJViE4qGc4YxZHl7OUdsS9bJ+Avxx8CTY590XnWwXJi77E0d92Bqvw4Ay2ZakQjmQxnCDRZzhBo4J8+AnIlgkCTY190nnWw1Oo8gvguTgi05KoQAi051hAJsqzd3hH7svUB\/nL8IdDk2BedZy+B9nTPVnTusXWK7l2KrcBoJGVrDPzl+IO9HHv2DP5y\/CHQ5NgXnWcdLNvaDaHvDqmryo9bBJKpRjSSyXDGFKcsZ4xegn\/6CMiVCAJNjn3ReeZg6dzjv4gFmn5wSG0y1QiBlgxnCDRZzhBo4J8+AnIlgkCTY190niHQ5KoMAk2OPaZ5wF6WgKx3tD1y\/CHQ5NgXnWcvgYZDapOpRjSSyXDGCJosZ4yggX\/6CMiVCAJNjn3ReeZguWLgo7TzpO65suOQ2mSqEQItGc4QaLKcIdDAP30E5EoEgSbHvug8c7Bcfvd0+tdxl6qy45Da5KoQAi051hAJsqzd3hH7svUB\/nL8IdDk2BedZ7dAwyG1yVUhGsnkWEOgybKGQAP\/dBGQKw0Emhz7ovPMwXLpyCX0bd1jVdkh0JKrQgi05FhDoMmyhkAD\/3QRkCsNBJoc+6Lz7BZok647gS5uWa\/o3qMYCwyBJltr4C\/HH+zl2LNn8JfjD4Emx77oPHOwXDzxnzikVqDm0EgKQHe4BH85\/mAvxx4CTZY9BJos\/6LyzsFy4YxduTI\/elUpXXVK\/aJ6h2ItLDop2ZoDfzn+YC\/HHgJNlj0Emiz\/ovLuFmi4RSC56kMnlRxrL0\/gL8cf7OXYQ6DJsodAk+VfVN7dAg2H1CZXfeikkmMNgSbL2u0dsS9bH+Avxx8CTY590Xl2CzQcUptcFaKRTI41BJosawg08E8XAbnSQKDJsS86z06BhkNqk60+CLRkeUMkyPJ2ekfsy9YF+Mvxh0CTYx\/Z844dO6hHjx60cOHCnK1+\/fpRWVlZQdtLly6lbt260fbt28ulO\/XUU2n8+PFUUlLimR8CLXKVhTaARjI0OisZwd8KxlBGwD4UNmuZwN8aSmNDEGjGyNKRYf369dSxY0dq0KBBTlRp4XXeeefR8OHD8xZ09uzZ1L9\/f5o0aRK1atUq8As5BRoOqQ2MzUpCNJJWMIY2Av6h0UXOCPaREUYyAP6R8EXKDIEWCZ9c5nwia+TIkTRt2jSaMWMG1a\/vfQQGp5kzZ07B0TKvN4NAk6tvNJJy7Nkz+MvxB3s59oh9WfYQaLL8rXtn8TVmzJiCo2N9+\/ZVfguNsvkJtAHtm9KA9s2slx8GvQmgk5KNDPCX4w\/2cuwh0GTZQ6DJ8rfuncXXokWL8o6g6alRXme2fPnynP8OHTr4CjbnCBoEmvWqK2gQnVSyvN3ewF+OP9jLsYdAk2UPgSbL36p3nvbs1asXFdoo4LVOzWs9m98I2m1nV1e3COSbRrX6YjCGKTbhGIBIkKsAsJdjD4GWPHvuj\/WzZs0a6tKli1qS1KhRo+QLk7DHSvv27duXsM9E3GnhVVpaary2jAuoxd3o0aOpXbt2nmV2jqCVvHk\/Vd70kdoNeu211ybyjll2snv3btq4caMSxJUrV84yCpF3B38R7Mop2MuxB\/\/k2T\/++ONqiZLzgUBLvh6seYwqzrgg2kbPnj3zHtPhFGijflGPmlTdoQQDRtGsVWVeQ7t27aINGzaob1EQaPHzdnsA\/+SZa49gL8eePYN\/svx5BE2Poi1YsIBGjBiBEbRkq8CeNz3y5XeGmZ9HU4GGa578iNr9++rVq2nixInq3LssDHXbpRfdGvhHZxjWAtiHJWcnH\/jb4RjGCtaghaGWkjxanAVZ4K+LnG8qM8jZaM4RNFzzlGwQZO2Dmixdf2\/g788orhRgHxfZYHbBPxinOFJljX2FWYMW9FBad9Do2wfWrl2b2+kZ1JYWaLjmKY6PYmGbWfugJk8Y\/NPGXJcHsS9bM+Avxz9r7CuMQOPjNGbOnJk3cvRi\/3znornzB7kiSgu0pgfvoOfL2shFbQY96908Tz75JKY4Beof\/AWg\/9sl2MuxZ8\/gL8cfuzjl2BedZw6WlsM\/Ubs3eRcnHhAAARAAARAAgfgInHbaaTR16tT4HKTIcoUZQZNiyiKN\/+EBARAAARAAARCIlwBvCsvKxjAItHhjCdZBAARAAARAAARAwJgABJoxMmQAARAAARAAARAAgXgJQKDFyxfWQQAEQAAEQAAEQMCYAASaMTJkAAEQAAEQAAEQAIF4CUCgxcsX1kEABEAABEAABEDAmAAEmjEyZAABEAABEAABEACBeAlAoMXLF9ZBAARAAARAAARAwJgABJoxMmQAARAAARAAARAAgXgJQKDFyxfWQQAEQAAEQAAEQMCYAASaMTIifcH6woULVW4+1XjGjBlUv379ENaQJR8B5\/2o1atXp0mTJlGrVq0KAps9ezb16tUrlwZ1Ey6+wrB3elq\/fj117NiROnfuTGVlZeEKkeFcYfi72yXGp+8gzjBK41cPw17Hu75VBu2OMXajDFxH\/AwfPtwoX7ElhkAzrDHdCDZo0CAXHBwsixYtgkgzZFkoOTNdu3YtjR8\/nkpKSijfJfdOG5xm2LBh5ToltvOXv\/wlkLizWPyiNhWGvfuFdSfXr18\/CDTDaAjDXwsEbpecnxn358GwKJlLHoX9ySefjD4hgYjR7XyHDh0g0BLgXVQuvIQCRgvsVqEeBXN++\/cSxk6v+f6OujGrmzDs3R6co5gQaMnw53Zp2rRp5b4k+n1mzEpW8VOHjX2vPmHp0qXUrVs36tmzJ76gWAod9wgxBJolsBXJjPsbln63fL+vSO+e1Lt4dTbsO9\/vC5VLCzTnt9uk3qMY\/URlr3n36NFDjeRgitMsCsLw1x1X27ZtIQbMcJdLHYa9bpfGjBlTbpQeAi1CRXhk1THOsyoTJ06k22+\/nZyzWHa9pccapjgN6qLQN1JMcxqA9EmaT+wGmeZ0m0ZDaVYvUdg7Px+33nor1qCZoVepw\/DXonjQoEG0YsUKNc3PT9B1myGKWSGzhGHPILy+BGJpRXwhkqWRYQg0gzgqFBhhRncMXGcqab6Gkqcg+vfvH3g9mfNbFzZxBAuhKOydnwH2hk0CwZg7U4Xhr7+EbN++nZzTPl5rMs1LlJ0cYdi7627mzJnqV6eeempuLWB2CCbzphBoyXAuOi8QaMlUWdSGUpdSL1THTrbg9RaWvRYJQ4cOpXbt2uVGFTDFGZx9oRG0Ql9ONPvS0tJyokC3V2xXbxwwK022UoeNfb12zbneEuI4vtiBQIuPbVFbxhRnMtUXdqrB69ssxJlZnYVh7\/W5wOYMM+7OLxXO3cv694Wm97VAO++88\/bb1YaR\/eD1ECX2vUQw1iUHZ2+SEgLNhFbG0mKTQPwVHnaxLpfMudMH4sy8rsKwd06xeXnEmVDB6yEM\/0IbYSDQ4mWPWZXgfG2lhECzRbIC2vFq8DBaYLeivaZzgnwodZply5YFXqdmt+TFby0se\/eb4zMRLhbC8vfapBTkMxOulBUzVxj2haaRMYIWT5xkKa6xScAwhrwOhMQOTkOIPsm9FvcH2cGJnVPR6yEsewi06OydI8A8zak3tgSJfa9pziD57JS6YlgJG\/tYg5Zs\/UOgJcu76Ly5D8zDFE48VagX+bN1ryMDnMJ4w4YN6mBI3snm9WBXlVkdmbD3uuIMI2hmvN2pw\/B3XzeEYzbC1UEY9u5pfrAPxz5ILgi0IJSQBgRAAARAAARAAARAIBYCmOKMBSuMggAIgAAIgAAIgEB4AhBo4dkhJwiAAAiAAAiAAAjEQgACLRasMAoCIAACIAACIAAC4QlAoIVnh5wgAAIgAAIgAAIgEAsBCLRYsMIoCIAACIAACIAACIQnAIEWnh1yggAIgAAIgAAIgEAsBCDQYsEKoyAAAiAAAiAAAiAQngAEWnh2yAkCIAACIAACIAACsRCAQIsFK4yCgByBvXv30nPPPUe7du2iq6++2rggn3\/+OY0fP57KysqoXr16xvmjZHjttdfo97\/\/PfFVRw0bNqRp06apn6ZPoQvETW2lIb37loB+\/fqp+pF43Kfmjx49mtq1aydRFPgEgQpNAAKtQlcvXi6LBKJes8R3OLIw0ndBJsVw27ZtdMMNNyhx1qtXL\/rBD35AZ555JlWrVs24CBVVoB1xxBHqSrNjjjmGWrRoYczFRobNmzfTggUL6O9\/\/7sS8hBoNqjCBgjsTwACDVEBAhWMQLEKNJuiyqatNIRHGt9HXxIOgZaGCEEZKiIBCLSKWKt4pwpLYN++ffTCCy\/QAw88QGvWrKGDDjqIWrduTXfddRcde+yxVOjSZr5k+JFHHqGZM2fSF198QZUqVaJGjRrRLbfcQpdccon6f+dF0QyxQ4cONHz4cMXzo48+ot\/97ndq5ISfli1b0q233kqnnXaaL2+eNn344YfpxRdfpJ07d1KDBg3oV7\/6lZqCrVKlCunO3mmo0DQec3jllVfoj3\/8I3366aeKwxlnnEGDBg2i5s2bkxY0bdq0oVNPPZUeeugh2rhxo\/LLZdbvy\/6++eYbeuKJJ2jixImKKds+\/PDDqXv37nTdddep8vGjRxavv\/565fe7776ju+++my6\/\/HJauXKl+u+33nqLDjzwQLrqqqvoxBNPVPUyadIkatWqlbLB087sZ8KECfTll1\/SYYcdRr\/85S\/ppptuopKSkrwcvQSa89Loc845h+655x7atGmTioPbb79djT5yneZ78o2UBh1BhUDzDXskAIFIBCDQIuFDZhBIlsBLL72kBNVPfvITat++PW3ZsoX+93\/\/V3XETz75JFWvXl2JnWHDhtGPf\/xj+vnPf64E1KGHHqrE1xtvvEFXXHEFnXLKKbRq1SqaOnWqEgpjxoxRNt99910lKFho\/OY3v6HjjjuOfvSjHymbnL9p06bUtWtX9dJTpkyhFStWKAF34YUX5gXBooeFDgsk\/tmkSRMlEufNm0cdO3ZUIoaFxeuvv04PPvggHXXUUQWn8VhA8agNv2NpaakqDwsYLk+dOnXoscceUyKJbfPveVrw2muvpapVqyphxOVgZsyAbQ0dOpTGjRunRFvbtm1p69atyhYLvzvvvFOVRQs0Fsa1a9emG2+8Ufn46U9\/qsQhvxfXBZelRo0aSoSxGGPxpwUaCyoWYosWLcrVwTvvvENPPfWUEtksjDiv11NIoC1btkyV4corr1Rs2R+XnUVpobVhEGjJfnbhDQRMCUCgmRJDehAQJMAiiUejWFDozvzNN99U4mnw4MGqQ\/aa4uTRL15UzqM1zsXlLBB4JIvXfOnfuztuXhvGAqR+\/fpq5E6vCWPxwX55zRgLEi9xwQKIRQ5vWmBxxGKPH97IwAKLR660WAo6jffZZ59Rly5dlKhhcahHuN5++226+eab6bbbblPilAUaj3JNnjyZmjVrpvzq9+W1bpyWhWHPnj3Vei4eAWOhw4\/2waNvegSRuXCZ77vvPurUqZNKp9+PR\/PGjh1LPGLHj1OUaoHGopRHINnOWWedlYsiLjcLPvbPI5amAm3JkiXlRDKvEePRPxbt+epFC06vtYYYQRP8gMM1CDgIQKAhHECgiAiwQGJxxgKDR3a8dlmarEHTaS+66CIaMGCAIuHuoBcvXqxGoC6++GI1beZ8eGSNR+Wc03jOv7MAuuaaa5RAGjFiRE4AcRoe+eF34BE99h1UoPFOTxaTPF2bb4TIOcXJo3J6qi+oDz19yCNyutzMhUcane9a6P14SpcX0XN6ng7mEUkeoeRRNB7p1A+PvHEZzz777JwYdIdkoRE0Tst+nFOkXFaOExanJ5xwgmeEYwStiD74KGomCUCgZbLa8dLFSoBHz3RHz+\/ATBu+qgAABalJREFUa6V4GpOFjh4lKiTQdu\/eraa\/WBwtXLiQePSNR8Cca83cHbfX+jAnPxY\/LFzOP\/\/8\/bB6CUCdyC06goonFh4szvKJQrafz1a+3\/NoILP98MMPlYiaO3eu4sTToFr8eAk0zsOjafyPR+Tc4rV\/\/\/6qnLwurkePHop5vodH\/ZgjT0ebCDReV6dH+XS+IOvDINCKtRVAubNCAAItKzWN96wwBHhajcXD9OnTadasWbnNAnotmJdAYwHC03O8PounF3k6ks8X48Xrr776arnRm3wCLcxuvTgEmpdQCiJovIQbs2QBxevQePPCIYccotasnXzyyUpM8QhlIYG2evVqtdGhc+fOgQSa12hXkMD02yTgJdB69+6dVzizTwi0IOSRBgTkCECgybGHZxCwQuDjjz9Wa8h4gTiLKF6DxOuvWDTodWV\/\/etfiTtsXpvUp0+f3HSYFhi8kcC51sq5NomFCq9B42lOPQ0atOBBpjh59I9Hn4KOoOWb4uTF\/zydesEFFyjRxAxYaDnFi9uHnmbl9\/\/DH\/6gNgDwo9dx8caCQgKt0Ps5heQPf\/hD+vWvf6122fK6O95sYfIUEmi8JpDrncuqH\/bNYpx95TsvLV8a3uXK6+L8zsELMkpn8o5ICwIgUJ4ABBoiAgSKhMD27dvpv\/\/7v9UIGHeuerH+V199pcRT3bp18wo0Ts+d+OOPP55byM6jR9wJ85EMv\/jFL\/IKNL1JgNdKcf7GjRsrYjwqx3n52A1e6K+nWJ04\/TYJsIjgzQM8vRdUoOXbJPD888+rIzR49yKv+Qoi0LTI4PVivA5PPzz1y+KWhVUhgZZvk4AWeDzS6dwkwDtweYqa16HpdXE8pcobFfhoDv671+O3i9O5AYOPUOF44OlvrvN8B\/3yTlUWpaNGjVK7Ufn55z\/\/qfJ+++23EGhF0i6gmBWXAARaxa1bvFkFJMA7BXmqkndDsqji59lnn1UjM\/pYBT2qw3\/jUTNefP7ee+8pEcDTmjyKxp0276zk0THe6cjiRI806Y6bO2o+yoN9vfzyy2rHJi9EZzu8yJ3zs7hg0cgjePnO3ApyzAbvxAwq0JzHbPDIF28y+OSTT9SC+JNOOkmJV17kH0Sg8Qgavycz4PfiKc05c+aoY0X27NmjxGwhgcaMeRSSR+74WA0+ZoNtsGDi9z744INzAo2FLou++fPnK0HEzHnN2zPPPEO1atVSOy75XDpTgcZ1yJsZnL7Zl3NXqRaizrPl9LvzzlXeycsPC3AuCwtMPYKm64XXujk3I2AErQI2MHilVBGAQEtVdaAwIFCYAI+e8fQjd74sAPhxH0zKAobFCgu5r7\/+Wq1DOu+889QBt\/fff7\/aFMBrrfjcMxZwvDOUz\/7SRzLwwncWY3xeFx\/+qheu8\/U+nJ\/FHgsaFhM8bcdCkc8EK\/R4HVTL05qXXXZZ7piMoAKN\/bgPqmXByYJMi0iTTQJ85hsfUcJHkeiDf3lxP4\/I\/eUvf1Hnyx155JFK+Ll3cep35oNq9WiiPqiWy8RTjM7NDPqw4KefflqdP8f1cO6559LAgQML3jlaaASNWXBd8mgkj3KySOUjOzgu9OMl0Phvzndnkce7g7le\/+d\/\/gcCDY0RCAgTgEATrgC4BwEQqJgEeNqUN3LwvzAXvjupmB6zkQRRjKAlQRk+skwAAi3LtY93BwEQiESAjy3h9WQHHHCAGonUZ5Hpi9952td9RlkYhxBoYaghDwgUNwEItOKuP5QeBEBAmABPYfIU6emnn67Ok+PNE0GvwQpadC3Q+AgQ3vV6zDHHqHtF+Ww1fmyIwKBl4fVpPN3Nm0PYb5jjV4L6QjoQyDIBCLQs1z7eHQRAIDIBXhfIGyb4xgFeF8ijabyLlNex8SaGQheWB3WuBZped8iL\/VmoSQg03pDCvnlXMT8QaEFrEelAwIzA\/wO\/b9frMVW7LgAAAABJRU5ErkJggg==","height":297,"width":493}}
%---
