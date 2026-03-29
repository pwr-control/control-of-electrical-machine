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
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAWoAAADaCAYAAACPZQgiAAAAAXNSR0IArs4c6QAAIABJREFUeF7tXQvYTlX2X0r4KkL8yyUxxl0ZI9IkNMiUSCbEFIkeCZXKLco1fMgYl4aKGipdVUJR\/CXN8C+5dJFbIcklEQqF+T+\/bfY3+zvfed\/3nPOefd59trWfx\/N9vnefvff6rbV\/Z71r7712vn\/\/+9\/\/Ji6MACPACDACxiKQj4naWN3wwBgBRoAREAgwUbMhMAKMACNgOAJM1IYriIfHCDACjAATNdsAI8AIMAKGI8BEbbiCeHiMACPACDBRW2gDK1eupI4dO9Lo0aOpffv2oUu4efNm6tKlC7Vs2ZL69+8fevtqg0ePHqUBAwbQjh07aMaMGVS8eHGt\/bk1\/tJLL9HkyZPpmWeeoUqVKkXef9AOpZ7wvJ+xm4C5KrOUY\/z48VS\/fv2gcMT6OSbqWKvPffBRE\/UPP\/xAXbt2pXLlytGYMWMoKysrNFSdpIGGdfXlNmg\/JOEXB516UnGbMmWK0InXl5xpRA29ZGdn01tvveXrhROaERrQEBO1TyXAYKZPn57z1AsvvCDe8tK48cFNN91E3bp1E3WcXq36fK1atXK8RDlpH3nkEVq7dq0wSnisyYgPnt7AgQNFP6VLl84xYicBoE\/87a677qJevXqRHLMkoV27duUaq\/x7nTp1RP8oqle7f\/\/+HI8abYI4161bJ+q5jdk5HjciSIUrPOpx48ZR37598\/SF9tQxdO\/ePZenL9sGRqg3YsSIXHqR45NKlfhIgsDn0puXY4d+UKQO5QvEiUOisTn7lHaSbCxOU5UvBtmnHAtIGfqSY1RtQ23D+bzETdWPtBmnLTtxUPUuZcAL4qmnnhLzA9+8VHvDN77Dhw\/n+qaUyB7Rd5Tf4nxSQiTVmah9wKwSo\/oYJjYmiTo5nJ\/DWJ1kpBIbJhuM11kShS+cE1oljU2bNuUKfbiRYOXKlXORm0pSF154oSDisIhaEoKcsE7Pc968eTkvnES4JiLqRx99lIYPH55DSvJ5iVsinSUiRvWlBxyc3nsiHfbr10+8BFWiTja2Sy+9NJe+MR7n35wvYBUbJ1HKz2CHIMixY8cmJepEz2McrVq1crVllfDdcJBE77RN\/N35QlfHi5egfPlLp8H50pQ2g79nKgTmgypCr8pE7RFSaSj79u3L8VwlCcAQ7733XmHcq1evdv1cGqoMD6BbSewgehQQtfRMJHG7eajqJJPen5w4aluSjORnKumrY4e3IycXZGnTpo1nosazXr7yS68ek0x9kTRr1kyQYSpcZYxaeq4SR+mxuv1fJUrny1TFRv1KreIicejdu7eI9Tu\/CUjPVercSezJxoZvKlLHXsbiXAuQ+pL24bQn6Tgkiu07n9+5c2eOzpPhpuIo25Y4yP9L\/bp52U77lt8C3n33XfGylvacSD51fnmculZUY6L2qEZnOADGqb7lpReD5mS4Qn0GoZAePXqQ02NAfTdydetPDjWVd+EW+nDG91Ridy7Q+Al9eCVqdUHuk08+yVmcg0yq954I10RErcbDVe8ZhCC9XNULU7FJ5DmiPl5YjRo1yrMomyhEg2eSvbCcY3MSdaqxOIlatqe+fNWXrXQcEhG12\/PSvpKFptSwEOqr3rMkXec3OtRz9ufsAyESNaQox+J8ETFReySsM7VaWESdaKeEk1xtI2opD7xnxODlC0315OQLzu0FmIionZ6kDOnAw\/ZK1H7JLFWcOtG3JnVsiYja6+6WTBO1+i1NvmScHrXbNzj5NzeiTrZYmMo5sZ2X2KP2qGGvoQ8YmzMcocboEsXY\/BC1NHLpXZQtW9Y1jJLoK7Wbh+P2lb9kyZIiHijjh87\/y5eOl9CHutiKCX3rrbeKcIJXXBMRtTN8oH5lVuPGyUIf8OScnqLqLUoc3eSUuOH5ZEQMWZ1f591CH4nG4jTTsEIfzrgydCpDH+pLQ\/0GJtcwpP7lS1jaRzKPOlXoI9GaDC8mcvY8j1T9369vzgfSWUxM9HUxmUetEq06FuckSEbUzhV2tCMXi2SsVS6MyT7kWNVdH2roA\/WT7VSRpObcheBlkdZJ1LKvRCElOY5EC5XJFhNlOEoSkoxR4+9ui2huuKH\/VGNTF5ATLSbKsTjDU8kWE\/FydcaNnVvznDs+pI7dYtB4ViXqRAvRyUIfifpz2pQzNKjasLr7xvOktaQie9Q+FellG1miLU3OiZ5sS10qona25bbVLxlRO+OLTkJQY49yy6AkSydRq6SRjKiTyeQFV5WA1O2LkyZNEvFN4Dl16lSaOXNmzqKu+m0DGEnduMV2pSkk86DdCNK5RdPr2NAf4vMgp0QvjmSHlhJtzwOxetkLnWg7nJcYtfpyhfzvv\/++0IHbeovENZlNYczO8Uhv389edp\/TOTbVM0LUR44coSFDhlDbtm1dTxotXbo0Zx8ykFRJyFRkvUwMU8du67hS7Y7xcsoNhPTiiy+ekVvCwrSLZLtMwj4kFea4TWkrcqJGpOXVV1+lQYMG0axZs1yJGhMDnlCDBg1MwSnlOJioU0KUkQpuYZVEB0DcBii9OTX8kRFBYt5potCH83BSzMXUNvzIiXrr1q309NNP08mTJ8V+XTevZuLEidS0aVOqWbOmNsG5YUaAEWAE4oJApEQNrxMxt3bt2tGCBQvEPlUnUR8\/flyERRAPxa6G3\/zmN2JHQ8OGDSlfvnxxwZXHyQgwAoxAaAhEStRz586lvXv3isMEOOLqRtSHDh2ixx9\/nFq3bi1i02vWrKGRI0fSqFGjqFq1arkEB4lzYQQYAUZABwJLliyhChUq6Gjad5uREfW2bdtEDgLEposVKya2+7gRtVMCxLThhVesWDFPyk4Q9VdffeVb6Lg8wPLFRVPu42T9sf7CQiAyonZLIgQhnIsJX3\/9tQh54LRTgQIFSBI1vOsWLVrk8aiZqMMyhejbYSKLHvMwe2T9hYlm8rYiI2rnMBJ51Lt37xYpEZGJDNnbQNoIhWALT\/ny5c8oosZLy5SvXjpMkuXTgWp0bdquP5NeREYQtdy6A4LG4uKnn34qwh3wwmvUqCGOtNatWzePBZoEpI7pYftEYPl0WE10bdquP5P4JWNEHYY5mQRkGPI427B9IrB8OqwmujZt159J\/MJEHZ1d++7J9onA8vk2CaMesF1\/TNQhmZtJQIYkUq5mbJ8ILJ8Oq4muTdv1ZxK\/sEcdnV377sn2icDy+TYJox6wXX9M1CGZm0lAhiQSe9Q6gMxQm7YTme3ymcQv7FFnaBJ76db2icDyebECc+vYrj8m6pBszyQgQxKJPWodQGaoTduJzHb5TOIX9qgzNIm9dGv7RGD5vFiBuXVs1x8TdUi2ZxKQIYnEHrUOIDPUpu1EZrt8JvELe9QZmsReurV9IrB8XqzA3Dq264+JOiTbMwnIkERij1oHkBlq03Yis10+k\/jFl0d94MABkXdj586dnkz\/9ddf91QvaCWTgAwqQ7LnbJ8ILJ8Oq4muTdv1ZxK\/+CJqJE\/CbSs333wzFS5cOKlFHD58mK6\/\/nqtVmMSkDoEtX0isHw6rCa6Nm3Xn0n84ouo4VEj+T9SkCL5f7KCuqnqpGtSJgGZrixuz9s+EVg+HVYTXZu2688kfvFF1EeOHBGpR6+88sqUHnUU5mISkDrktX0isHw6rCa6Nm3Xn0n84ouo4SXfd999tGrVKmrZsiXdcsstdMUVV1D+\/Pmjsw6lJ5OA1AGA7ROB5dNhNdG1abv+TOIXX0QNEzh16hRt376d3nzzTXrllVeEVdx2223i6qzSpUtHelO4SUDqmB62TwSWT4fVRNem7foziV98E7VqBidOnKBNmzbRq6++Sq+99hpVrVqV7rrrLmrcuHEkXrZJQOqYHrZPBJZPh9VE16bt+jOJX9IiamkSuIB2165dNGLECNqzZw\/NmDGDihcvrt1iTAJSh7C2TwSWT4fVRNem7foziV\/SImrErN977z2aNWsW7dixgzp27EitW7emSpUq0VlnnaXdYkwCUoewtk8Elk+H1UTXpu36M4lffBM19kcvX76cnnvuOVq7di1dc801dPvtt4tFxaysrOishIhMAlKH4LZPBJZPh9VE16bt+jOJX3wRtbwt\/NixY9S5c2dq0qQJlSxZMjrLcPRkEpA6QLB9IrB8OqwmujZt159J\/OKLqI8fPy5i0GXLlo0ktJHK5EwCMtVYg3xu+0Rg+YJYhTnP2K4\/k\/jFF1HDox42bBgNGTIk5WIh6upeUDQJSB3Tx\/aJwPLpsJro2rRdfybxi2+ivv\/++8U2vFTx6KNHj9LDDz+s1WpMAlKHoLZPBJZPh9VE16bN+tvxwzFqfEV1+uqrr6IDNElPvogaoY\/169cTfnopDRo08FItcB0m6sDQGfGgzRMdALN8RphZoEHM+Wg39ZyzQTw7757a1OC3RQO1E9ZDvog6rE7DaoeJOiwkM9MOE1lmcA+rV5v1x0QdlpXw9rwQkcxMUzZPdPaoM2NTYfWavWgbZS\/6mj3qMABljzoMFDPXBhN15rAPo2eb9cdEHYaF\/KcNJuoQwcxAUzZPdPaoM2BQIXapEvXawVdRueKFQmzdf1Mco\/aPWWRPMJFFBrWWjlh\/WmCNpFFriBqXCEyaNIleeuklcZQbW\/FwtPzOO+8M7WYX9IE9223btqX69evnURB71JHYrLZOmMi0QRtJwzbrDzs+sKCIEluPGtnypk2bRvv27aM2bdrQ9OnTBVHjMlvsn8blAuleJoA+kD510KBBIukTE3Ukcy\/STmye6Bz6iNSUQu+s1dQ1tGLrQdHuDxOuDb19vw0GCn3I28hBzgULFsw5rYj91aNGjaLhw4en7VVv3bqVnn76aTp58qR4GTBR+1Wt+fWZqM3XUbIR2qw\/SdSITcOjznQJlahBqqNHj6ZHHnkkLaKGV4522rVrRwsWLKBGjRoxUWfaUjT0b\/NEZ49ag8FE2OTvRv6LcDqxQcWiNK9n7Qh7du8qEFHL0MfOnTsFmT711FMi3PHOO+8Qbn3p3bt3WqGPuXPn0t69e6l79+40duzYpEQNsZYsWZJxIHUMAPgiAZatheWLt2Zt1d+1LdvToeuyhXJiTdQQAF4vbnKZPHky\/frrr4KYcXcicoEUKVIksAVu27aNpkyZImLTxYoVo+zsbPaoA6Np9oPsUZutn1Sjs1V\/K7YcpFZPrBHi929egfo3L58KCu2fB\/Ko1VHBg8ZlAueddx4VKFAg7QGvXLlS3BTjLPCu+\/fvn+vPvOsjbbgz2oCtE12CyvJl1LwCd64uJJqQ5wOCBCJqbJt7+eWX6ccff3QFo3LlylSvXr1QLhVgj7pCYIMz\/UEmMtM1lHx8NuoPcWnEp1HO+vl7+n5aWyOUFJion3jiCZo3bx41a9ZMhDq+\/\/57Wrx4MV133XViJwj2VOM+xXQLEzUTdbo2lKnnbSQyFUsb5VMPupz7yUzauWx2pswnV7+BiBrb8MaPH0833XQT1axZM6fBdevW0fz58+mhhx6izZs35\/pMh7Qc+tCBanRt2jjRbScym+VTvWlsyzs08y\/xzEctlZTophf176jLN7ykR3pMZOnhl+mnWX+Z1oD3\/kHSWEDET5SpHarRoPZ\/iDdRw6PGoZamTZtS48aNKV++fIQtewh3YGvdY489RjiwUqtWLe9IBajJHnUA0Ax6hInMIGUEGIot+nOSdIe6FwuiNolfAoU+oFNcUTNw4EARm0aMGjeT48QiQiIXXXSR2Fe9cOHCAOr3\/ohJQHoftfeatkyERBKzfN5twcSacdcfCBr5PGTeaWAsSRq\/m8QvgYkagmBrHggbh1OwPa9SpUp0\/vnni7\/jlCIWFXUWk4DUIWfcJ0IqTFi+VAiZ\/Xlc9QeC\/nDrQUHQMtQBpJ17pk3il8BEjVAH9k+DlGXBdr2JEyfS0KFD0zpC7tU8TQLS65j91IvrRPAqI8vnFSkz68VNf9KDnvPRd7kIGguHHeqWynOwxSR+CUzUiEUPGDAgF1EXLlyY7rjjDrrnnnu0e9OmfTXRMZXiNhH8YsDy+UXMrPpx0J\/0nldsOZCTtlSimIig5eexJ2oceMER77vuukvItGzZMkHOOASDePUNN9wQiUWZBKQOgeMwEdKRm+VLB73MP2uq\/pKRM1CTBI14dLKbW0zil0AetboN76effqKZM2dSv3796ODBgyJPBxYZEavWXUwCUoespk6EsGRl+cJCMjPtmKI\/ELMk5w+3HMjJI+1EJZUH7axvEr8EImp41IhD33777VSuXDmxHQ9hECRnwu\/Yuqd7DzWHPjIzOcPs1ZSJHqZMalssnx5kVY\/5mx+OJSRm1Xu+umJRavDbor4GFHuihrQrVqwQlwRg8RBpRnEbyy+\/\/EJ\/+tOfRPKkdG948YKoSUB6Ga\/fOjzR\/SJmVn3WX\/r6kN7yNweOkVuc2c1rPk3KxeiSYoV8k7Pankn8EsijlsJgCx4Kdn588cUX4vfq1atHspDIHnX6kyDTLTCRZVoD6fUfpv7kNjlsmwMhp\/KU5cgRzihXrBBd\/dtiFMRrToZA7IkaoY+3336brr\/++lyxaPwdiZiaN29OWVlZ6VmBh6dNAtLDcH1XCXMi+O48ggdYvghA1thFEP05PWQQ8o4Dp2PMqYpc+AvLY07Vn0n84sujxtHx9evXi9OIuHC2U6dOhC15smCRERfcPv744xyjTmUFHj4PMhE8NGtMFZbPGFUEGkgi\/aneMRr24yGrnjJ+j4qU3QCILVHLS21xGlFew6NeFlCoUCHq0qWLSH2K\/B+6i0lA6pCViUwHqtG1abP+QMb\/98XX9GvBYgJQScZevWM3Qsbfwogth6Vhk\/jFl0ctAUgU+ggLIK\/tmASk1zH7qWfzRAcOLJ8fa4i2biKvWJDy1oO+ByNjyZcUxwLfaXJPd7HP9yB8PmASv\/gi6lOnTolbXXB8PFGBJ33BBRfQWWed5RMW\/9VNAtL\/6FM\/wUSWGiOTa5isPzVWrHrEQYkYz6kxZNO84yB2YhK\/+CJqxKC7du1KuCAgUUFqU1x6y\/uog5hG7mdMnujpS8cedRgYurXhRsKo52fhzq1dScTYZQHPuMoFJ8R1e6Z7xkFxji1RBxVY13MmAalDRiZqHahG12bY+pPhCPzEvmLpCYdBwqpHLIm4XPEs4SWDiEXoonihXOCFLV90mvHWk0n84sujVsVD+GPTpk307rvv0r59+6hs2bJiux5+RlVMAlKHzLZPBJbvv1bjJOHTXvFR4QWnE45Q7dItNIHPExFxKpu2XX8m8UtgopYnE3GZ7aWXXkoff\/yxSM6Ey2gbNGiQSsehfG4SkKEI5GjE9olwJsh39gWlhFalF+wkYL+7JBLZmZOEU3nDYdir7foziV8CEbXM9YGUpurltp999hk9++yzIg8IJ2VKfyrYPhHiKp9bCEI3ActwBKxK3TXhFpJI3\/K8tRBX\/XmTzoIbXrxcbsuLiV7NIXE92yeCSfIlI9+wYsBuYQiVgFUv+DQh+0silL7F+WvBJP35G7m32rH3qI8ePUqPPvootWvXjurWrZsjNXvU3gzAay3bJ4Ju+aImX+hV3RlRrMAJ8c0ybgTM9nkagdgTNYRYvXo1PfTQQ3T55ZdT+fLlxbFyjlF7NXFv9XQTmbdR6KsVRL5MkK+TgLE1zUsIIoh8+tAOv2Xb5bOCqKF2hEBAztu3b6cLL7yQmjRpQmXKlAnfIhK0aBKQOoS2fSJAPrnYBvwSbTsTn3lM3ONHD859wU7ylQSd7BaQZP2dCfqrUKGCH8hjVdckfgm0mIhLbRcuXCh2d5QuXTqSvB5uGjYJSB0WGNeJHuVimxN33eTrR89x1Z9XGW2XzyR+CUTU2PUxbdo0mj17NlWtWpXat28vEjGpmfS8KjudeiYBmY4ciZ41ZSJkKtzgDDng\/wg7yJiv+H+x02GITO5+MF1\/OmwTbZpin7rkM4lfAhG1BAYXBuDQy2uvvUbvvPMO4fh4586dxQIj5\/pI33x0TAQ1768aapDbyzDqdI8ap5Jcer0ls4gqlz69s8G54GYi8aaSy\/m5Dv35HYPO+rbLZw1RSyPAKUXEqf\/617\/Sjh07ONdHSLPDy0RQvV1Bsi5Hi\/F3HTFeKaZbuMHp9Z6JR5C96C8kU8lIM7bLZwVRI5MeyPnNN9+kt956SxgKQiCtWrWiiy66KJK4tUlAhjlTJPk68\/1Kb9dU4vWLge0TneXzaxFm1TeJXwKFPnCBQI8ePWjDhg3UsWNHat26NVWqVClluAMXDgwePJhWrlxJNWrUEHux1X3YUk1Lly6lbt265WgtUUY+k4D0YmLOHL+6TrOpY1F3LLgdrkDdoLkevMicrA4TWboIZvZ52\/VnEr8EImoceNm6datYSPR62zgWIAcNGiRIvWHDhuIW8+nTp9OkSZOoRIkSuSzuxRdfFMmdUuUMMQlIKYDbnXD4LEiydbdpqIYZBMn+Z09vHGO8tk90li+zL5J0ezeJXwIRdRAAYLQTJkygxx57jIoUKSL2YD\/44IP08MMPC29cLRMnTqSmTZvmyiPi1memgQQp49Zk8XPLgcBknCjGm3XycE6+XxsW15w6ZCILMpPMecZ2\/WWaX1RNR0bUaqdYfFy+fDk9\/\/zzIttesWKnr+ZBwQW6Q4YMEYuSOP0IsAYMGCC8cOc9jJkAEqQ856PdnonZScJ+sprZPhFYPnNIN8hIbNdfJvglkR4iJ+rNmzeLC3D3799PAwcOFDFuNXxy6NAhcYs5QiSITa9Zs4ZGjhxJo0aNomrVquWSA0CiLFmyJIideX5m16ETNH\/DEfr422O0+lv3a+1LF8kv2qtTppD4h\/+XKpxf\/Axa5AXCQZ83\/TmWz3QNJR+frfrDCWtZsK5mQglE1PB6v\/zySxGjLliwYC5vGIIhlJEqdg0l9+vXj7p3706NGjVKiAW879GjR1PFihXFrhK16H7jSe85e9HXecYHT1n3Vfa2eywsnwkUEHwMtutPN7\/4Qd4XUcvLbQ8ePEjjxo2jvn37iotsZUFiJsSh4f0605zCM168eDH16dOHChQoIB5B2AMJnVQChvIR8sA2P9STRA3vukWLFpEQdSKCluTcoW6pSFJQ2j4RWD4\/U9W8urbrL7ZEnepy23POOYd69eoltu45Pept27aJWDO251WvXl0cPx02bJgIf6ghjd27d1P\/\/v1FO3Xq1BGkjVDImDFjBKnr9qhB0q2eWCMWCGUBQYOc+zfP3b\/uqWP7RGD5dFuQ3vZt119siVqqHfuop0yZIshUXQhMZhZyARGEu3HjRqpSpUrOIiHaw+3mIOj69evTp59+KsIdqfZbhw1k9qJtpIY5MkXQEkfbJwLLp5dIdbduu\/7C5pd09OE79IH90Oeeey4hgx7I11mwMwPhkDjl+nALdfRvXiFyD9qJpe0TgeVLZ+pm\/lnb9RdbopZXcN17773i0oB169blsZZEpwh1mFUYQDpJOtNetIqT7ROB5dMxK6Jr03b9hcEvYWnDl0cdVqdhtRMGkGq4AyQ9757aOdcphTXOoO3YPhFYvqCWYcZztusvDH4JS1NnNFGbTNJQsO0TgeULaxpnph3b9Rd7ok62+yMuoY8VWw6K3R0o8KSn3Fotki13fqaU7ROB5fNjDebVtV1\/sSdqp8lgUREnDZHyFAuJf\/7zn41Oc+rcgje1QzXqUPdi42aC7ROB5TPO5HwNyHb9WUfUUrvYETJ16lSxjxqJl3SXoED2nLNB5OtAAUGDqE0stk8Els9Eq\/M+Jtv1F5RfvCPovWaoMWrshx4\/frzYEeJ1f7X3oeatGQRIZ8hj7eCr0hmC1mdtnwgsn1bz0d647foLwi+6QA9E1CBkJP1Hvg614Aj5LbfcQj179kyZ6yMMgfwCiZBHrzkbctKRYodHg9+evrPPxGL7RGD5TLQ672OyXX9++cU7cv5rBiJqJGVav369SEmqFlzBheRJURx2Qb9+gUS4A2EP00MeElPbJwLL53\/CmvSE7frzyy86dROIqDEgLCDu2bNHZM9DmOO7776j888\/nwoXLqxzvLna9gOk6k2busvDCZztE4Hli2yqaOnIdv354RctACuNBibquXPn0ogRI2jmzJlUu3ZtccEt8nggI16qK7TCEsoPkHHzpoGR7ROB5QtrJmSmHdv154dfdGsgEFEjuT+u0Ordu7dIriQLjpTPnj2bhg4dKrxr3cUPkK2mrhGx6bh400zUuq1Hf\/u2E5nt8vnhF93WFIioZc4PXJml5p1O9HddQngFUt3p0aBiUZrXs7auIYXaru0TgeUL1Vwib8x2\/XnllyiAD0TUv\/zyi8glffnll1Pbtm3F4iFi1suWLaMFCxaIkEhWVpb28XsFUnrTGJDpOz1U0GyfCCyf9imitQPb9eeVX7SC\/J\/GAxE1nsWVW0j6j59ly5YlbM0DOSNPdeXKlaMYu6ddH1hE\/N3If4nxIOxh8r5pJ2i2TwSWL5Jpoq0T2\/VnBVFD+ydOnBBEvXfvXrHbAze1yGu2tFmH0rAXINVFRBNyTPvBxfaJwPL5sQbz6tquPy\/8EpVWAnnU+\/bto+HDh4tDLyVLloxqrHn68QKkuogYJ28awto+EVi+jE2dUDq2XX9e+CUUID00Eoio4UlPmzaNatSoQY0bN44kAZObLKmAVMMecVpElLLaPhFYPg8z1OAqtusvFb9EqZpARI0j5IMGDaIlS5aI+LSagAn\/h7dtQq4PNewRp0VEJuoop4C+vmwnMtvliz1RJzpCDpPHSUXsBsFP3SUVkHEOe3DoQ7f16G\/fdiKzXb5U\/KLfgv7bQyCPOsoBJusrGZBxD3swUZtiZcHHYTuR2S5fbIkaIQ9sv7vttttowoQJebLnwaRNCX2oh1zittuDQx\/BydGkJ20nMtvliy1RI+Tx5ZdfUoUKFWjjxo15sueZFPqI6yEXlWhsnwgsn0mvFf9jsV1\/sSVqVZWmZ8\/DIReEP+J2yIWJ2j9hmPqE7URmu3xWELXJ2fPU+LTJV22LiRIjAAAVfklEQVSlIhjbJwLLl8oCzP7cdv3FnqhNz56XvWgbZS\/6Wlh5HLflcYzabILyOjrbicx2+WJP1KZnz1Pj0z9MuNbrvDKunu0TgeUzzuR8Dch2\/cWeqE3Pnlf8gf8VBhfn+DRvz\/PFGUZWtp3IbJcv9kSNWWFq9jw1Ph3XbXkc+jCSd30PynYis10+K4gaVmti9jx1\/3Sc49PsUfvmReMesJ3IbJfPGqLWNTPgrQ8ePJhWrlwpEj8hS1\/dunXzdOcGJG4ZR44PFGTLQ\/gjrsX2icDyxdUyT4\/bdv0xUSexzyNHjoiET61bt6aGDRvSihUraPr06TRp0iQqUaJErifdgIx7fg9VQNsnAsvHRG0yAkzUSbSDyYvj6Y899pjIyocdJg8++KC4TLdSpUpJidqG\/B5M1CZPXX9j4xeRP7xMq81E7VEjOP24fPlyev755yk7OztP6lQnkDbk92Ci9mgcMajGRB0DJSUZohVE\/dNPP9EHH3xAGzZsyCXqBRdcQO3ataPzzz8\/LS1t3ryZunTpQvv37xd3M3bs2JHy58+fx6PGH5AXG+XJVQdp+v8dPP17m4upTpn4xqchw86dO0WSK1sLyxdvzdqqvyZNmuQoButlJpRAaU6xj3rkyJEiMVO9evXELeSyhEXUsj0YQ79+\/ah79+7UqFGjpKEPWw66SCHZIzNhigQfA+svOHYmPBl7jxpxY+zEwKJfqVKlQsV0zZo1tHjxYurTp0\/ORbkIe5QvX57at2\/viajjftCFiTpUk8pYY0zUGYM+lI5jT9TYmTFu3Di6++67Qyfqbdu20YABA8T2vOrVq4stQMOGDRPhD9xyrhYVSNsWEiEnT\/RQ5lvGGmH9ZQz6UDqOPVGfOnWK3nvvPbHPGXHk8847LweYfPnyEcIfajjED2pyAXHMmDEitFKlShVB3Niqh7YTEbVtC4lM1H6sxsy6TNRm6sXrqGJP1Ah9dO3aldatW5dH5lq1atGMGTOoePHiXvEIXE8F0qYTiRz6CGwSRj3IRG2UOnwPJvZE7VtiTQ+oQNqS2lSFiie6JsOJqFnWX0RAa+rGCqJGiGLLli20bNkyOvvss6lZs2Yi9wcW\/ZwhCk04kgqkTScS2aPWZTHRtstEHS3eYfdmBVHjaPfQoUPF9jy5hQ5b9jp16kQ33HBD2Ji5tqcCacPVW04heaJHYkbaOmH9aYM2koZjT9TY9QGSvuOOO6h06dJiV8aQIUNoz549NG3aNHH8O90DL140IYG0cccH5OeJ7sUKzK3D+jNXN15GFnuiVm94gcCSqNXfo1xMtHHHBxO1l6lkdh0marP1k2p0sSfq48eP0\/Dhw6lp06Z02WWX0YgRI4RH\/c0339CcOXPE71lZWalwSPtzCSTSmiK9KUrcc1CroPBET9tEMtoA6y+j8KfdeeyJGghs2rSJ7rvvPsJxclx2e8kll9CBAwdo\/PjxVKdOnbRB8tKABNLGHR\/sUXuxALPrMFGbrZ9Uo7OCqCEkEjOtWrWKPv\/8c7ED46qrropk\/7QEWAJpW44PKR9P9FRTyezPWX9m6yfV6KwhaggKjxqEnc5pxFSAJfrcSdS25Phgog5qEWY9x0Rtlj78jsYKot63b5+IUy9atEjsnz7nnHPoL3\/5C\/Xu3TtP3mi\/AHmtL4GUt443qFiU5vWs7fVx4+vxRDdeRUkHyPqLt\/5iT9QyzWmhQoWoR48eVLRoUZE3+sknnyQchOnfv3+e3NE6VAYgl338BWEPNUrcbx13YsQTXYfVRNcm6y86rHX0FHuixvY87OxAqtOSJUvmYPTdd9+JPdTwtKPanjdr8SfU6ok1TNQ6LFVzm0xkmgHW3Lzt+os9UePAy9ixY0Uy\/zJlyuQiahx46du3b2QHXrr\/fSllL\/pajMGmrXmQx\/aJwPJpZlLNzduuv9gTNdKcrl69ml544QVq06YNVa1alXbs2EHPPvusOD5+5ZVXChPR7VUDSJWo1w6+irCgaEuxfSKwfPG2VNv1F3uixn5phD2Q4yNZef3117VaIoCs+eBrtGLr6XsSf5hwrdb+om7c9onA8kVtUeH2Z7v+Yk\/U4ao7eGsqUdu2NY9DH8HtwpQnbScy2+WzgqixJe\/tt9+m5557TpxIRKz6zjvvpKuvvjrw7S5+JxiAPNh6hnjMtq15TNR+rcG8+rYTme3yWUHUc+fOpYULF1Lnzp3p4osvFqcTZ82aRbfddpuIW0dRytesR4euyxZddah7MU3tkPtOxSjGoLMP2ycCy6fTevS3bbv+Yk\/UyO2By2cfeOABcVGALLiYdsKECYS81EWKFNFuKeXqXU9HGvQT\/di2h5o9au3mo70D24nMdvliT9RqmlN1Z0eiv+uaESpRw5uGV21TsX0isHzxtlbb9Rd7osbJxFGjRtHvf\/97uvHGG0VMGicScS0XbifHjpCCBQtqt8LSN\/ShY1VbiX5s20PNHrV289Hege1EZrt8sSdqWDhyT+OoOH6WKFFCpDrF3YlTpkyhypUra58E6EAlatv2UDNRR2JCWjuxnchsl88KooaFY+fHV199RXv37qXChQtTtWrVqECBAlqNX238fzpNoxMlqog\/2baHmok6MjPS1pHtRGa7fNYQtTYL99iwJGob91AzUXs0AoOr2U5ktsvHRB3S5Cpx9yt06twSVu6hZqIOyUgy2IztRGa7fEzUIUweW28eV6GxfSKwfCFMhAw2Ybv+mKhDMC6VqG3cQ80edQhGkuEmbCcy2+Vjog5hAq3YctDaPNQSHtsnAssXwkTIYBO264+JOgTjmvPRbuo5Z4NoycY91OxRh2AkGW7CdiKzXT4m6hAmUPaibdZeGMAedQgGYkATthOZ7fKdsUS9e\/ducVUXLsTFFV4PP\/ywuGggX758uabV0qVLqVu3bjl\/q1WrFs2YMSPXRQTwpuFVo9h42IU9agOYNs0h2E5ktst3RhL18ePHxV2Kl19+ObVt25a2b99OAwcOFP9AxGp58cUXqWzZstSgQYOEU6XV1DXWXhjAHnWaDGnI47YTme3ynZFEjYRNyAEyaNAgKlWqlJhK2dnZIvte+\/btc02tiRMnUtOmTalmzZopidrWwy7sURvCtmkMw3Yis12+M5Kocdz8559\/FpfeIonT4cOH6cEHH6Rbb72V\/vjHP+ZMB3jeuOEcdzDiXkaANWDAAGrYsGGuEMnvRv6LsEXPxgsD2KNOgx0NetR2IrNdvjOSqNX5s2\/fPhozZoz407Bhw3LdWI7kTo8\/\/ji1bt1ahETWrFkj8lsjWx9yichS\/IH\/Fb\/m\/34jrRrW3KDpGd5QcCclQkC2FpYv3pq1VX9NmjTJUQxyGZlQ8v0b+UkjKvCq33jjDZo8eTJ16tRJ3AaTKh0qhjd69GiqWLFiTohEPexi480u7FFHZJCau7Hd47RdvjPSowbhvvrqq\/Tuu++KRUVc3+VWoHyEPFq1aiUy8UmihnfdokUL8ciZcCoRcppkKDo4jeXTgWp0bbL+osM6Mo8aW\/NA0FhMxEW4iQrqIc91r169qE6dOoK0EQpBqERe+6US9bmfzKQCOz6MDjHuiRFgBM4YBM640Mdnn31GXbp0of379+dSMsIazZo1o65duwqCrl+\/Pn366aci3LFy5UqqUaOG2C1St27dPMYBwsauDy6MACPACNiMQGQetc0gsmyMACPACOhEgIlaJ7rcNiPACDACISDARB0CiNwEI8AIMAI6EWCi1okut80IMAKMQAgIMFGHACI3wQgwAoyATgSYqHWiy20zAowAIxACArEjahyAWbhwoThSfuTIEbr77rvpjjvuoKysrBDgiLYJr7KcOnWKFixYIPaS4\/h98+bNxZZFpIo1uXiVT5VhxYoV9Oabb4o993HQqR8Z161bR4MHD6bPP\/9cbENFagQcGjG5eJUPOXqee+45mjRpEh07dkykL0YaY9NtNBn24JehQ4dS9+7dqVKlShlVU+yIesOGDTR27Fhh5EWLFqURI0bQ1VdfTS1btswokEE69yoL8p0goyBeTiVKlKAXXniBcIITkx6nN00tXuWT49+7dy\/de++9dNFFF4mXUhyI2quMOMiFw159+vQROWuQynfbtm3i7ED+\/PlNVSF5le\/9998XJ4+Rbx5pIWCvkAuJ1+JW4BjhLMf8+fNp8eLF9PTTTzNR+1XizJkzxSN33nmn+IlDMaqB+G0vk\/W9yvLSSy\/RTz\/9lCPz5s2bBWnjxGbx4sUzKULSvr3Kh0aQB2batGmiPUlqcSBqrzLiGxFOueHELS7KkNkkCxcunOfiDJMU6lU+SdT4JnTuuecKokY6Y+T0iVuBJ\/3yyy8LO\/zwww+FLOxR+9AijBsedKNGjXJSo8aFtJxi+pEFhnP22WfneJi4AQce2YQJE3JlHvQBpfaqfuSTL9xly5bRTTfdRM8\/\/7zwPk0naj8y4qWKgpQIH3\/8sQhfQcZEOW+0K8hDB37kQ+gD+eWfffZZ0TIu\/YB94htgXAty6OMbAUI4TNQ+tHj06FGRm7pDhw4ixocSV6IOIgu+kr333nvieD1eWMluwPEBq5aqfuRD3B3fEO677z769ddf6R\/\/+EcsiNqPjCAxEDTCdpdccgnhW9IXX3wh1hpSZZDUoiAPjfqRD+tG+Idc8sg5\/8QTT4hYtemhnWQwMFF7MBK3Kr\/88osw7BtvvDGHpEDUMH78K1asWMCWo3\/MryzffvutyN0tFzgqV64c\/aB99OhVPixWTZ8+ncqVKycWoKDPuBC1VxkBG4j60ksvFRdloGCNAess48aNMzZ85VU+We+aa67JyXC5adMmMSchX5zmpWriTNQ+Jryz6pQpU0QMTI1Rv\/XWW+JNbvLCmpvIXmXBIlvfvn3p5ptvFulfcUNOHIoX+aTXBh2qpXTp0vTMM89k\/CtnKpy9yIg2nLFeEDW+GYHATSYyL\/LJEEm9evVyETViu1gULlKkSCoYjfyciToNtair0FiIgZcJ8kLcOm7FqyyzZs0SebmxMOO8sd1kmb3Kp8oQJ48a4\/YqI+qBmGGveAlJ4sb2UpN16lU+NfQBYpYEb7p8HPrQxCDOfZ2Ia4LATN7ilAiKZLLA00LBYgbi0bNnz87VDC5SmDFjhrFfmzFYL\/IhhhlnovYqI+otX75ceJjY\/QGb7d27t\/Heplf54FXPmzdP7ET6\/vvvqXPnzmKrJeLVcS3sUcdVczxuRoARYAQygEDsDrxkACPukhFgBBiBjCLARJ1R+LlzRoARYARSI8BEnRojrsEIMAKMQEYRYKLOKPzcOSPACDACqRFgok6NEddgBBgBRiCjCDBRZxR+7pwRYAQYgdQIMFGnxuiMrYF8FDIVZyoQ\/NRN1Zb6OU7wYT8u8jb7SX2Kfejly5en9u3b++lOa11keuzYsaM48KLr1CUODHXp0kXIoasPrSBx464IMFGzYSREwA\/5+qnrB3KQ25w5c3yRNNoPg6hx2AP\/wjqyD1mQDtR5yMcPHl7q4lg+8kLj0Emms755GS\/XSY0AE3VqjKyugSRPuJUDRItsZ\/JE2WeffSa8PxRcyoDjzx999BGNHz9e3FBSoUIF6tmzpzi+j6xwal14vsjli2eQurRixYoi62HDhg1dj0sjSTvqgsiqVKmSU3fVqlV52lVTn6qn\/bZu3SoyKuJGDnjfIGrkhEFyoEWLFtEVV1yRc6OKPEU3depUkRypRo0a9NBDD4nxbdmyRdRDSgLkOf\/b3\/4m8ioDIySLQtvI3oh2ZSpWXOwAmYEPxjBw4EC67LLL8tiNStQyx8kf\/vAHWrJkicCpadOmos0yZcrkedb54kn2AmOitm\/KMlHbp1NfEiGh\/T\/\/+U+RlRBFTfiuesk4FgxPEEfaq1evLojqkUceEQQF0lbrIm0p6uErOAhv+\/btgrzwfO3atXOND6EVkDjqg0zxf7VuMkICQYLAQPLITPfKK68IskfKVBCrTCuKz5AnGWNGYnuMB5ndQOoIQyA5\/FNPPSVkh5wYd48ePUTYBHnAcaHBzz\/\/TPfcc49Iw4rnTp48KWTfuXOnyFeMpGDABfmmcYwanyH0ohY3ot6zZ4+oj1ttgOEHH3wgXobOo9dM1L7M2rrKTNTWqdSfQCBqkBiIGldEqTlTVPKVN5KAQPA7CBXkBOLD12u1LtrEV3zkKJG5lp1Z2OQoX3\/9dVq7dq0gfdk3EhYdOHBAkHcyogbBIfOczKR46NAhgmcNwgTp4r4+NcuiDKGAfPHtAUm98BN3GWJ8IPf9+\/eLxElIzwlPGuNA5sJ+\/fqRTC2LMUFevBCQbwWevby5Rab8vO6663Iut5CyJvKoZRz9u+++E30BV2fIgonan13bVpuJ2jaN+pQHN3PAi0OGPhAW7p8EQSJcoZIvvk6D0HGBKW7tAJEgs5q8pkiti9\/hFTsLLgl1xmfdYslqW4mIWg0duC0YJiM2kPPkyZNFSAQeddmyZWnXrl3097\/\/XRC1es0ZFucQCoGXKy9qVTP8gdyRT9tZ4OU7x+VG1OolGMmSADFR+zRsy6ozUVumUL\/ibNy4kS688EJBvvCUkRcaN4GDrN54442cXR\/wknE7OLxYeKJOUlHJ1c1Ldl4nJseJK8V27NghPEmk+4R3Cm8W44E3nIioZQ5kfAuQyfgRF8cNOK1btybEn9VdH2o78IIxfoQs4MWrtwQ5iRphHHjT+MaBEA8K8EGYBeEN3GSievUY\/+HDh6lQoUJ58qO7ETU87xYtWoh24VHjRQaPXvYlcXISNfSBi1fddsJwjNrvLDC\/PhO1+TrSOkJ40vAmkS72nHPOEV40yBs3nIMM4DXDO0a+YRA1SBQXNICosLgGLxQhAZCzrIsYsBp3xu00WCS7\/\/7788So8QwIE2ESLOp9+eWX4v94UYCEk4U+EF6BNyuJHSETEDDIDi+UZEQNAkY\/CFWAbNHPk08+SQcPHszlUeMbB8I7uEQY4Q2EV\/B\/7AQBSWK8Mk6O\/rDQir\/DC\/cSo0b7kF3mcMb43a7nQmhGhlhw0TF0IsfgvFuSiVrrlMlI40zUGYHdnE7lrg+QLohAvXQVOyIQrgBhgrix4CZ3PsADXr9+vYhFw0P98ccfc+qCqLB7wm0nhzNJPvrEbgnEZfGCUHdgoG4yonbmQL7lllvEywAhimShAsgMspW7QfAMbp3GQmG3bt3EpazqDe8gdVkfu1KAEbxwyAnPWeaZxvj97vqoU6cOvfPOOzkX3oKkMX4QdteuXcVLB23ilh+8WKAT7ArBbSpyDCBmtS4TtTnzK6yRMFGHhSS3YyUCbqEMhD7mz58vyNvP9W+pYtRhAchEHRaS5rTDRG2OLngkBiIgd3FgXzQW\/uTlwtdee63YX+6nMFH7QYvrqgj8P\/PRqvghCJplAAAAAElFTkSuQmCC","height":218,"width":362}}
%---
