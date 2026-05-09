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

model = 'psm_svpwm_nonlinear_ctrl';

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
run('n_sys_generic_1M5W_pmsm'); %[output:02666e55] %[output:47b973aa]
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
l1 = Kd(2) %[output:254225a9]
l2 = Kd(1) %[output:9ba7162b]

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
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:78531472]

%[text] ### PLL DDSRF
pll_i1_ddsrt = pll_i1/2;
pll_p_ddsrt = pll_p/2;
omega_f = 2*pi*50;
ddsrf_f = omega_f/(s+omega_f);
ddsrf_fd = c2d(ddsrf_f,ts_afe);
%%
%[text] ### First Harmonic Tracker for Ugrid cleaning
omega0 = 2*pi*50;
Afht = [0 1; -omega0^2 -0.05*omega0] % impianto nel continuo %[output:4a3235a3]
Cfht = [1 0];
poles_fht = [-1 -4]*omega0;
Lfht = acker(Afht',Cfht',poles_fht)' % guadagni osservatore nel continuo %[output:49427ef0]
Ad_fht = eye(2) + Afht*ts_afe % impianto nel discreto %[output:141fac74]
polesd_fht = exp(ts_afe*poles_fht);
Ld_fht = Lfht*ts_afe % guadagni osservatore nel discreto %[output:4590e09d]
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
kg = Kobs(1) %[output:89a6cd8e]
kw = Kobs(2) %[output:66f5352f]

%[text] ### Rotor speed observer with load estimator
A = [0 1 0; 0 0 -1/Jm_norm; 0 0 0];
Alo = eye(3) + A*ts_inv;
Blo = [0; ts_inv/Jm_norm; 0];
Clo = [1 0 0];
p3place = exp([-1 -5 -25]*125*ts_inv);
Klo = (acker(Alo',Clo',p3place))';
luenberger_l1 = Klo(1) %[output:5677cc0c]
luenberger_l2 = Klo(2) %[output:4646c5b9]
luenberger_l3 = Klo(3) %[output:536fbf1f]
omega_flt_fcut = 10;
% phase_compensation_omega = -pi/2-pi/12; % for motor mode
phase_compensation_omega = 0; % for generator mode
%[text] ### Control settings
id_lim = 0.35;
%[text] #### rotor speed control
kp_w = 2;
ki_w = 32;
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
figure;  %[output:042f8e6d]
plot(Zmodel,ocv_model,'LineWidth',2); %[output:042f8e6d]
xlabel('state of charge [p.u.]'); %[output:042f8e6d]
ylabel('open circuit voltage [V]'); %[output:042f8e6d]
title('open circuit voltage(state of charge)'); %[output:042f8e6d]
grid on %[output:042f8e6d]
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
%[output:02666e55]
%   data: {"dataType":"textualVariable","outputData":{"name":"tau_bez","value":"     1.455919822690013e+05"}}
%---
%[output:47b973aa]
%   data: {"dataType":"textualVariable","outputData":{"name":"vg_dclink","value":"     7.897123558639406e+02"}}
%---
%[output:254225a9]
%   data: {"dataType":"textualVariable","outputData":{"name":"l1","value":"  67.668222262819981"}}
%---
%[output:9ba7162b]
%   data: {"dataType":"textualVariable","outputData":{"name":"l2","value":"   0.283130953403918"}}
%---
%[output:78531472]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.283130953403918"],["67.668222262819981"]]}}
%---
%[output:4a3235a3]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:49427ef0]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:141fac74]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000","0.000200000000000"],["-19.739208802178720","0.996858407346410"]]}}
%---
%[output:4590e09d]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.311017672705390"],["54.332172227996914"]]}}
%---
%[output:89a6cd8e]
%   data: {"dataType":"textualVariable","outputData":{"name":"kg","value":"   0.039461503083742"}}
%---
%[output:66f5352f]
%   data: {"dataType":"textualVariable","outputData":{"name":"kw","value":"   1.254711180325163"}}
%---
%[output:5677cc0c]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l1","value":"   0.606931756868082"}}
%---
%[output:4646c5b9]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l2","value":"     3.449190983162578e+02"}}
%---
%[output:536fbf1f]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l3","value":"    -4.186041938481557e+02"}}
%---
%[output:042f8e6d]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAVsAAADRCAYAAABikxHnAAAQAElEQVR4AeydCbxN5frHn53uP1FClEI5QlEq3Qxlyk0oFbp1w226DYhSoU6XBg1mTVJUmkWjlFs0kalLklAUQsYGU1TG9Pd98+67zjp7WOucPay9z+PjOWutd3ze31rrt5\/1rPd91gF\/6D9FQBFQBBSBpCNwgOg\/RUARUAQUgaQjoGSbdIi1A0VAEchoBBKkvJJtgoDUZhQBRUARiIWAkm0sdDRPEVAEFIEEIaBkmyAgtRlFQBEIGgLB0kfJNljnw5M2H374oVStWlVGjhzpqbzfQgsXLpRTTjlFevXq5beq7\/K\/\/PKLdOzYUZo2bSo\/\/vij7\/qJqACOjJdxJ6K9VLWBvuiNsO+13yBg7tQV3RkD17UzPdv2lWyz7YwmYTyQIGQIKXKjJqGLcJOp7ItOudFHjBghDzzwgNSuXZukqOJXN8gjWT+KnIcBAwZI6dKlZfz48XLkkUdG1TvoGeB+zjnnSM+ePYXzEXR9C6qfkq0LOaw5bhAr3DAU4eKGbBAubpuPVUS+FWd9CIoblDzaoQ43NW2wz5Z2yY8ktE05hF\/+aBcifdLXSy+9ZCxe+qI9ylOP+gjtOdNt\/+jAPm1YfSmHcHzJJZfI6tWrZdasWdKpUyehPHlW6M\/ZPvnu9tCRMlaoY+vb7U8\/\/SSR+kIHdLN1acvWYcsxeYwVfNm3YyWfvkizwjHpyPPPP28I6+STT+bQjA3dbVn6pX\/Ej270AVY0Onjw4PBTCOm2bbYcUyaS0Cf9Uw5hnzTwpW3OB+elbdu28sMPP+RrgrLUoS4CTu5CH3zwgblmyHdiRh9OHNgnjfroTHmuN9q37Tqvt86dO+d7YnHmU9\/Z35VXXknTwvkQEbOfbX+UbB1nlJM\/btw4R4oYcuHisolc4D169LCHwo1k87nonPW5EW6++WZzA9sKjz32mCEtjmlr9OjR7OYT2qRtm7Ft2za54YYboj5q09edd95pi5tylKeeTaQ92rXHidpCVJUrV5Zly5aZJn\/77TdZu3atVKxYUUqUKGGIxokLhSALL7r8+uuvAoaMj3oIbXGu2GfLMfuMFXzZt0If9GWP2VoLCjKaM2dOWE\/y+vbtGz4\/HNMv\/TMmjp0STzdnWfZj6UK+UyA2+qV\/m84+hL9hwwabFHUbqT44gZetRHvOawYLH0Ik340D1ypp5FmhLm1wDJbO6w0Spw55CO1edtllwjniGHFej1jmWOmcD9oiP9tEyXb\/GeUEjx07Vg499FB56623ZPny5XLbbbeZ3EmTJpktf6LlU58LpUGDBrJgwQIj7HPBIdRFSCP\/ySef5FCmTZuWh4xJ5EZ55pln2BXKoctFF11krEvqmowIf9CXss2bNxduLG4E6pFGO1RxjoXjeHLEEUfIa6+9JpAputPOIYcckqcaZerWrSuMHxzQkb6bNGkikJQXXGmwfPny+foKhUKGuOmbdsESXcANa44tddGLfMpxbIXxus8ZNzwWFPW3bNki6MmYwH3dunVmrPRj2\/vqq68MSbhxCIWi64Ye6IQenJcuXbpILF0o5xT6R2gHPRD2wZUfNdrmGCwgNh7FY9XnmgYH8OJHwpalHds2uIBJNBzs+bV16Z+6Q4cONdc7utk02rXl2M6cOdNgSLrzeuQ6pz9+lPlx5nygA3WyTZRs959RTjAn+sQTT5ScnByTClFxMXORWWvCmd+wYUNDztygK1euFOpzg2DpIeybhhx\/7I3NLzkXvyMrvAtBYRnSN+2QwQXNRQqRcuwW2kIfm84NyX6rVq3YCPWoTzsmIcF\/qlWrZsYPjvRt9eEYXJy4RcI1mjoQ+dSpU2XMmDHCUwA3Mzc15bdv326I2OIEYV599dVkGeEm5txAIm3atDGPy1hTJnPfH3Qjb9+u+U\/9o48+2vyo0Q\/Y0978+fMj+nNj6WYadPyJp4ujqNkFQ3bs9YJu6EIapM02lrjrQ8aMAxxLlixpqjpxY9wmcd8f+qIcuEOm4BDrWt5XJfxUY\/UFP4Q8xOrDUwYuBLakFyVRsk3w2YZIIDWnQHQJ7iZwzVmif\/\/994217iTXwigLSeEv5AaF0LnpIQk\/bVKees5zwo+OJQBnW6RDMJYoIAX8kljsznLsF0S3aLrQXpAElxiYg5ETj8LoyA8wFrbzPEDqkLs1MHAlYIgUpp+g1lWy3X9mOMGcaB4ZV6xYYVLtoziPyOXKlTNp3LQIBzyKYhlhFVSpUsW8aMEKjnRjUt6ruB+pnDc1fj8v7UBMlONmYYuvjpuHm4hjBOuZi5zxMm7SCio8DUCwc+fONdamtXC84hqtX7BGeBTnR4sb31q2tg7HpIMTj6U2nZuYc2PzbbrdWozsMecNYoVg7eMu\/Uarj14IZaLpZtuOp4stZ7dWNx77GRdix2afVmzZSFtb355\/rhvOPz9cTjdCpLr4V3FNYDh02ef+8HN9WH3BBbHtow\/3Cu4Em+bc8pTBExD3Gk8Mzrxs2Vey3X8mOcEdOnQwfiX3I6f74uZm5MKFjKlOPvW5ULgxsYrIR7h5uYkp51W4MSErLk50sY9xtIt4aQdLE0uCx2b0YMsxb30tAVpd6YO+4rXLzcPYufHdZa3OlOGmoX\/KgItXXClvhXboq1SpUsZVg\/6MgzRbhh9A2uaYdIsTx1Y4N+yTT30rkA84gAn5CLpyDumbtihLv5ShLGUQ8mkvlm6Us0Ib\/NjF0sWWtVvOM0Jf6IKwj2XMvi0XbUsZynKNMg70pSyuCOtG4Dgsjh3GiuFh63q5PiBm+kNH+rb92Wa5HsARLNDHCrhQBsOFPvnh4jgbRcnWcVb5FeeicSSZF1RYLTaNC+q+++6zh+Ylms3nEdRZn4tr+PDhwk0cruBxx60L\/T788MMCqXlpAh8d06CcZTkmHX14k2zzunXrJtzY9ti9tZa2O919bG8oLFwsXZvvHgvpWI4WN46tuPuqVauWMAeTfPDksRNdscSxuHjDzTH5YOQ8N6TRB32xb4UbmnRLKtYaI5837rY9jhGLm1\/daB+daQOhz2i6kO8UzjPnmzHZdPZ5Scf5s2nRtpTh2nP2b8cdrY5Np679ESMNnbmu+XHmCYI0t1An1jXFdYfP3akPbXJtWEua+rTjbjtbjlNKtlhPV1xxhWBVSIR\/48ePNy8x7K9eQazCCM36SoIwnT4lbhB3A9z8tgwXizPfWZ8XElxk5NMOdWx50smHPLixKOMWZ1u8rLAXorstytEWbTrbsOXoF+HY5rNPGsJUKPSwfdAO7dEu5dGPfMqy5Zh0t9h6kcrQFvWt0D\/1aYvytm97TDnSObZ10QkiJJ19CB0LCmsKQqCNefPm0azw2Gp29v2hL9qzYs8BeEIq1p2yr6j5MaN9W5Yt9d15lImlG1gg6Ekbtk\/a4tiKTad9t6AfY7Jl2SeNcvSNDs400p3i7J82bF+R6lqM0Y82KEsdhDRnPsekU4ayCPc054LzAyG3aNFC+EFkhgE\/UpRx60ObznTa5ThbJWVk+8cffwhTgKL5bAB4zZo1wjxJTiQS60KivErRRgDSwN0CCtzo\/Ejz6Iv1hFVJejzBGsftQb14ZYOZHwytcB1gefPDx751PeAz5zwFQ8v0apEysuVXbvbs2dK4ceOoI16\/fr3gh4taII0ZXDDxLIk0qldku8a64ofZKViTWFFeQKEc5WnHS3ktExkBLG6MI+d5YN9ar5FrFa3UlJAtL1TwH2F9RCNT3pAyJxK\/TfXq1Y0PccKECYJFXLROiY5WEVAEshGBlJAt7gMeMerVqxcVw127dglvSfv06SNff\/21DBo0SIYMGSKff\/55vjo8LqpUzePfVjwyDg89f1VTc854kZqPRNKQkHSyXbJkiUyfPt3EGAiFQlGHWKZMGcH6xc1QrFgxs4Ty9NNPN0tAI1XiESWbhDFm03jsWHRcy83Sb4tH0LfZeL7OPvtshpV2STrZrlq1SmbMmCH169c3v+S8iMCd4JxcDwqLFy8206x27tzJYVh42RE+0B1FQBFQBDIUgaSTLdM5nL\/mzK1jmo7bcc4LKJby8Tbz999\/N0s+mX\/HJPMMxdaX2h999JGv8plSOBvHBfY6LlBQ8YNA0sk2ljKsrGIuLXP08OnycoyAyLwgw3fbu3dvqVGjRqwmNE8RUAQUgYxAIOVki0WLtQs6drqIPWZCNBGNsIRxPTRr1oxiKoqAIqAIZDwCKSfbjEdMB5C5CKjmikAaEVCyTSP42rUioAgUHQSUbIvOudaRKgKKQBoRULJNI\/jatVcEtJwikPkIKNlm\/jnUESgCikAGIOCJbH\/66Sdp3769WdVFlKV4kgHjVhUVAUVAEUgpAp7IlmAwrOzq2rWr5ObmxhTKpHQE2lmQEVDdFAFFYD8Cnsg2FAoJiw5YY9y6dWuJJZTZ37ZuFAFFQBFQBPYj4IlsDzroIGGBwYEHHri\/WvRN+fLlo2dqjiKgCCgCRRQBT2SLC+Gpp54SQiTaz9rs3r27iEJWNIato1QEFIHEIuCJbLFWCeT99ttvG3dCjx495K9\/\/avce++9QqxIfLqJVUtbUwQUAUUguxDwRLYMmRizNWvWlH79+sncuXPl6aefFmYpnHvuuUJsgzfffFPU2gUpFUVAEVAE8iPgmWydVf\/yl78IoQ+HDRsmEydOFL73zmeXN2\/e7Cym++lAQPtUBBSBQCJQILLFoiUmLV9VwLLNyckRyPbwww8P5CBVKUVAEVAE0o2AZ7LFan3ppZekRYsW0rBhQ2PREm\/2s88+E8Im1qlTR3A1pHtA2r8ioAgoAkFEwBPZEuS7bdu28uijjwrbKVOmCD7a8847T\/jCQhAHlnk6qcaKgCKQzQh4Ilu+A8bUr5kzZworxCpWrJjNmOjYFAFFQBFIOAKeyHbbtm3St29f2bhxY1wFsILjFtICioAioAgUMQQ8kS2YbNmyRQYOHCj4aWMJZShfhESHqggoAopAXAQ8kS1uhFtvvVWIe8DLsVhCmbi9agFFQBFQBIoYAp7I9uCDDzaxEWIFoHHmFTEMdbiKgCKgCMRFwBPZxm0lQwuo2oqAIqAIpAoBJdtUIa39KAKKQMoRmLFsi\/zS6DbpNnaxsJ9yBRwdKtk6wNBdRUARyC4EVm\/eIXvKHS9j53yf9oFlJtmmHTZVQBFQBBQBfwj4Jlvm3BJi8YQTTpCmTZvK5MmT5fbbbzcRwPx1raUVAUVAEUguAqs27UhuBz5a90W2xK196KGHTCjFF198UY466iipVq2alChRQh588EHZs2ePj661qCKgCCgCqUPgmLLFU9dZhJ58ke2GDRtk0aJFwpzbY489Vg444AApXry4XHXVVSaIOMFqIvThSNJdRUARUASKJgK+yDYaRHybjBi30fL9puOqsJ\/f8VtXyysCioAiYBFYtWm73U371hfZlitXTmrVqiX333+\/LF++XH7\/\/Xf54YcfZOTIkeZzOWXKlCn0gHBVjB07Vgh6U+jGtAFFQBEo0gisdvhsM8WNYE5YDbEAYQAAEABJREFUKBQSXo4R9evyyy+XOXPmmJCLuA9wLWDhmoKF+PPVV1\/J7NmzhcDkhWhGqyoCioAiEEYg3USLIr4sWyoccsghcvfddxvfLaSID5c4t4mwan\/55RcZPny4dOrUSbCi6S+aVK1a1fiJ+eBkNsiaNWuyajz2nOi4VmTUec2m8wVHfLJwmaGQY8qk9+UYSvgi261bt8qgQYNM5C8Il5kJ99xzjzkmEhhfcli7di3tFkhwH1SuXFnq1asXtz5ujJycHMkWqVSpUtaMxXlOdFyZdY1m0\/n6+LNFsrdEubhckrACcRryRba0xYyEd955RyBeezxhwgQhfdasWdKuXTuSfcuSJUtk+vTpxqoNhUK+62sFRUARUAScCDjn2DasVvj3Sc62C7Lvi2yZcbBr1y7hY4887vfv39\/sP\/HEE0JksMGDB5tP5xREkVWrVsmMGTOkfv36gvk\/btw4Q7y9evUqSHNaRxFQBIo4AoPfWxFGoOFxpcP76drxRbZMyeLLujwmOhVmYQPp5LvznOVi7Tdv3tzMcMA9gFx00UWGyPmYZKx6mqcIKAKKgBsBrNoZ324xyQdu+EYaVYtFtqZY0v\/4IluCiPOCDHcBU7TQji0vytjHul227E+HNMcqioAioAikA4Ebxi4Od1v867fC++nc8UW2kGn37t3lkUceMY\/7TZo0kTPOOEPuuOMOYSrYypUrzeqyRAwIixZrNxFtaRuKgCJQdBAY9N5KsVYtU76wbIMwel9ki8InnXSSTJo0Sfjabm5urgwZMsT4Ws8991w54YQT5P3336eYiiKgCCgCyUYgX\/sQ7aD9vlqI9u2udfKVSVeCb7LFbYBvliA0devWleOPP15Wr14tHTt2lC1btkjJkiXTNRbtVxFQBIooAvho3USb2zJHINygQOKbbEeNGiWQbIMGDcQKL7OqVKki+HODMjDVQxFQBLIfAUuyp97\/X3FatB3qHiUd6lYIFAC+yPbnn3+WKVOmyJgxY+SVV14xftqlS5fKDTfcICeeeKKZ\/hWo0akyioAiEEgECqtUJJKlTSxZXAe5LatwGCjxRbY7d+40yjO9q3Tp0rJx40bZvn27XHjhhTJ16lSBjE0B\/aMIKAKKQIIRgGD5vM2Fj80TpyVLN5Bs7j63wRd3nBEo1wG6WfFFtsxGINgM07sgW3y3CI1BtJaMOVZRBBQBRaCwCFiC5YONECxbO9OAtp0kmxtAaxYdrfgiW+bZdujQwUzvWrdunRx33HHSpk0bIxUqVJCyZcvadnWrCCgC2YhAksfkJFdrwUKwWLTOriHZxzrUFCzZoJOs1dsX2VKJKV64DPDRElbxgQcekGHDhsnAgQMFq5cyKoqAIqAIeEEgFrk6LVjagmBz97sKINmgvQBDx1jii2wJPkNchN9++80QK24F4s6ecsop8uqrrwohEmN1pnmKgCJQNBGAVBEsVCxVrNayPaYY3yvHpEciVwjVWrAQbO4+VwGkm4koeiJbXoIxC+Hdd9+V119\/XdgS+cvKm2++adIh4UwEQXVWBLIfgdSNEFKdsWyLQKAQKcSKvxXhmHQ3saIdJGrJ9e2udYyLAKIljTzKZLJ4IlssViJ7Eenru+++MwG+iWtrhTi2l112mZQvXz6TsVDdFQFFwCMCECoCcSKQKKRqrdULH58npJEXjVgbHVfazIWFUN3kGoTAMR6h8FzME9lCoi+\/\/LKxaHv27Gm206ZNEyss0b300kslFNI4tJ6R14KKQMARgEwRCBOBPCFULFQrpCHkRyJVhohVGolY3+5WRyBaLNdsJFfG7hRPZMuHHQmhuGPHDmnbtq2w\/fHHH8Up5FPO2bjuKwKKQKERSFoDECliH\/kHvbdS+n64QSBUa6FCqpApYgmVOpGUikSqmx5sZtwBRY1YI+HjiWxZvHDxxReHl+faZbrOLfmUi9SJpikCikDqEYAUkRn7\/aeQKaQJmUKiVuwjP8tdJyz+JRwxK5LGECqCNYpgmeICUFKNhFbeNE9ke8QRR5gVYgT1jiZMB6Nc3ub1SBFQBJKFgJNIsTohUgQydVqmTjKlHI\/71I2mF2RqH\/tzW+aYR30noTIrAJJFINyi4AKIhpWfdE9k62yQqF\/z5s0zX9glJgLzayFgZxndVwQUAZGCYgARItYihSAhUQQixSK1ZGqJlDzKIZBprL4hUwSiRCBNyBSZe2OVPI\/9uS2rmJdYSqixEPWW55tsiWXbrVs32bNnj9SpU8d8ygY\/7sSJE731qKUUgSKMACSKQIqI89EeIrUkCqFGI1Lqx4IQIrWWqZtM7eN+JOtUCTUWqoXP80W2xEEYO3asPPTQQ9KvXz+55pprzHfCCCROOvmFV0lbUAQyDwEI0GmJukkU8nQSKZYogp8U0sUaReKNPBKRWssUArVkal9IkQfhQqRIvPY1P3kI+CJbFjdg0ebk5OTRiGPSyc+ToQeKQCYisF9nCBSJRKKdxn1v3to7SdRpibpJlHb2NxtxA4kibosUsuTxPhaRWjKlfsTGNTEQCPgi2xIlSshf\/vIXWbx4cR7lV6xYEV6+mydDDxSBgCEA6SGRCBRLk0d5J4GyH4lE567dYd7a01asIUKAiJNEcx0vnSBSa41CqG6LVIk0FrqZleeLbPkSw9VXXy0sbOAzOL1795ZOnTrJddddJ0QDIypYZg1ftc0GBCA8pLAEah\/naSseLkeXOtDETS0IieY6Xjrpo308pLMn3xfZMuymTZuaFWTNmjXjUGrWrClvvfWWEA3MJOgfRSABCEB4SCQCxfpEsDqdftALH\/9ziSgWqn2M90OgqI0VijhJlEd5BCsU6xNLdMKVlfK8tSdfSRQEVaIh4ItsN2\/eLCNHjpRff\/1Vrr32Wunfv7\/ccsstgs82WgeargiAAMSJRCJPyBHyRCBPBCJFIj3C8yIJoT3ajieQJxKNQJ0kCpki+jgfD1XN94uAL7ItVqyYmeqFFdu8eXNhFgIE7LdTLZ\/5CEB0SDzyvOD5NRKPPK31CYF6QQbiRCBPBL8mgnWJQJ4IpIkVyhaJRqA8ytOel761jCJQUAR8kW2pUqVk8ODBsnDhQhMsnC1uhUsuuUQmT54sGhuhoKch\/fUgTiQeeWJt+iHPdVv3eBmcQHaIkzxz979IsgQKYbrJUwnUE7xaKAAI+CJbqy8zEurWrSuPPPKIvPbaa2aGwj333GM+AGnL6DZ9CECaVhJNnrTrZWQQJ\/LXisXNCqQOdSuImzzjWZ+5+18kUVetTy+oa5kgI+CbbLFemfrVp08fqVevnhDH9qSTThJi2hKKMciDzVTdIDgE4kR47EbwdSL4OhG31clxJJ8ndXlkR2jXCy4QJxLL8oxEnk9eVMGsrcc6dZOnEqgX5LVMtiDgi2wJo9iqVStp166d8CVdPpEza9YsYQpYpUqVNJ6tj6sCkkMseRJtCeJEIE4EsnQ\/sjvJE9JEIE2E9ryqAHEifsnT+dgOeWJ1IhAnQpteddByikBRQsAX2fLNMWYg4KsdOnSoiY3AS7N4gH355ZcCSVetWlUaNWokfGInUp3x48cLZazgDyZmbqSyQU6D9CyJQobxCJR84ohSFoE4EdrxMk4Izkos8sTnibj9nlidiJKnF7S1jCJQMAR8kS2LGvDV4rP12h3xEiDorl27ypIlSyQ3N1f4nM769evzNbFmzRp57LHHzIwHIokFOWwjRIhAjpAllihiLVG3BQp5ItTJN3BXgpM4neQJISI8riNu4uQ4luVp23V1p4eKgCKQAgR8kW1B9Fm9erXs3bvXWLR86rx+\/frmhdqGDRvyNQcBlytXLl96EBIgSSex8oiPQLSkQ6RINF0t0TnJM9fxth3fJmTptjqd5MnjOsLjOkKb0fqLm64FFAFFIKUIJJ1sa9WqJXy\/rGzZskIs3NmzZ5s4ChUqVMgzUBZKrFu3Tu68806pXr26+SrEhAkTTJ08Bfcf4GogJkMyZeb8pTLs3YVyzgP\/FTex7lcjvGH5JsLb9wtqHiJ9m5czAomy2ujNf1YQ5JHzSkuvBsWN\/KPGH9Kg3HYjR\/6xQX7\/eb0kczzpaJunlXT0m+w+dVwrAn+twhFI+CZN844vsiWqFy\/E2Dr15nju3Lmye\/duZ3Keffy8p556qomrgC+2dOnSefJ37dolJUuWFGY5fP3118bVMGTIEPn888\/zlLMHuBlYuZYMKXbYUTJ01g5hQj6+VIKO2H7ZYlFaC9U+1n\/Zt7EgH\/Q8Q56\/rq50P6+2kYsb1ZSGp1Q3q+xi6coLxlj5mZqn48qJe+6DdG6z6XzBEQj3bBDkAC9KMN2LmQhYCw888ID5RePFlZVvvvnGLN2NtZqsdu3aMn\/+fCH4uBVn32XKlDGfSG\/cuLHw0q1JkyZy+umny5w5c5zFkrqPqwC3AFYsrgHbmSVXHvutr9Q+3tvHels2+VvtQRFQBDIRAU9ky4cc+aDj+eefL1iwbBs0aGAe9dmyggwyhTDdIMyYMUN69eplpoqRh1nPvNy1a9dyGBbm7jKVjCll4cR9O6mIJAbJEuw5EslCsPhSIdfcllWkUbW8Fvk+FfW\/IqAIKAJxEfBEtnzIkZkB+FshWraY51aWLl0qffv2NS++3D2y0AGLdsGCBWY5L9PAFi1aJHXr1s1TlJkORA\/DTYElPW3aNLMs2F0uT6UEHEC0Fz4+T4gSZZvDkrUWLARr03WrCCgCikBBEfBEtpDfpk2bBD8rL7B40WVdCHaLm4FybkVq1Kgh3bt3lxtvvNG8+CJa2PXXXy+nnXaaUBf\/7YcffiiVK1c2L8cGDBhgyuG7ZbEE9d1tJup4xrIt5sUXhEubkGxuyxzBkk2CBUsXKoqAIlBEEfBEtrgRbrjhBsE3izsB14FbSKecuP6FQiG54IILBIsVS5gtx6FQSKzFTAQxqtEm\/lzK4X6wMXPJS7Tgk8Wite3ywgtrNrdlFZukW0VAEVAEEoaAJ7KFFMeMGSP4WnEnQIZuIZ1yCdMsiQ1BtLwIs13k7rNm8cli2do03SoCioAikEgEPJFtIjtMd1u4DNxEmxvHmk23ztq\/IqAIZD4CvsjW+liZUeAWfK\/kBxkSiNbpOsjdZ9HmKtEG+ZSpbopA1iDgi2xxE+AusC6Eb7\/9Vj755BPp3LmzCbXIzIMgI8OMAwgXHZkfm6tECxQqioAikAIEfJGtW59QKCQsu+3SpYt5eRZrUYO7bkqOHZ3gp0VIwjfLyi\/2VRQBRUARSAUChSJbqyBLbbdu3Wrm0dq0IG2xZrFq0QmiHd6+JrsqioAioAikDAFfZMtc2vbt2wtLaZ3SokULM40r0gqylI0kRkdYtBAuRRoeV1pXgQGEiiKgCKQUAV9kyyov\/LPEpHXKCy+8IHyDjBCKidE+ca1Ask6rVt0HicNWW1IEFAHvCPgiW77UcNZZZ8kpp5xivj\/WunVrISZCpUqVTPAY792mrqQlWnrMbZnDRkURUAQUgZQj4Its0W7UqCS2nDYAABAASURBVFFyzjnnCGEQOf7444+FaV8TJ07kMFCCVYsLAaXw1TIDgX0VRUARUARSjYAvsiU+AnEMWE1GKESUveKKK2TkyJEyduxY2bZtG0mBEUu0KKRWLSioKAKKQLoQ8EW2e\/bsMe6CihUr5tG3WrVqQh5BxPNkpPHgT6v2z++cqVWbxhOhXSsCioBBwBfZHnbYYcJLsjfeeCM8zYsIYASX4SOQJUqUMI0G4Q9ki6ALMxDYqigCioAikC4EwmTrRYGDDjrIhEt8\/fXXhU\/cMP2LUIn9+vWTHj16GCL20k4qygx+b0W4mw51jwrv644ioAgoAulAwBfZoiCRvwiDOHr0aGH614gRI4RA38xQID8IgkU749stRhVCJ2psWgOF\/lEEFIE0IuCLbPn6LYsaWNwAuTL1ixi0WLxpHEO+rmfuJ1oyOtRTqxYcVBQBRaCwCBSuvi+yJRBNzZo15bPPPhN8tYXrOnm1x36qL8aSh662rAgoAgVBwBfZEmiGb4j17NlT6tevn2fZrrV4C6JEIus4XQjHlCmeyKa1LUVAEVAECoyAL7JlJkLXrl3l4Ycflrvvvtv4bPHbIizjJb\/AmiSootOFcJuuGEsQqtqMIpDRCARCeV9ky3JdvguGr9YtpJOf7lFZFwJ66IsxUFBRBBSBICDgiWx5Ida9e3dZvHix4C5gypdbSKdcOgfldCEwCyGdumjfioAioAg4EfBEtrgHOnbsaAKF4y7AbeAW0innbDzV+5Ct7bNhtTJ2V7eKgCKQoQhkk9qeyBb3AFO8iFcbKerXmWeeKUFwIzj9tbpqLJsuUx2LIpD5CHgiW+cwgxz1a+ayzUZVYiGov9ZAoX8UAUUgIAj4ItsgR\/3ChWBXjemUr4BcXapG0UVAR54PAV9kS2SvYsWKSRCjfkG2dnTqr7VI6FYRUASCgoAvsg1y1C\/11wblklI9FAFFIBICvsiWGAhMAQti1C\/110Y6vZqmCPhFQMsnCwFfZIsSyYz6xVLgVq1aSdWqVaVRo0YyZcoUuowruBDUXxsXJi2gCCgCaUTAN9miK4HCEx31a9u2bdK\/f39hOfCSJUvMUuBBgwbJ+vV\/BpWh32gC2do89ddaJHSrCCgCQUKgQGSbjAGsXr1a9u7dayzaAw880AS6gdQ3bNgQtzv118aFSAtkPwI6woAjEBiyrVWrlrz88stStmxZIXzj7NmzBdKtUKFCRAhxNaxYsUKQD7\/8n\/Ub+vUnk0Z6JsmaNWsyUu94GOu4\/rxG4+EUlPxsOl9wBBKRQNKQGBiytWNfuHChnHrqqUIYx6ZNm0rp0qVtVp7t8uXLJScnx8jBxYubPBYzNDylukmzeZmyrVSpUkbqHQ9fHdef12g8nIKSn03nC45ADDkE4I9vst26dau88sor0rt37zyCf5W8wo6pdu3aMn\/+fOHTO1ZitYm\/Vl+OxUJI84KOgOpXNBDwRbY7d+6U2267TZ588knjX00kRDNmzJBevXoJfdAu5j8zH9auXcthVIFsbaa+HLNI6FYRUASChoAvsv3555+FrzU8++yzMnDgQDN7gBkECFHASpUqVeDxlS9f3li0CxYskN9\/\/12YBrZo0SKpW7duzDZXb94RztfgM2EodEcRUAQChoAvsiX61+GHH56UIdSoUcN8Jv3GG2+U6tWry7XXXivXX3+98Kn0WB06LVt8trHKap4ikFAEtDFFwAcCvsi2RIkSJpTikCFDZOnSpfLjjz+GhcDhWKQ++s5TNBQKyQUXXCCzZs0SnNpsOQ6FQnnKuQ+cK8eUbN3o6LEioAgEBQFfZLtx40YZPny4vPPOO9KyZUshxq2Viy++WMhP9cD05ViqEdf+FAFFoCAI+CJbPmU+depUY3lifTqFdPILokRB6zhdCJXLFi9oM1qvyCGgA1YEUo+AL7JFPRYcMBeWF2RM98KdwLfJSCc\/leIk22PKHpzKrrUvRUARUAR8IeCbbJn7es0118i3334rWLMEFCcS2JgxY3x1nIjCukw3EShqG4qAIpAKBHyRLcFixo4dK48++qj069dPWN3FyhesXPy4TA1LhdKR+jhG3QiRYMm2NB2PIpCxCPgi2+3btwtfa4BgnSOuXLmyObQLEsxBCv7oTIQUgKxdKAKKQEIQ8EW2hx56qBxyyCFmepZzmteyZcuEaWFIQrTy2Miq\/Qsa9JtjHgHTYoqAIpA2BHyRLYsaunXrZlaOtWnTRnhRdumll5rFCJdffrkh4lSNhJdjSKr6034Kh4DWVgSKOgK+yBawCBr+\/vvvy+233y7t2rUzK73effddIUIX+ekQjYmQDtS1T0VAEfCDgG+ypXFiILRu3VpYWtu+fXtJ9fxadHBatRoTAURUFAFFIMgI+CbbdevWCQRLsO\/69evLCSecYEiX5bqpHKhz2lcq+y1yfemAFQFFICEI+CJbZhvce++9cvTRR5uXZMy1nT59uvD5mvvvv9\/MVEiIVh4aWbVpe7hUo2qRA4yHC+iOIqAIKAJpRsAX2TKPdsuWLSbuLCERQ6GQVKhQQW6++Wb5\/vvvZdOmTSkbzupNf4ZW1Pm1KYNcO1IEFIFCIOCLbJmNwEIGLFx3n4ReJN+dnqxjnfYVD1nNVwQUgSAh4ItsmUfboUMHufPOO4UZCPhvWbLL1xuaNGkiLHog7GKyB7i3RDlxviBLdn\/aviKgCCgChUXAF9niJhgxYoSsWrVKWKLbvn17Q7yQ7uOPPy6EWUQKq5Sf+jrtyw9aWlYRUATShYAvssVPy+fGp02bJrEk2YPZW+J\/X4vIQp9tsuHT9hUBRSANCPgiW\/TbvXu3vPDCC9K8eXNh6hdfUyAIjXP5LuWSKbgRbPuVyxS3u7pVBBQBRSCwCPgm2+eee07GjRsnPXr0kFGjRhnXwdChQ+WZZ55J2SCdZJuyTrUjRUARUAQKgYAvssVnO3nyZCFo+HnnnSe1a9eWK6+8UkaOHCmkk18IXTxX3VPu+HDZIM2xDSulO4qAIqAIuBA4wHUc85DwiqFQSMqUKZOnHMehUCilixpQQP21oKCiCCgCmYCAL7I97LDDDNFOnDhRrI+Wz+HwJVxCLxKCMRWDti\/INLRiKtDWPhQBRSARCPgi24MOOsisFuNrDaeffrowt5aXZHy1gdCLqVjUsGrTDkm4zzYRSGobioAioAjEQMAX2dJO9erVZcKECcKLstzcXBk2bJiZBkboRfJTKTrHNpVoa1+KgCJQGAR8ky2dEXgGciXMYoMGDQSLl\/RUCJZtKvrRPhQBRUARSCQCBSLbRCpQmLYaHqfRvgqDn9ZVBBSB1CGQcWSrcWxTd3FoT4qAIpA4BDKObJ1D16lfTjR0XxFQBIKMQErIds2aNebrDscdd5ycfPLJMnr0aGHKmBuY8ePHS9WqVcPCd83cUcRmLtscrqZkG4ZCdxQBRSDgCCSdbAm7yNcdIM4lS5YIgWyef\/55mT17dj5oIOXHHntMli9fbmTq1KlRv2+mRJsPPk1QBBSBACOQdLLdtm2b8IUHZi4UK1ZM+HYZMxm++OKLfLCsX79eypUrly\/dmaBBw51o6L4ioAhkCgK+ydbvwFjKSwzcihUrmqqbN28Wvl3G53RMwv4\/v\/76qxAXl8DkzOVlShnzed3uBjv1a9Y+y3jFihWSLYJVny1jcY5Dx5VZ12g2nS\/rktxPMWnfJJ1smZNbtmxZwapdu3at3HTTTcKy3rPPPjvP4Hft2iUlS5aUPn36yNdff22C3QwZMkQ+\/\/zzcDlLtCRccm5TycnJyRqpVKlS1ozFeV50XJl1jWbT+bLuSPgiCJJ0smWQxMB96qmn5MILL5QTTzxRnnjiCUO45FnBAh4+fLg0btzYEDNLgVkSPGfOHFskz\/aYsgfnOc70A\/ePT6aPx+qv47JIZMY2W89XatGP3FvSyRY3wLPPPmu+WfbWW28JS3wjxVBYvHixPPnkk7Jz5848mmIF50nYf\/DogDvDsxbs40ImbxlWJusfTXcd1\/9m10TDKEjp2Xi+GFMQJOlkiw\/ogw8+kAcffFB4RIk2aKKGQcZEECOiGJ\/dWbhwodStWzdc5ZiyxWXTg83kizvOkDUfv2hmLNhHBd3+OYNDcVAc9BrIfw2ESSSNO0kn2++\/\/16++uor4fHE+QtOwHHm0DIl7MMPP5TKlSubj0cOGDBAeEGG77Z3795So0aNfPBAuvkSNUERUAQUgcIhkNTaSSdbLNNFixbls0K7dOli5tAyl5bvmTFKZiBMmjTJlJ0xY4Y0a9aMZBVFQBFQBDIegaSTbcYjpANQBBQBRSABCCjZJgBEbUIRUATSi0Am9K5kmwlnSXVUBBSBjEdAyTbjT6EOQBFQBDIBgYwhW+brjh492kQNO+GEE4TgNr\/88ksmYBzW0esYmPr2wgsvmLESKa19+\/ZmKXO4oYDteB2XU20+GnrllVdKUM+hnzF98skn0qhRIzPvu1WrVvLll186hxqofa\/jIoDUoEGDhHuN2UGXXXZZcq7BFKBDbJYrrrhCmEqagu6idpExZMuy3VdeeUWYi\/vpp5\/KqlWrZNy4cVEHFsQMr2Pg5n311VfNWLlxmdHBj4t7wUdQxuh1XFZflm0z75qVhTYtaFuvY1q9erVZWv7QQw8JUe3OP\/98YbXknj17gjYko4\/XcTEdc968efLf\/\/5XFixYIBUqVJBHH33UtJEpfzBamLd\/1113memn6dY7Y8h2ypQpwpxc1t2XKlXKxMf96KOPhAA26QbRa\/9ex8C85LPOOsvESmC1XYsWLQSC4hfaa1+pLOd1XOgECbFS8Mwzz8y3ZJv8oIjXMbH4pk6dOsLS8gMPPFA6d+4sffv2NUvOgzIWpx5ex2XrcL7s\/pFHHml3M2ILNzC1lLgs3EfpVjojyJYTvnLlSqlWrVoYL078pk2bMoZs\/YyhY8eOwjxkO9ilS5cKN3IqP6xp+4639TMu2vr444+N64A4GRwHUfyMiSePHTt2yLnnniu4fC6\/\/HJzTYZCocANzc+4mPtevnx5qV+\/volnwlz5Sy+91DGm4O9ilBEe4Pbbbxdir6Rb44wgWy5miJUlvekGrKD9+xkDFwlj5TEI10n\/\/v2lU6dOcthhhxW0+6TV8zMuQmg+99xzxvr7v\/\/7v6TpVNiG\/YwJVwjBknAjEK2uYcOG5n0CPs\/C6pHo+n7GhYuOFZ7Tp083bgS+sDJw4ECBsBOtV1FpLyPIljCNJUqUEC5s54nB2guFgmdBOHW0+37HQEzYiy++WAji8\/TTTxvLybYVpK3XcfFihrFgAUZagp2JY7I6t2vXTmrWrClgwbL0H374QQiab\/ODskU\/L\/cR7wZwN\/D0QRxqfvg7dOgg3333nRCPOijjyTQ9MoJseXwmdgK+TAswFzSPOViBNi3IWz9jwD974403mpCU77zzjvBli6COzeu48J9x\/ggOT4yMNm3aCAGKsATT\/ZbYja3XMVGPF0dYjOxbOeCAAyQUCp4RsH8Ed1y0AAAIlUlEQVRceV4WRbqP8HEWL17cDie8hazJCyfoji8EDvBVOo2FecuLsxuLj19XpkZdcMEFwgWURrV8de11DK+99pqJC3HVVVcF9kWLc+BexoV1NGbMGBP3gqhUzCo555xzZObMmVK7dm1nc4HY9zImFCV+x+TJkwWfJq6D119\/XXhhFu\/zTtRNh3gZF0+MZ5xxhrz99tvmxSzj4tuBnKcg+D7TgVsi+swYsj3ttNMEBz0WERfCSSedJMxpTAQIqWoj1hh69eolCD6xZcuWCR++5IULViDCTAx8aKnS1U8\/Xsblp70glPU6Jspdf\/31cvXVV5t50Rs3bhSeSkKh4Fm24Iq+0e4jrj+Ecv\/4xz+El2S4ffDX4lq45ZZbAmmxo28mSMaQbSgUEiZWM+ePFxG33Xab8ZFlAshWx1Ao+hiGDh0qCFYFX6zA+nMKVv0RRxxhmwrUNhSKPy63wlhJfLEDi9edF4TjUMjbmEKhkPCExXxOZo088sgjSXvznQhcQiFv48JlcN1115mXY4yLedHRAvknQq9ktsF9w\/cMueaS2U+8tjOGbOMNRPMVAUVAEQgyAkq2QT47qpsioAhkDQJKtllzKnUggUVAFVME9iGgZLsPBP2vCCgCikCyEVCyTTbC2r4ioAgoAvsQULLdB4L+VwQiI6CpikDiEFCyTRyWGdfSyJEjzdxeL4r7KeulPVuGxQDEgiX4jp\/YtswHRSfbThC2hCVkTjQr\/pK1Ko52aR9hPwjjVh28IaBk6w0nLZUkBAhOc8wxxwhhF1M955Z4DQT7SeTQLrroIpk\/f37SVsUxV5RVdw0aNEik2tpWChBQsk0ByOnsgoAoPXr0EBtxn33SsMIGDx5sArBbq5KJ31iZWGcELCcINsF\/IpVdvHixEICFVW7c+NSFvCKNlQn\/rPajXWdZ2iWaGfls3ZYt7dEudejnvPPOy\/MVBIiar1iQR\/uEO6T\/3bt3mwDejIE+GRPt0B7WYOvWrYWxk45lDR7gwhcJ6Ovhhx82kcmsPkS+IsAMbdEP+tJPLKEuuGJ9W5wuueQSYbl5pHpuSx1sqE87kcprWuYhoGSbeefMl8asb+eGJeo+q+8IkkIUMZZisgoPS4yYBRDOqFGjhFCBrFxjn\/COrB5ylyWIec+ePeVf\/\/qX+TrBM888I8OGDZMZM2bk042vFxB85t\/\/\/rd8++234ixLu1i0EBxbt2VLe7RLHdphpRZh\/hgPHWHh3X333UaHv\/3tbwJJsqyUFYbvvvuu8LUL+uzTp49Aehs2bKCarFmzRggm\/fHHHwvLvhkzIR\/BiM\/1zJ07Nxy1C3Lu27ev0C9YMA7Ggz6msTh\/3njjDWH1FTo1adLEuG3AOk41zc5CBJRss\/CkuocE4fBoy1JglgTffPPN7iJC4BRCILJ2HsLCCoSYIz1mQ1IsgSSQDFGgatWqZb7BNXv27HztYhUef\/zxQnSvUCgklG3ZsqVAhvkKuxIoQ1nq0A9WLF9CYJ+irNsntCHH6P3bb7+ZMJyEcOQHpUqVKiaQN0tP9+7dK4yJekcffbQQY4N0CJhPxRBCELIn0ArfRmPslCWua+PGjc2XGOiHH4Zjjz3WBJ4hP578\/e9\/N1\/coC\/6RMeVK1fGq6b5WYiAkm0WnlTnkLBcu3XrZgJaY8XxKGsft53lIFhiMhCxCosTawxicJax+1hmfA7mxBNPNB855PH6+eefl++\/\/94WCW9xN9AvRG8TITXcE\/bYubX7WK\/fffedUNamQYQQH1Ypac48jq1s3bpVsHh5iQTBPffcc4aEbT662BCC6IwuhOu0+XwFpGTJkuaQsRJhDlcF44TcifJFPVMgzh\/n10Xok77jVNHsLEVAyTZLT6wdFo+vREnDSoRk8TniryS+rC3DlpCHPEZPnTpVsEaxgCE38tyClYb\/lEdpXA7IF198Yb695S6LdYllba1KthxXqFDBXTTPMcRUtmzZPF8G4PGfQC+QaZ7CroPHH3\/cpOAO4Dt1fBrFErTJcPw5\/PDDBSt2y5Yt4VT6gWRJYKxdu3YNh4ZE988++8y4UMiPJ8SLtWWIe8uTAhayTYu2ddaLVkbTMwsBJdvMOl++tYU8IU4sRSpjrRKtHwsLInE+XpMPGVD2pZdeMpYq+aQ7y2JdfvPNNyYWLeQJ2fIyB7KmrFMgetwLEBRleWTnuFmzZs5i+fbRDxfCf\/7zH\/NSiZiq+G7pC\/3zVXAl0BfhKol9jNUNQXPsKmYidB111FHy4osvCn0QuJ3ythw\/Ku+9955xG9Am4\/jnP\/9pvlpgy8Ta4oagTdoeMWKEVKpUybgV3HWIqIW1TB8QPS\/03GX0OLMRULLN7PMXV\/trrrnG+CpxDzAjgZdKd9xxhwm6DhFyDHnwtj0nJ0dIw43APl\/17d69uxBHl3RbFp\/nfffdJwMGDDAfOYRoO3fubPy2boXwpd51110mxiuP4jfddJPceuutQrq7rPsYKxyfJ64AYqpicWKlQsTuss5jQgPii8V9gS+WdrCweeMP6TnLYvH23fcCjFkClCfWKz5mW4YZC2BAvFr05+UYLgr8wrZMrC31OQe0TR\/0RZ9gSoxiZh1Q\/9prrxWeDvjAolsHd1nKq2QeAkq2mXfOfGmMxYT\/lTfpPAITcR\/rikZ48YSVyWwEHuttOaY2tW3bVvr16ydYxrwMc5bFV8qLokmTJpnHa8ozUyAUyh8wOxQKmSDUlMEqZYaBsyzETv+0iU5OwZqGOJlFgf7MWIDoKYO13qVLF3aNONuhDONkvOhIHtYq\/dSrV0+wGhkTFbEkcVmQT3n0Y2xYz\/QfCoXC8WrRn\/bIp64XYV4sdWgbndCNevQPtujGMd\/6Gj9+vHz66afm5eE999wj6Asu7rKUV8k8BJRsM++cqcYJRGDXrl3CFDjmFONCwe0AyTFNK5M+uZRASLSpJCHw\/wAAAP\/\/jR7XPwAAAAZJREFUAwBPeUoe3qzqNgAAAABJRU5ErkJggg==","height":209,"width":347}}
%---
