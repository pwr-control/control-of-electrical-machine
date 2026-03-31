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
% model = 'psm_svpwm_nonlinear_ctrl';
% model = 'psm_mpc_bemf_ctrl';
model = 'psm_mpc_ekf_bemf_ctrl';


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
%   data: {"dataType":"textualVariable","outputData":{"name":"tau_bez","value":"1.4559e+05"}}
%---
%[output:47b973aa]
%   data: {"dataType":"textualVariable","outputData":{"name":"vg_dclink","value":"789.7124"}}
%---
%[output:254225a9]
%   data: {"dataType":"textualVariable","outputData":{"name":"l1","value":"67.6682"}}
%---
%[output:9ba7162b]
%   data: {"dataType":"textualVariable","outputData":{"name":"l2","value":"0.2831"}}
%---
%[output:78531472]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.2831"],["67.6682"]]}}
%---
%[output:4a3235a3]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht","rows":2,"type":"double","value":[["0","0.0001"],["-9.8696","-0.0016"]]}}
%---
%[output:49427ef0]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.0156"],["2.7166"]]}}
%---
%[output:141fac74]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.0000","0.0002"],["-19.7392","0.9969"]]}}
%---
%[output:4590e09d]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.3110"],["54.3322"]]}}
%---
%[output:89a6cd8e]
%   data: {"dataType":"textualVariable","outputData":{"name":"kg","value":"0.0395"}}
%---
%[output:66f5352f]
%   data: {"dataType":"textualVariable","outputData":{"name":"kw","value":"1.2547"}}
%---
%[output:5677cc0c]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l1","value":"0.6069"}}
%---
%[output:4646c5b9]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l2","value":"344.9191"}}
%---
%[output:536fbf1f]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l3","value":"-418.6042"}}
%---
%[output:042f8e6d]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAP4AAACZCAYAAAAPZ4ezAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQmUFsURxxs8QEWRw3AJ3giGKAoGFOKFSkRBIIAgRkE8EIX4ooBXQkRF8AwIBPR5RaN4Cyo8UBOMKAoCQSSAB4gg3ooK3krer9f67J2d+ebYb2babM97+3a\/b3tmqqvr31VdXV1VbcuWLVuUuxwHHAeqFAeqOeBXqfF2nXUc0BxwwHeC4DhQBTnggF8FB9112XHAAd\/JgONAFeSAA34VHHTXZccBB3wnA44DVZADDvg\/DvoLL7ygTj75ZHXPPfeo9u3bl1QUvM8eN26c4rtbb71V1a1btyTv+vLLL9VFF12knzV27Fi1fv16NXDgQDV06FB10kknleQdQQ+Jyrv77rtP3XTTTer2229X++yzT1Ga4rSN0znh02OPPabOPvtsNXLkyEi3e\/m73XbbRbqv1I1ee+01NWLECHXNNdeE8rDYux3wSz0yEZ73\/wT8jz\/+WA0aNEj17ds3dIKJA+Y4bSOwvNAE4DAhdu3aNTLoudkW4EMLvJk2bVqlFEc54MsgLl26NBIvV69eHamdNBLNIJ+vvvpqLSzy3mbNmqm33npL8f4DDjigXMfMe83\/yfe\/+c1v1LPPPqsfXWwmh2kXX3xxgW5pa2qt5s2ba2EWzb9hw4ZyWpTPXELHq6++qq0F6Y9XA5qf165dW3h\/48aNK2g\/770iqKK5g3hoCiYa4bzzztN8NPnBhDN16tQKfecL+R807bLLLrqNWCSmluR76Sd\/Q8+FF15Y6IdXhrw8kpdjWdWrV0+DUPgp\/Pjoo480P822jIU5dgAXy8ZP8wrP5Lkyxl7a\/OTEe6\/Qz3uwqJDPDz74QNNsyqGXRyafZBK75JJL1JgxY9R1112nZasYz720mpaoVyZigfDHxhWAf\/nll6tRo0aFmqAQFsdMFYEVoZFB5PMxxxyjgQZDMQO5zFnZvLdbt26FAUAwBXQyiDATM87PnAyiAaZyiakvwBfgIGQCJiYn05Tmf4cffnhk4MuAB5n6Xg1qaj4BhB8PhS\/0w8\/UL7bcEB7CB+m7AN8UeJPfQoNpvUhboQEwyASKSW32Zddddy26NPFqfJEXk0YZCxP8Xo0u\/faCn3HwmvmmAvKO8bBhwzS9ixYtKiejMiEX69uMGTPKKZsgeROe8xu+Sf\/kfgG\/0OnXh6iTQDngf\/3112rlypWqRYsWqkaNGkWfQduwNuYDvOatDFCbNm0qgIqBNNvfcsst5cBsCoEJWNEKQevIYuajn8Y3tV6xWdY7oRTT+GHAF+2LRmECZiIWIE2YMKGcb8Dk4Z\/\/\/Gc1evToQtugNb6pSURjwV9zIjJ5z2RjTsImOLyToQDdz2KDsCD+m5rcnNRkHGWSQNuKFRI0wXvfIfR6QeUHmmJj7DX1vZOayLppMYhVIsA1LSUvHszPXgvSO5l5J6gk\/oYKGh9T5NBDD1VHHnmkqlOnTtQJpGg7YZo5cOaAXHvttWr48OGFGc4P+KaJapqAaQLf1ChZAl+EF5MQgWD97LV0sLZMHk6cOFE7fII0vteEpR3Al\/vMsTHXkAJ8MZuF935WEOPmXUrRXjSVF5TepQdtiwEfS868\/JZK3glBZE80NcsL0woxn1cZ4HuXYDzXC3zhgx8eTJ4L8L2gimK1RAVsBY3\/zDPPqAcffFDNnz9ftWrVSs\/2TAS1atWK+kzfdnE0Pg+Q9RQzvFcjmS\/watdiWt20FJjxzXvNCUTMXT\/gi1PINLd69uxZzoNeWY1vzvDmsqUYD8M0vhcQXosqTOP77Q4U0zzmRCOWxZNPPlnw6sNv05Lwgq6Y6VxMw5VC4\/uNsZj6MrGaGl\/+J5Ond8njNdXFqgviubn08tthEl6JryAJMAO9+ps2bVJLlixR06dPVwxYhw4dVL9+\/VTbtm19HSphLy\/FGl9mTD+zKEir+M3oMrBBz\/EDvneWNgdHnFTmc7FQhKa423mmo8h0YhXjYdga3wSEaD0xfxlfHJ7F1vhBloTJQ9qY2tRrHgMAMd+9mtf0+eDw9Vvje01\/oalUa\/xiY8zkZW6Xek19v+URy1h8BX7A9\/NZyHh4+egnP+JwTmLm8\/xI23nffPON9hCjeXHAJd1\/DvPqQ5Cfx5TvzXtNEy+Oxuc5pika9Bw\/4HOv11wWs9Tr0ZUdhiDgC81+pqpMVF4gyPdRvPoIm1hNsl995plnalDK+htrhv+ZkymTVRSvvukN93r1vfSZnm+Tf7zX3OHAsTp37lwlYBEfBQJuTuqyI+Pd9fGb4L1efdqEOcbCxthP4+MkNO\/D2SvygpN58eLFhYnV1OBxvPoia5ns4\/\/www\/q7bffVnPmzNHmP1f\/\/v1V7969Yzn2wqyBUjgrwt7h\/h+dA3HiDGTSYzmYdqBQ9B78\/FrG4XkpeldB4wN2ZpVZs2ZpjcDVq1cv7VxCG1SrVq3oe1etWqXNm+uvv77Cdl\/QHq+YNn7bM6XopHtGcQ6Y1oV3yRDFlPRaXY7f4RyoLM\/D31C8RTngf\/LJJ+qcc85Rb7zxhtbqJ554og4LrF69eqT3cD9OJsz1yZMnVwD+mjVr9IQyZMiQSM9zjRwHHAfS4UAFr\/4777yj2DfdeuutY73xu+++U1OmTFG77babeuqpp3yDgHAWLlu2TJ166qmBz2Ztx4+7HAeqMgfAID9pXRU0Pvu6hHuG7eGj3c028+bNU5j5WAlXXHGFL\/BpM378eO1g2X777bXm79y5c8GiAPDs57\/44otp9dc913HgZ8GBdu3aKeJb0gJ\/hQCe888\/X0fuha3tcOoQe8z17rvvak8\/+5nffvutjjbzC\/tduHChtiRat26tNm7cqJcFAwYM0J5cLlkr0uEmTZpYNUB4oIlq7NSpU2xrKO2O2EqbrXQxHjbThuJDQaZxUlRksYKp\/\/LLLyvCcaNcHTt2LAdY855iW1XS7u6771bbbLNNwRtss5Poq6++UiyDGjVqpGrWrBmFPZm1sZU2W+liYGymLQscRNrHjyPBmPF+Gh8fACGlBABxKOfzzz\/X7Vjvsydravw0Z7o4fTHb2iwottJmK10O+BEDeOKAxQv8mTNn6tu7dOmivf0cJMEXgMl\/7rnn6u9lizCLmS5OXxzwk3Kr7D4H\/GT8ywIHJdf4ybpadlcWHU5KnxPi+JxzPIvPs6xw4IAfcWycEEdklNHM8Sw+z6wAPqb5ggUL1HvvvaeOO+44bZ5LdpZkXSp+l9P4ybhqK8Bspcv2ZUgWOAjU+Oy5\/+Uvf1G\/\/vWvdUCNJPjDGXf00Ucnk9CQu7LocFLCnRDH55zjWXye5arxOZIL6DnRhYYXLz1JGTjBxf8qez7fjyUO+MkExVaA2UqX0\/gBXn088wTgEGCz1VZbFYAPw6Lm5Esiwg74Sbhmr\/fcAT\/ZeGaBA19TX\/bciZ7DrGf\/\/bLLLlNPP\/20jl4jYm\/bbbdN1qsid2XR4aREOyGOzznHs\/g8y9XU5+WfffaZzpby97\/\/XYfh4tgjrh4rIC0HnwN+MkGxFWC20uVM\/QgBPGTfYc1P7H5Y\/H4ysf3pLgf8ZBy0FWC20uWAHwB8knF8+umnasuWLb6SSKRd7dq1I5\/TjyrODvhROVW+na0As5UuB\/wA4HPkljxiVKbZfffd9aEUTP9169apvfbaqyB1JNUo5eWAn4ybtgLMVroc8AOAj3OP1FkkBpTEgWh\/kiEuX75cDR48WD333HOF\/yUT14p3OeAn46StALOVLgf8Itt5ftt25gEcmBenhFYUkXbAj8Klim1sBZitdDngBwCf8\/icouO8vWTIEY3\/8MMP62onr7zySurlpJPBIJ27nBDH56vjWXyevfXxV+raR15Sj9w1Vd04cpDqfdwR8R8S4Y7AkF2y6lx55ZWFdT7aHq8+qblw7lFc4KGHHorwiuhNnMaPziuzpa0As5UumzX+vQvfVefeu0IP76R+LVW\/gxsmE4qQu0JP57GVx5ZeWp58kz4H\/GRjbCvAbKXr5wL8GUMOVB333jmZUFQG+OTV40cutvgI6CG3XlgyziTUOuAn4ZoL2U3CNVsnJVPj5wJ81vKUKyJqT64dd9xR14EH+JUJ5mHX4IYbbtCpuM3qKw74SUTYAT8J12wF\/rjZa9S42W\/qLmUOfPbsic0n4y4gZevutNNOUw888ICO0e\/Ro0cSXhfueeKJJ9SNN96oT\/\/5Af8Pf\/iDTtPdsGE665skxCMo+D0aNGhQqUkvybvD7rGVNlvpElPfxvG8\/ul16vqny+pKPD6gqTp0\/73Dhj\/R\/33X+Oa23ffff68o\/AcYmRDY3+eQzk477ZTohVSZpXwQ+cLJre8HfB7MRFOs8Eail1fiJnY6SEzCOYUaNWpU4kmlv9VW2mylixGwlbazHn5XLXr7K1X9iw818P3KZJdCgnyBL9t5lMY+5JBDtHY+66yz1ObNmxU1uTmtl2SNj6NwwoQJerkgRTP8gE9efXLt26Tx8XWQiQiabEuvbSttttIFcGykja289tct0bjOBfi8mCq5d955p+rXr5\/+m4y4mG7s4WPqhxXP9M5KxAHcf\/\/9WmMeddRRWutzuTV+5edvW9erttIlpr5tdRLmvb5RdZtcBvyaK6erh\/\/cN1uN7yeKrPUBLwUwklzeGvLyDG+tdawBl1c\/HodtBZitdNkIfLQ9+\/fPvbFRD36tedeo+yeMyhb4QUUxJC8+4byVDdd1Gj8euIu1thVgttJlG\/AB\/b0L3yl487f+cJUGfpoKsELtvEGDBqmlS5cGyhmmOck4KrOdx8Md8B3wS8eB+E+yZVIC9Jj3\/OZqVremurTN12rkuadnB3xhH8dyo1bNjc\/y4DvcPn4ybtoixF7qbaUrb40PyDHp71nwTsG0F9Czd7\/h1f9oB3hmGj+Z2JXuLgf8ZLy0FWC20pUH8AF7GeA\/UUTniYaXEScmn9h8rixwENvUp8AlJbEru8b3E\/EsOpwMWvZGx+UhxFF5WJWBL8BGs897vQzsfhem\/cS+LcvF5GeBg9BDOlEHuRTtsuhwUjqrshA7nhXngBfkYsoH3QXYO+y1s+p3cCPfQzhZ4KAo8Nm\/J6U2hTSaN2+ug3nS0PTCoCw67IQ4KQfi3\/f\/NlmaAIcbaPIwkMvand+Y8x32qhN64i4LHISW0DrssMMUh3M+\/PBDRVmtq666SifoSOPKosNJ6f5\/E+KkfIhz38+RZ0HgXvdJ2Ro9yoVGb1qnpvbQB2n1Ys\/JAge+wCe09tJLL9WHaFq1alWgkaw7d9xxhyuhVbNmlPHPrI2tALORLgHvM6s+UJ98slEt\/0jpmhFRNHfQGh2znavj3nU04Ct7hj434AcF8AR9XyoJz6LDSWm1UYilL7bSljVdXm391sdfakDzE0dje2UEzc1VaoAHyWIWOPDV+GTcGTNmjGrRooXq1auXnhHJtf\/444+r+fPna42fxgm1LDrsgJ+UA\/HvKxXwBdA\/AbgM0FyVBbX0yjTP+btZ3e0K5roAPz4Hkt2RBQ4C1\/iE5xKaK7n1N2zYoPbZZx81btw41bRp02Q9Crkriw4nJbxUQpz0\/cXus5W2YnR5gVsG4p80NJ8ro6VNfglwZd3N5122U2qn6l+p\/fdspJo3Tie9VdKxzgIHodt5knMPrY+TL+6pvDidz6LDcegx29oKLmi0hTYvmL\/77lu1fO2H6pNvt1YbPvtOs7NUYPbT1Hxnams+B623beGZnzxmgYPARBxU0jnooINUt27dVOPGjVMFvHQ+iw474EfnQDGtXEoz20uRV0P7AboM4MmdrA74PgXyWM+zhz9nzhz16KOP6vRbffr0Uccdd5xOPZWW1nfAjw7KuNaICWLRvGma18XA\/BOQy9bSXKYZnowL8e5ywA+qjGnwkUM7ZMwhMQfpssjCkyQDT9jQOOCHcajs\/14Qr\/ngc7019fmWmgWTWtqU2rQ2KSymmWm3y3Zb1Hbfb1Jt9m1qXdYiB\/wiwBfNT7LN6dOnq\/fff1\/17t1b58Or7LHcvNY20aBVsVVaghKmiU2gpwli0cKm9jVNbPne1NZhvEyLZ2HvjfJ\/m2nLQgEGBvBMnTpVTZs2TSfVZEvvhBNOUE2aNCl5aWxzkLLocBSh8GsTJihmVJdsMZmmdJYA9oLY\/EyQSRIQJ+FbGM+SPLNU99hMWxY48AX+559\/rmbOnKkOPfTQ1MFuK\/CLeagl0gva09bCxUAsHmzSmsnWFIlAK+P0KhWweI7N4LKZttyAH3fwWS0wUZCNl7z7e+65py7GgYVgXkT+mRl+vEd8s+iw0GNGdHHYQjRymkA2ASnOrMqa0zYDzGZw2UxbFjgI3cePMgmsW7dOp9weNWqUqlevnpo9e7Zavny5uuCCC8rdvmbNGjVr1iw1ZMgQ38dKh0tdUIM95BfXbop8mipKnxvvtLVu1mjHst8CaokAK\/\/ddqlqYYTYxuIQttIlk6WNPIOmRYsWqeHDh9ufgYdCGxBMZB8XabTJze8tiLFkyRK1bNmywEIZAnyeUdmCGoD9sRWbdHECfqJcgFmALH8LwOvV+F4XFNm3SV21xy47RHlcZm1sLQ5hK10MjK20UZuS3TOuzFNv4c0nYq9WrVrlnHlB34uEC3CJ8Z80aZLaY489ygk\/x3rHjx+vMPnZFkTzd+7cufAOub8yBTUw4ak\/9siysjTF3suMyZbTVIBbAB6EVhsLMAitttJmK13wzVbaUKDsoIGTzIAPsKmIu3HjRgX4MDdq165dwAJn8il2yQGeoIQcrPfnzp2rqI93xRVXlNv2W7hwoT7w07p1a\/0OsvUOGDBAV83hqszaBsCbKYqFaMl2UtkjkzavCW2lzVa6xNS3raCGV4FmBnwCdQDj6tWr1fr163V9O5x15tW1a1dtqgNguTDhCfAZPHiw\/ipq\/v27775bF+iQajpJgQ\/ozRTF0ADgR3beQ2c9KcXlhDg+Fx3P4vOssgow6hsD9\/FxwhGii7kfdq1YsUJNmTJFjR49Wu\/7o\/EXLFignXsyQRD2iwOwbdu26phjjlFsGXL6j0kE737SDgP61lfOL5BYlvWkoQZ9KS8nxPG56XgWn2dJcRD3TeWAL2t41t8A0y+ojzh9zP\/q1asX3iXbeazr+Zs1PhV1qZPHNh9Xly5dCpbAqlWr9IRAPT6+l9j\/uBrfC3oSJTx27oFxeRCpvRPiSGwq18jxLD7PcgG+FNI45ZRT9Foec997Yf6j2fOO1fea9yM7715yLW\/23QlxfCF2PIvPs1yAn4zM0t0VR+N3nbSkUIXELEZQOmrKP8kJcXzOOp7F51muwBfvflRTP1n3Kt4VFfgUJ6CyKBdr+v9cdkipSAh8jhPi+Cx2PIvPs1yBL95909Tnb7b6jjjiCJ1+Ky9T3zTx\/aqQJGN1+F1OiMN55G3heBafZ7kC349ctP9LL71UiLwzt\/OSdS+Zxic4Z9zsN\/XNWZj4QqUT4vij7HgWn2fWAR+CiHaaMGGCzrefRkWdMFPf9OJnZeI74CcTXu5ywE\/GuzAcJHtq+bt89\/GD1viLFy\/Wh2zYf4+yvx+XwLAOm2t7KouWKjgnCp1OiKNwqXwbx7P4PMtV4\/ut8SGIrbxzzjlH7bfffsl6FHJXMeB71\/ZZOPRMcp0Qxx9yx7P4PMsV+MnIrfxdxYA\/7\/WNOiw367W9M\/WTj6sDfjLehVm+yZ4awdSnCVl28d5zPJZDNDNmzFCvvfaaOv3001Px6IfNdOa+\/YwhB1a6Pllc5jkhjssxt8aPz7GyO3IDPmeVic775S9\/qXr06KFP2BHCS8FMfo8YMaLcIZ2kHfTeF9Rh06mXZlhusX444McfZcez+DzLFfhBxTGjnrpL1t3gmc4089MOzQ2i3Qlx\/FF1PIvPs1yBz7Ydx3NPPPFE1bFjxwL1S5cuVXfddVfmZbLzNvNhgBPi+ELseBafZ7kCn5e\/+uqritx3mPkcpV25cqUit97YsWNVu3btkvUo5C4\/U98GM98BP9lwO+An41tua3whl3LZnLVnXb\/DDjvonHpp7N\/L+\/w6bIOZ74CfTIAd8JPxLTfgk2IL596f\/vQnfaY+q8uvw2aIbh7efOm7E+L4UuB4Fp9nuZr6xOVTLHPz5s2qe\/fuqWp5kzV+wJf1fdYhut4hc0IcX4gdz+LzLFfgewtfmOR7i2DE6ZrsCkgGnjPOOEOX4ZYDP17g27K+d6Z+nFH+qa0DfjK+5WbqJyM3\/K6JEyfqsF92C8iySxbeoUOHFtJwewtqvPlVLdXz5mX6wRd02lVd0Klp+EtSaoEQ21iAQSYlG2lzPIsvjLkU1Eiacy9q94j8a9CggU7ISQeJDBw5cqRq2LAsE663oMaX+3ZTNy8oy49\/c8+Gqk2TmlFfVfJ2thZgoKO20mYrXTbzLJeCGlnk3CNG4KKLLlKPPfaY\/k0IsNfUl4Ia50z\/QJe+Yn3\/woXpJNGMOkPYWoAB+m2lzVa6bOZZLgU1ooKgFO0oR3XppZeq\/v37q\/bt25fT+FJIgLTZrPPzduy5NX6yEXdr\/GR8y3WNX+pDOph9kydP1rsEUlqrWEGNxs1bF\/Ll5xWmaw6bE+L4Qux4Fp9n5pI3s0o6QmYah3T8CmpceeWVauDAgaply5YVNP539VsUjuFmnXTDb7icEMcXYsez+DzLFfhpHdJhOw+w4+TjKlZQ45lPGxRy6+UZuCND54Q4vhA7nsXnWa7At+GQzlWLahTy5n98w5HJOFjCu5wQx2em41l8nuUKfF6e9yEdAb4Njj3n3EsmwA74yfiWq3MPkvM6pDNu0m3q7Ce3aK7llXjDO2ROiOMLseNZfJ7lrvGTkVy5u2SmO\/eKyQqNz2WDR99p\/GTj6oCfjG+5a\/xkZCe\/yw\/4Njj2HPCTjakDfjK+VVngH3DaGIVXn8sBP1x4bAWYrXTZPpFXWeBv6jhCfVd\/Xy3xNnj0bRcUWwFmK122j2euwGfPnXh6CmWaV+3atVWfPn1SOaMvHRbg2+LRt11QbAWYrXTZPp65AX\/Tpk36AE2jRo10ss2tttqqgP0aNWqo\/fffX\/G71Jd0eGP3W\/WjbfHo2y4otgLMVrpsH8\/cgE\/k3qhRo3Sm3axTb\/U9Y5j67NhxVnn0bRcUWwFmK122j2duwCeufvz48eroo49WZNzJ6qLDfYZdrjD1uWzZyrNdUGwFmK102T6euQEfU5+qOffdd5867LDDVP369QvYT3uN3+uiSeqLg063yqNvu6DYCjBb6bJ9PHMDPqfzXn75ZZ3ZxXulvcbvOXqa+qrFiQ74McwsWwFmK10O+EpV20JKXUsuZrru1z2lvmnWQVNky1ae7YJiK8Bspcv28cxN48MYTujde++96sEHH9Qe\/Msuu0wtX75c9e7dW1fXSeOiw90m\/0fv4du0lWe7oNgKMFvpsn08cwM+zr1JkybpPfyePXuqqVOnqksuuUQ98sgjekKgtJbkySvlBECHT7hjnfph+\/pWbeXZLii2AsxWumwfz9yAT9JNUl9ffPHFeg\/\/8ssv19t733\/\/vbr66qt1hZ06deqUwzwFNceMGaOowtOkSRNdWHPPPfcs18abr9+bo58Od7n\/S32PTXv4tguKrQCzlS7bxzM34JMIE9P+j3\/8o06FLcBH299www16EuB7uaitR+JMMursu+++6plnnlFz5szRcQBmoM+aNWvUrFmz1JAhQ3wNhedffl1rfC6btvJsFxRbAWYrXbaPZ27AhzEPP\/ywXuNT\/OKJJ55Qv\/vd73QI7wknnKB69eqlqlWrVgDvW2+9pR544AENaNb\/JOrEYsACqFu3bqHdkiVL1LJly9Spp57qC3yzQGbeBTS8BCLENhatECG2kTbHs\/gL4VwKaphk4uwnC8\/999+vFi9erCvlAv6DDz5YVa9ePbBH+Af+8Y9\/qDfeeENbDdtuu22h7bx583RgECb\/9ttvryeKzp07F55nAn\/Inu+pQcenU447\/nAoPZkxGeLzYClj02UrbbbSxdjZSlsuBTW8wgz4MeMBMwAOK5GN5ieZJmG+F154YQU\/wMKFC7VTsHXr1rqEFkuBAQMGqDZt2uhXm5VxRx6wWbXbrZY1+Fq\/fr2ml+WM0GsLcbbSZitdjJuttL333nvqpZdeUg899JDKPL02jFm9erV27q1cuVI1a9ZMAep27drp9b1X4zFBPP7441rTUxILp10xq0AA482rj8YfdtuzuohGrXnXqupffGgLthwdjgOZcgCsUVGKWpNpXL4BPDjx8Nx36tSpYIqj9dnTx3uPx9502rEuGT16tHbwBZnBfnn1cRqy3jfPAzAT8+Mux4GqzAEAnxbo4asv8IPy6gd9jwl\/1llnactAND1EMxnMnz9fj1+XLl2Ut0y2N69+VR5o13fHgSw54At8tPMtt9yi9ttvP31IRzz4OOeee+45dcEFF6QSwJNlx927HAeqMgd8gU8AD2v1Z599Vu2+++6qZs2aOtU263w0OZ+5iORzl+OA48DPjwO+wC92Os\/sItl53OU44Djw8+NA6Ok8zuaj7dnKM\/fkf35ddRQ7DjgOCAcCgY8jDq\/7zJkzdVv23\/v166fOP\/\/8Cvvzjp2OA44DPy8O+AIfDU+4bYsWLXR4LqBni48oPiKeRowYUXLnHg7FO++8UwctENXHdiJRgmZocNqslXgEogvZrjzzzDNVt27dKsQkrFu3Tu9YrF27Vm2zzTZ6exNa07yi0iY0sMUKDxkroi7TvKLShvU4YcIENXfuXLXDDjvoE59p8i0qXcSsoOTef\/999Ytf\/ELHqngPmKXJP++zV6xYoZ566ik1dOjQ1F4buJ3nl2xTtuNgkhmDXwrqONjDD8LACb+rrrpKOxjT3Mv0Yzi7GfSPCQdA\/\/73vy8XZ8CkOHbsWHX44Yfrn1WrVukwZOj1nlgsBV\/kGQhDGG3SFh\/NuHHjFNusHKpKG\/hRaSMclQmeaE1Kpd966606GjIsIjQpH6PQxXhy4rR79+56nBcsWKBDs5GBNDJJF+sLx+BRrrz\/iCOO0PINIZmLAAAIXklEQVSf1uULfGZmhB6Nx2k7uRgshBxrwDydVwriENQOHTrodN7M1AwGW4lZOhDJMci7+\/btq7s0bdo0\/Vs+8zcnFydOnKgGDx6sJz9CmjmTcN5556UKsCi0QR\/0E+7JARmiLk877bRU6eKdUWhDppgcOZ\/RtGlT9cMPPyi+A\/RRojyTyFgUugA+YeYEqzGRiwIivXzWPi1k67\/\/\/a9OeIPyyxz4MJk9e8DPabwDDzxQvf766\/q0HgPHQZVSXiwjMJ379++vWrVqVQAdWvekk04q5auKPovJB9PzqKOO0u0kbiFoAADZ7Nmz9Qx9\/fXXqx133DE1WqPSxsEqtlkJqIKmLIAfhTZJ2Y4Fh99ot912U8OHD1e\/+tWvcucZYDv99NMLpv5tt92mY1jyujiWywSUC\/DpNOv5p59+Wn300Ud6Zsb82HvvvUu+7gb4zLoIafPmzXMFvlgdYcBn2YPmh0eYhWmf2DMtoiDa0KBYZJjSWCNo2KyAH8Y3cjGcffbZ+vwHcsQEheVIvxo2bJgKxqLwDIsNmrBuMfWZBP72t79pRZTm0q1Yh3MHfiqj4fNQr2nPZwYN8ytN54+XFK9pz2fWeT169CjX9MUXX9RCSwoyliNppCFLQhtLsYEDB6oNGzaUuz3NU15+SyI\/vnnDvZns056YoownPGMCR\/FgsW3evFnTBR\/T9o0E4avKAB8GYNpwHBEwsb4B+Jg6aWkDP6bjDMIBxWEjLgSAQ0QtW7YsNBf\/B1pVliVZTJBRaDPpyAJY8r4otOHUY+nRvn17vZbGAsDDj3MvLc0ahS6iVBlv5A5\/Fs5a8k0y9mku3ZzG\/5EDsp2HQyaLrR4\/xsv2z5QpU7STzDxEJPEMTAJnnHGGDlsW5w+OTvwhe+yxR2pzQBTaOAglV5bAj0obyyM0K1qWuozEhKS5xo9KF1mhrrvuusIan1wSadIVJiRVSuOHMcP933HAcaB0HAgN2S3dq9yTHAccB2zhgAO+LSPh6HAcyJADDvgZMtu9ynHAFg444NsyEo4Ox4EMOeCAX0JmsyPBFRZtiFebA0lsI5WqDiG5EImq5FBR1IivLLzHUdkrRSS81ZWi3h\/WTp7ftWtXfdaiVHwPe6+t\/3fAL+HI5Al83v3mm29GBj3drizw2S7jpxSx9tBCSHiaoExjwi2h+GT6KAf8mOxG0P\/9739rAaVoCAVBCELh75NPPlk\/jQNGffr00THpRPixf03gCnv9O++8sxo0aJDOVizahwM111xzjY755zgo4aJ+EYvEOsyYMUMHwhDkRDjusGHD1CuvvFJ4N2GxXo1PkMpNN92k05\/zfCwNDj8BNp6H9rvrrrvUIYccoukl\/Jh3EcxECDD0ESPAyUlqJhBcxW\/OBPCstm3b6lNlfE\/CVXhCBBx0wC\/CvuEJ2ZOFZu+JPBP48JAAGsK3mQyIRCTAhmAqb5Skd8IoNoE44P8k7A74MYEP4AAwQg1AEEyAxek8U+NzZh9hBxgEq9AOwec+UwDRlgCdgBFyH\/A\/hJ5AF\/IdmheHhngHEw1BQ4SakruAk4IAz2+ZAYABJLH7HERhguIzP1gI0MekQ5ETDqdwpJe+UO6MyUBSqf\/1r3\/V9LCM4V76Ap3QQdQlE4gUVqV8WoMGDXRfeQ730pbJwqTZzLXgBT6n4yjYyvu\/+OILfQKSI9JE\/ZmXA35MAf6xuQN+TL6R4EK0D6f4zLWiCXwAh8AS9onG\/Oc\/\/6k1sxf4PA8tyyRRv379wpFkLAQ5JQiJPA9AIfjyPQddAC2FFyhS6gd8rA00NZMFx2F5DvdxSo4DKRQxJWEHmtQ0\/TmuKinXCFNmEmDyEODLKUY5Z2HSa55qxDrhXeL34N1ERjLZmVrfC3xo5h6ey0X9RuGfA35MofVp7oCfgIdkG0Z4MWGJ75bjpSbwJdsMbTB\/GzdurLWjF\/hoTr+DNUwEppOQEFy0IOnPBAzvvPOO1qSA6Mknn\/QFPhYEEwsA9CZP8a7xzc9mDQSWB0xwvFeAzwTEZz+6zOdgHUydOrUcl\/0ceH6mvnmyMMgf4TR+AgEOKqiR7FFV4y5MfUBBWjK0HWmk0LYczX300UcL4ENDLVq0SGtbtKmpBU1TH41Plhwxm3kAkwbnAMxEEGhftPaxxx5b0PhowJtvvlkDm\/f5aXyWHNxHG5YcmPJoeUCLqW+e+zbBhUmOtcK6GpOck26SHwEwC\/CFLvI2SNIUrBuy\/zDJMeHAq+OPP17TZ1pCxUx9rBDyM5ALgotJFb6zDDEvlhL4LrCGmJzcGj8aDp3Gj8anQitZu3OGmzU+wOeHNSjOLsngw9+cNgR0+AAANsDjPpKa3H777Xp9zRrfLFdGHj80OBOG91gozkJAy\/85JITjDUsCcAbtKAB02u+\/\/\/7ah8AEBFAALyfRgoAPYLFQ8AtAEyCmWjLZiEzgwxjKp\/\/rX\/8qrMfpI2DnHq9fgiPCHNFluWQ66vzW+PXq1dPFWyiwyjO5x1uwlFN+0ArwmaiYRMmR4Lc74Jx7Pwm7A35M4ANs8dZjaqM5ccRhDqN9yHzD6a7f\/va3et0+ffp0deSRR6pTTjlFtyPhA4koWB5wChEBJdcaEwPZfPbaay9tPfglGjU97aLd8eqzVi62lShmO8\/HAw+IcCYWM\/UBD5MPbdC8aHOAPHnyZO2vEI0PHZj7fI\/1gSee\/AW8k\/ak2MIaoZ9YS0xSJJH0pm7zAz5LJIqxYgHxLPLiMVnQ1+eff14\/k2SnkqSVHRP4zkTFpMoyimSj+EGYRB3wHfBjwt01L8YBgIkHXhydTAxMgmjrqJcf8E1\/RtTnFGvngO+AXwo5cs\/4kQMSPMT5enYysGyIIPRuvRVjmAN+tuL0P8DM8I7eg5KGAAAAAElFTkSuQmCC","height":153,"width":254}}
%---
