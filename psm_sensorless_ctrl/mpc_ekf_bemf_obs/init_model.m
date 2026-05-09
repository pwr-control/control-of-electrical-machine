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

b = tau_bez/omega_m_bez;
external_motor_inertia = 5*Jm;
% external_motor_inertia = 1;

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
fPWM_AFE = 6*2.5e3; % in case of mpc controller run 6 times the maximum pwm
% fPWM_AFE = 2*2.5e3; % in case of sv controller run 2 times the maximum pwm
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
%   data: {"dataType":"textualVariable","outputData":{"name":"l1","value":"  24.984105426951174"}}
%---
%[output:9ba7162b]
%   data: {"dataType":"textualVariable","outputData":{"name":"l2","value":"   0.101088737746315"}}
%---
%[output:78531472]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.101088737746315"],["24.984105426951174"]]}}
%---
%[output:4a3235a3]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:49427ef0]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:141fac74]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000","0.000066666666667"],["-6.579736267392907","0.998952802448803"]]}}
%---
%[output:4590e09d]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.103672557568463"],["18.110724075998974"]]}}
%---
%[output:89a6cd8e]
%   data: {"dataType":"textualVariable","outputData":{"name":"kg","value":"   0.013273093780640"}}
%---
%[output:66f5352f]
%   data: {"dataType":"textualVariable","outputData":{"name":"kw","value":"   0.423833817530528"}}
%---
%[output:5677cc0c]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l1","value":"   0.237172904101351"}}
%---
%[output:4646c5b9]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l2","value":"     1.436150968933706e+02"}}
%---
%[output:536fbf1f]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l3","value":"    -1.779725838133451e+02"}}
%---
%[output:042f8e6d]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAVsAAADRCAYAAABikxHnAAAQAElEQVR4AeydCbxN5frHn039EyVEKZQjFKXpZihTbqJSoVs33KbbgCgV6nRp0GDWJEWlWTRKuUUTmbokCUUhZGwwRWVMf983726ddfaw1jl7WHufx8dz1lrv+Ly\/tdZvP+tZ7\/usYn\/oP0VAEVAEFIGkI1BM9J8ioAgoAopA0hFQsk06xNqBIqAIZDQCCVJeyTZBQGozioAioAjEQkDJNhY6mqcIKAKKQIIQULJNEJDajCKgCAQNgWDpo2QbrPPhSZsPP\/xQqlWrJiNGjPBU3m+hBQsWyEknnSQ9e\/b0W9V3+V9++UU6dOggTZs2lR9\/\/NF3\/URUAEfGy7gT0V6q2kBf9EbY99pvEDB36orujIHr2pmebftKttl2RpMwHkgQMoQUuVGT0EW4yVT2Rafc6MOHD5cHHnhA6tSpQ1JU8asb5JGsH0XOQ\/\/+\/aVMmTIybtw4Ofzww6PqHfQMcD\/77LOlR48ewvkIur4F1U\/J1oUc1hw3iBVuGIpwcUM2CBe3zccqIt+Ksz4ExQ1KHu1Qh5uaNthnS7vkRxLaphzCL3+0C5E+6eull14yFi990R7lqUd9hPac6bZ\/dGCfNqy+lEM4vuSSS2TVqlUyc+ZM6dixo1CePCv052yffHd76EgZK9Sx9e32p59+kkh9oQO62bq0Zeuw5Zg8xgq+7Nuxkk9fpFnhmHTk+eefN4R14okncmjGhu62LP3SP+JHN\/oAKxodNGhQ+CmEdNs2W44pE0nok\/4ph7BPGvjSNueD89KmTRv54Ycf8jVBWepQFwEnd6EPPvjAXDPkOzGjDycO7JNGfXSmPNcb7dt2nddbp06d8j2xOPOp7+zvyiuvpGnhfIiI2c+2P0q2jjPKyR87dqwjRQy5cHHZRC7w7t2720PhRrL5XHTO+twIN998s7mBbYXHHnvMkBbHtDVq1Ch28wlt0rbN2Lp1q9xwww1RH7Xp684777TFTTnKU88m0h7t2uNEbSGqKlWqyNKlS02Tv\/32m6xZs0YqVaokJUuWNETjxIVCkIUXXX799VcBQ8ZHPYS2OFfss+WYfcYKvuxboQ\/6ssdsrQUFGc2ePTusJ3l9+vQJnx+O6Zf+GRPHTomnm7Ms+7F0Id8pEBv90r9NZx\/CX79+vU2Kuo1UH5zAy1aiPec1g4UPIZLvxoFrlTTyrFCXNjgGS+f1BolThzyEdi+77DLhHHGMOK9HLHOsdM4HbZGfbaJku++McoLHjBkjBx98sLz11luybNkyue2220zuxIkTzZY\/0fKpz4XSoEEDmT9\/vhH2ueAQ6iKkkf\/kk09yKFOnTs1DxiRyozzzzDPsCuXQ5aKLLjLWJXVNRoQ\/6EvZ5s2bCzcWNwL1SKMdqjjHwnE8Oeyww+S1114TyBTdaeeggw7KU40ydevWFcYPDuhI302aNBFIyguuNFihQoV8fYVCIUPc9E27YIku4IY1x5a66EU+5Ti2wnjd54wbHguK+ps3bxb0ZEzgvnbtWjNW+rHtffXVV4Yk3DiEQtF1Qw90Qg\/OS+fOnSWWLpRzCv0jtIMeCPvgyo8abXMMFhAbj+Kx6nNNgwN48SNhy9KObRtcwCQaDvb82rr0T90hQ4aY6x3dbBrt2nJsZ8yYYTAk3Xk9cp3THz\/K\/DhzPtCBOtkmSrb7zignmBN9\/PHHS05OjkmFqLiYucisNeHMb9iwoSFnbtAVK1YI9blBsPQQ9k1Djj\/2xuaXnIvfkRXehaCwDOmbdsjgguYihUg5dgttoY9N54Zk\/5xzzmEj1KM+7ZiEBP+pXr26GT840rfVh2NwceIWCddo6kDkU6ZMkdGjRwtPAdzM3NSU37ZtmyFiixOEefXVV5NlhJuYcwOJtG7d2jwuY02ZzL1\/0I28vbvmP\/WPPPJI86NGP2BPe\/PmzYvoz42lm2nQ8SeeLo6iZhcM2bHXC7qhC2mQNttY4q4PGTMOcCxVqpSp6sSNcZvEvX\/oi3LgDpmCQ6xreW+V8FON1Rf8EPIQqw9PGbgQ2JJelETJNsFnGyKB1JwC0SW4m8A1Z4n+\/fffN9a6k1wLoywkhb+QGxRC56aHJPy0SXnqOc8JPzqWAJxtkQ7BWKKAFPBLYrE7y7FfEN2i6UJ7QRJcYmAORk48CqMjP8BY2M7zAKlD7tbAwJWAIVKYfoJaV8l235nhBHOieWRcvny5SbWP4jwily9f3qRx0yIc8CiKZYRVULVqVfOiBSs40o1Jea\/ifqRy3tT4\/by0AzFRjpuFLb46bh5uIo4RrGcucsbLuEkrqPA0AMHOmTPHWJvWwvGKa7R+wRrhUZwfLW58a9naOhyTDk48ltp0bmLOjc236XZrMbLHnDeIFYK1j7v0G60+eiGUiaabbTueLrac3VrdeOxnXIgdm31asWUjbW19e\/65bjj\/\/HA53QiR6uJfxTWB4dB5r\/vDz\/Vh9QUXxLaPPtwruBNsmnPLUwZPQNxrPDE487JlX8l235nkBLdv3974ldyPnO6Lm5uRCxcypjr51OdC4cbEKiIf4eblJqacV+HGhKy4ONHFPsbRLuKlHSxNLAkem9GDLce89bUEaHWlD\/qK1y43D2PnxneXtTpThpuG\/ikDLl5xpbwV2qGv0qVLG1cN+jMO0mwZfgBpm2PSLU4cW+HcsE8+9a1APuAAJuQj6Mo5pG\/aoiz9UoaylEHIp71YulHOCm3wYxdLF1vWbjnPCH2hC8I+ljH7tly0LWUoyzXKONCXsrgirBuB47A4dhgrhoet6+X6gJjpDx3p2\/Znm+V6AEewQB8r4EIZDBf65IeL42wUJVvHWeVXnIvGkWReUGG12DQuqPvuu88empdoNp9HUGd9Lq5hw4YJN3G4gscdty70+\/DDDwuk5qUJfHRMg3KW5Zh09OFNss3r2rWrcGPbY\/fWWtrudPexvaGwcLF0bb57LKRjOVrcOLbi7qt27drCHEzywZPHTnTFEsfi4g03x+SDkfPckEYf9MW+FW5o0i2pWGuMfN642\/Y4RixufnWjfXSmDYQ+o+lCvlM4z5xvxmTT2eclHefPpkXbUoZrz9m\/HXe0OjaduvZHjDR05rrmx5knCNLcQp1Y1xTXHT53pz60ybVhLWnq04677Ww5TinZYj1dccUVglUhEf6NGzfOvMSwv3oFsQojNOsrCcJ0+pS4QdwNcPPbMlwsznxnfV5IcJGRTzvUseVJJx\/y4MaijFucbfGywl6I7rYoR1u06WzDlqNfhGObzz5pCFOh0MP2QTu0R7uURz\/yKcuWY9LdYutFKkNb1LdC\/9SnLcrbvu0x5Ujn2NZFJ4iQdPYhdCworCkIgTbmzp1Ls8Jjq9nZ+4e+aM+KPQfgCalYd8reoubHjPZtWbbUd+dRJpZuYIGgJ23YPmmLYys2nfbdgn6MyZZlnzTK0Tc6ONNId4qzf9qwfUWqazFGP9qgLHUQ0pz5HJNOGcoi3NOcC84PhNyiRQvhB5EZBvxIUcatD20602mX42yVlJHtH3\/8IUwBiuazAeDVq1cL8yQ5kUisC4nyKkUbAUgDdwsocKPzI82jL9YTViXp8QRrHLcH9eKVDWZ+MLTCdYDlzQ8f+9b1gM+c8xQMLdOrRcrIll+5WbNmSePGjaOOeN26dYIfLmqBNGZwwcSzJNKoXpHtGuuKH2anYE1iRXkBhXKUpx0v5bVMZASwuDGOnOeBfWu9Rq5VtFJTQra8UMF\/hPURjUx5Q8qcSPw2NWrUMD7E8ePHCxZx0TolOlpFQBHIRgRSQra4D3jEqFevXlQMd+7cKbwl7d27t3z99dcycOBAGTx4sHz++ef56vC4qFItj39b8cg4PPT8VUvNOeNFaj4SSUNC0sl28eLFMm3aNBNjIBQKRR1i2bJlBesXN0Px4sXNEsrTTjvNLAGNVIlHlGwSxphN47Fj0XEtM0u\/LR5B32bj+TrrrLMYVtol6WS7cuVKmT59utSvX9\/8kvMiAneCc3I9KCxatMhMs9qxYweHYeFlR\/hAdxQBRUARyFAEkk62TOdw\/pozt45pOm7HOS+gWMrH28zff\/\/dLPlk\/h2TzDMUW19qf\/TRR77KZ0rhbBwX2Ou4QEHFDwJJJ9tYyrCyirm0zNHDp8vLMQIi84IM322vXr2kZs2asZrQPEVAEVAEMgKBlJMtFi3WLujY6SL2mAnRRDTCEsb10KxZM4qpKAKKgCKQ8QiknGwzHjEdQOYioJorAmlEQMk2jeBr14qAIlB0EFCyLTrnWkeqCCgCaURAyTaN4GvXXhHQcopA5iOgZJv551BHoAgoAhmAgCey\/emnn6Rdu3ZmVRdRluJJBoxbVVQEFAFFIKUIeCJbgsGwsqtLly6Sm5sbUyiT0hFoZ0FGQHVTBBSBfQh4IttQKCQsOmCNcatWrSSWUGZf27pRBBQBRUAR2IeAJ7I94IADhAUG++23375q0TcVKlSInqk5ioAioAgUUQQ8kS0uhKeeekoIkWg\/a7Nr164iClnRGLaOUhFQBBKLgCeyxVolkPfbb79t3Andu3eXv\/3tb3LvvfcKsSLx6SZWLW1NEVAEFIHsQsAT2TJkYszWqlVL+vbtK3PmzJGnn35amKVw7rnnCrEN3nzzTVFrF6RUFAFFQBHIj4BnsnVW3X\/\/\/YXQh0OHDpUJEyYI33vns8ubNm1yFtP9dCCgfSoCikAgESgQ2WLREpOWrypg2ebk5Ahke+ihhwZykKqUIqAIKALpRsAz2WK1vvTSS9KiRQtp2LChsWiJN\/vZZ58JYRNPOeUUwdWQ7gFp\/4qAIqAIBBEBT2RLkO82bdrIo48+KmwnT54s+GjPO+884QsLQRxY5umkGisCikA2I+CJbPkOGFO\/ZsyYIawQq1SpUjZjomNTBBQBRSDhCHgi261bt0qfPn1kw4YNcRXACo5bSAsoAoqAIlDEEPBEtmCyefNmGTBggOCnjSWUoXwREh2qIqAIBBSB6Us3y5YWA+Xk+\/8nY2Z\/n1YtPZEtboRbb71ViHvAy7FYQpm0jkg7VwQUAUVgHwKrNm2XPSXLy8qN26VK2RL7UtOz8US2Bx54oImNECsAjTMvPUPRXhUBRUARCC4Cnsg2uOoXTjOtrQgoAopAqhBQsk0V0tqPIqAIpBwB3Acp7zRKh0q2UYDRZEVAEVAEEolAZpJtIhHQthQBRaBIIHBUuQx4QeY8E8y5JcTicccdJ02bNpVJkybJ7bffbiKAOcvpviKgCCgCisBfCPiybIlb+9BDD5lQii+++KIcccQRUr16dSlZsqQ8+OCDsnv37r9a1j1FQBFQBBSBMAK+yHb9+vWycOFCYc7t0UcfLcWKFZMSJUrIVVddZYKIE6wm3HLEHU1UBBQBRSB1CKzcuC11ncXpyRfZRmuLb5MR4zZavt90XBX28zt+62p5RUARUASCiIAvsi1fvrzUrl1b7r\/\/flm2bJn8\/vvv8sMPP8iIESPM53LKli1b6DHiqhgzZowQ9KbQjWkDioAiUKQRWLVxuxk\/naqa8wAAEABJREFUL8cQc5CmP17J1qgXCoWEl2NE\/br88stl9uzZJuQi7gNcC1i4pmAh\/nz11Vcya9YsITB5IZrRqoqAIqAIBAoBX2SL5sSvvfvuu43vFlLEh0uc20RYtb\/88osMGzZMOnbsKFjR9BdNqlWrZvzEfHAyG2T16tVZNR57TnRcyzPqvGbT+YIjpn+72VDIUWmOi4ASvsh2y5YtMnDgQBP5C8JlZsI999xjjokExpcc1qxZQ7sFEtwHVapUkXr16sWtjxsjJydHskUqV66cNWNxnhMdV2Zdo9l0vj7+bGGYRxpWL7yLM9xYtJ046b7IlraYkfDOO+8IxGuPx48fL6TPnDlT2rZtS7JvWbx4sUybNs1YtaFQyHd9raAIKAKKgBOBGfusWtIaHlOGTVrFF9ky42Dnzp3Cxx553O\/Xr5\/Zf+KJJ4TIYIMGDTKfzinIiFauXCnTp0+X+vXrC+b\/2LFjDfH27NmzIM1pHUVAESjiCIz5dJ1BoNhv6yXdL8dQxBfZMiWLL+vymEhlKyxsIJ18d54tE2\/bvHlzM8MB9wBy0UUXGSLnY5Lx6mq+IqAIKAJOBAhAY\/21xX7bEIdsnTWTt++LbAkizgsy3AVM0UIttrwoYx\/rdunSpeyqKAKKgCKQFgQg2hvGLAr3XeLrt8L76dzxRbaQabdu3eSRRx4xj\/tNmjSR008\/Xe644w5hKtiKFSvM6rJEDAiLFms3EW1pG4qAIlB0EODzN9aqbbTXV7vf+m8CMXhfZIvGJ5xwgkycOFH42m5ubq4MHjzY+FrPPfdcOe644+T999+nmIoioAgoAslGIE\/7WLQD31shA99bbtLx0w5rX8vsB+GPb7LFbYBvliA0devWlWOPPVZWrVolHTp0kM2bN0upUqWCMC7VQRFQBIoQAhAtroM8RNuuViB8tfY0+CbbkSNHCiTboEEDscLLrKpVqwr+XNuwbhUBRUARSDYCkCzWLF\/Pta4DLNrcljnSqHr6p3s5x++LbH\/++WeZPHmyjB49Wl555RXjp12yZInccMMNcvzxx5vpX87GdV8RUAQUgUgIFDbNSbLWmqVNiPbtLqdI+7oVOQyU+CLbHTt2GOWZ3lWmTBnZsGGDbNu2TS688EKZMmWKQMamgP5RBBQBRSDBCECwvPy68LG5giXrJlms2S\/uOD1QrgMnBL7IltkIBJthehdki+8WoUGI1pIxxyqKgCKgCBQWAUuwXccsMgTL1roLaBtL1pJsbsuqJAVWfJEt82zbt29vpnetXbtWjjnmGGndurWRihUrSrly5QI7UFVMEVAEEoBAkptwkqu1YCFYLFpn15DsY+1rCZZs0EnW6u2LbKnEFC9cBvhoCav4wAMPyNChQ2XAgAGC1UsZFUVAEVAEvCAQi1ydFixtQbC5e198QbBIEP2y6BlNfJEtwWeIi\/Dbb78ZYsWtQNzZk046SV599VUhRGK0jjRdEVAEii4CkCqChYqlitVarvvksGuA9EjkCqFiwW58sFnYioV0MxFJT2TLSzBmIbz77rvy+uuvC1sif1l58803TToknIkgqM6KQPYjkLoRWlKFQC2x8kIL4Zh0N7GiHSRqyfXtLqcYcoVoSSM\/08UT2WKxEtmLSF\/fffedCfBNXFsrxLG97LLLpEKFCpmOh+qvCCgCHhCAUBGIE4FE3dYqaeRFI1aW0kKkEKqbXIM2R9YDJHGLeCJbSPTll182Fm2PHj3MdurUqWKFJbqXXnqphEIahzYu4lpAEcgQBCBTBMJEIE8IFQvVCmkI+ZFIlaFisUYi1re7niIQLYSbjeTK2J3iiWz5sCMhFLdv3y5t2rQRtj\/++KM4hXzKORvXfUVAESg0AklrACJFpi\/dLJDlwPdWSJ8P1wuEav2pkCpkilAGQqVOJKUikar1tRY1Yo2EjyeyZfHCxRdfHF6ea5fpOrfkUy5SJ5qmCCgCqUcAUkSmO8gU0oRMIVErFz4+V0hnkcD4Rb8IhBpNWwgVwRpFsExxASipRkPsr3RPZHvYYYeZFWIE9Y4mTAej3F9N654ioAgkEwEnkWJ1QpgIZOq0TJ1kSjnIlLrRdINM7WM\/U63chMq0K9IQCLcouACiYeUn3RPZOhsk6tfcuXOFDz4SE4H5tRCws4zuKwKKgEhBMYAIEWuRQpCQKAKRYpFaMrVESh7lEMg0Vt+QKQJRIpAm1iky58aqZhaAfezPbVnVxBlQQo2FqLc832RLLNuuXbvK7t275ZRTTjGfssGPO2HCBG89ailFoAgjAIkikCKCnxSihEQRS6IQajQipX4sCCFSa5m6ydQ+7keyTpVQY6Fa+DxfZEschDFjxshDDz0kffv2lWuuucZ8J4xA4qSTX3iVtAVFIPMQgACdlqibRCFPJ5FCsAh+UkgXaxRxjxzidKZx7CZSa5lCoJZMrWVKHoQLkSLOtnQ\/tQj4IlsWN2DR5uTk5NGSY9LJz5OhB4pAJiKwT2cIFHGSKASJdBz7vXlr7yRRpyXqJlHa2ddsxA0kiriJNLdljvB4H4tILZlSP2LjmhgIBHyRbcmSJWX\/\/feXRYsW5VF++fLl4eW7eTL0QBEIGAKQHuIkUGuFQqI8yjsJlH0niWKFInPWbDdv7Wkr1hAhQMRJohAoFickilhrFEJ1W6RKpLHQzaw8X2TLlxiuvvpqYWEDn8Hp1auXdOzYUa677johGhhRwTJr+KptNiAA4SEFJVBrhUKiPMrTVjxcjiy9n4mbWhASzXW8dNJH+3hIZ0++L7Jl2E2bNjUryJo1a8ah1KpVS9566y0hGphJ0D+KQAIQgPCQSASK9YlgdTr9oBc+\/ud8USzUghAoamOFIk4SxQpFsEKxPrFEx19ZOc9be\/KVREFQJRoCvsh206ZNMmLECPn111\/l2muvlX79+sktt9wi+GyjdaDpigAIQJxIJPKEHCFPBPJEIFLE+QhvCRTrE6E92o4nkCcSjUCdJAqZIvo4Hw9VzfeLgC+yLV68uJnqhRXbvHlzYRYCBOy3Uy2f+QhAdEg88rzg+dUSjzzt4zsE6gUZiBOBPBH8mgjWJQJ5IpAmVihbJBqB8ihPe1761jKKQEER8EW2pUuXlkGDBsmCBQtMsHC2uBUuueQSmTRpkmhshIKehvTXgziReOSJtemHPNdu2e1lcALZIU7yzN37Jh7yRKKRpxKoJ3i1UAAQ8EW2Vl9mJNStW1ceeeQRee2118wMhXvuucd8ANKW0W36EIA0rcQiT4gT8UOetOtlZBAn8rdKJcwKpPZ1K4qbPKMRKOSK5O57kURdtT69oK5lgoyAb7LFemXqV+\/evaVevXpCHNsTTjhBiGlLKMYgDzZTdYPgEIgT4bEbwdeJ4OtE3MTJcSSfJ3V5ZKdNxAsuECcSy\/KMRJ5PXlTRhNGLRJ5KoF6Q1zLZgoAvsiWM4jnnnCNt27YVvqTLJ3JmzpwpTAGrXLmyxrP1cVVAcoglT6ItQZwIxIlAlm6r00mekCYCcSK051UFiBPxS57Ox3YsT6xOBOJEaNOrDlpOEShKCPgiW745xgwEfLVDhgwxsRF4aRYPsC+\/\/FIg6WrVqkmjRo2ET+xEqjNu3DihjBX8wcTMjVQ2yGmQniVRyDAegZJPHFHKIhAnQjtexgnBWYlFnrwkQqK9NFLy9IK2llEECoaAL7JlUQO+Wny2XrsjXgIE3aVLF1m8eLHk5uYKn9NZt25dviZWr14tjz32mJnxQCSxIIdthAgRyBGyxBJFrCXqtkAhT4Q6+QbuSnASp5M8eRRHeFxH3MTJcSzL07br6k4PFQFFIAUI+CLbguizatUq2bNnj7Fo+dR5\/fr1zQu19evX52sOAi5fvny+9CAkQJJOYuURH4FoSYdIkWi6WqJzkmeu4207vk3I0m11OsmTx3WEx3WENqP1FzddCygCikBKEUg62dauXVv4flm5cuWEWLizZs0ycRQqVqyYZ6AslFi7dq3ceeedUqNGDfNViPHjx5s6eQruO8DVQEyGZMqMeUtk6LsL5OwH\/iduYt2nRnjD8k2Et+8X1DpI+jQvbwQSZbXRm\/+qKMgj55WRng1KGPlnzT+kQfltRg7\/Y738\/vM6SeZ40tE2Tyvp6DfZfeq4lgf+WoUjkPBNmuYdX2RLVC9eiLF16s3xnDlzZNeuXc7kPPv4eU8++WQTVwFfbJkyZfLk79y5U0qVKiXMcvj666+Nq2Hw4MHy+eef5ylnD3AzsHItGVL8kCNkyMztwoR8fKkEHbH9ssWitBaqfaz\/sk9jQT7ocbo8f11d6XZeHSMXN6olDU+qYVbZxdKVF4yx8jM1T8eVE\/fcB+ncZtP5giMQ7tkgSDEvSjDdi5kIWAsPPPCA+UXjxZWVb775xizdjbWarE6dOjJv3jwh+LgVZ99ly5Y1n0hv3Lix8NKtSZMmctppp8ns2bOdxZK6j6sAtwBWLK4B25klVx77ra\/UPt7bx3pbNvlb7UERUAQyEQFPZMuHHPmg4\/nnny9YsGwbNGhgHvXZsoIMMoUw3SBMnz5devbsaaaKkYdZz7zcNWvWcBgW5u4ylYwpZeHEvTupiCQGyRJmLxLJQrD4UiHX3JZVpVH1vBb5XhX1vyKgCCgCcRHwRLZ8yJGZAfhbIVq2mOdWlixZIn369DEvvtw9stABi3b+\/PlmOS\/TwBYuXCh169bNU5SZDkQPw02BJT116lSzLNhdLk+lBBxAtBc+PlcIcmKbw5K1FiwEa9N1qwgoAopAQRHwRLaQ38aNGwU\/Ky+weNFlXQh2i5uBcm5FatasKd26dZMbb7zRvPgiWtj1118vp556qlAX\/+2HH34oVapUMS\/H+vfvb8rhu2WxBPXdbSbqePrSzebFF4RLm5BsbsscwZJNggVLFyqKgCJQRBHwRLa4EW644QbBN4s7AdeBW0innLj+hUIhueCCCwSLFUuYLcehUEisxUwEMarRJv5cyuF+sDFzyUu04JPForXt8sILaza3ZVWbpFtFQBFQBBKGgCeyhRRHjx4t+FpxJ0CGbiGdcgnTLIkNQbS8CLNd5O61ZvHJYtnaNN0qAoqAIpBIBDyRbSI7THdbuAzcRJsbx5pNt87avyKgCGQ+Ar7I1vpYmVHgFnyv5AcZEojW6TrI3WvR5irRBvmUqW6KQNYg4ItscRPgLrAuhG+\/\/VY++eQT6dSpkwm1yMyDICPDjAMIFx2ZH5urRAsUKoqAIpACBHyRrVufUCgkLLvt3LmzeXkWa1GDu25Kjh2d4KdFSMI3y8ov9lUUAUVAEUgFAoUiW6sgS223bNli5tHatCBtsWaxatEJoh3Wrha7KoqAIqAIpAwBX2TLXNp27doJS2md0qJFCzONK9IKspSNJEZHWLQQLkUaHlNGV4EBhIoioAikFAFfZMsqL\/yzxKR1ygsvvCB8g4wQionRPnGtQLJOq1bdB4nDVltSBBQB7wj4Ilu+1HDmmWfKSSedZL4\/1qpVKyEmQuXKlU3wGO\/dpq6kJZfQuhMAABAASURBVFp6zG2Zw0ZFEVAEFIGUI+CLbNFu5MiRcvbZZwthEDn++OOPhWlfEyZM4DBQglWLCwGl8NUyA4F9FUVAEVAEUo2AL7IlPgJxDFhNRihElL3iiitkxIgRMmbMGNm6dStJgRFLtCikVi0oqCgCikC6EPBFtrt37zbugkqVKuXRt3r16kIeQcTzZKTx4E+r9s\/vnKlVm8YToV0rAoqAQcAX2R5yyCHCS7I33ngjPM2LCGAEl+EjkCVLljSNBuEPZIugCzMQ2KooAoqAIpAuBMJk60WBAw44wIRLfP3114VP3DD9i1CJffv2le7duxsi9tJOKsoMem95uJv2dY8I7+uOIqAIKALpQMAX2aIgkb8Igzhq1Chh+tfw4cOFQN\/MUCA\/CIJFO\/3bzUYVQidqbFoDhf5RBBSBNCLgi2z5+i2LGljcALky9YsYtFi8aRxDvq5n7CNaMtrXU6sWHFQUAUWgsAgUrr4vsiUQTa1ateSzzz4TfLWF6zp5tcd8qi\/GkoeutqwIKAIFQcAX2RJohm+I9ejRQ+rXr59n2a61eAuiRCLrOF0IR5UtkcimtS1FQBFQBAqMgC+yZSZCly5d5OGHH5a7777b+Gzx2yIs4yW\/wJokqKLThXCbrhhLEKrajCKQ0QgEQnlfZMtyXb4Lhq\/WLaSTn+5RWRcCeuiLMVBQUQQUgSAg4IlseSHWrVs3WbRokeAuYMqXW0inXDoH5XQhMAshnbpo34qAIqAIOBHwRLa4Bzp06GACheMuwG3gFtIp52w81fuQre2zYfWydle3ioAikKEIZJPansgW9wBTvIhXGynq1xlnnCFBcCM4\/bW6aiybLlMdiyKQ+Qh4IlvnMIMc9WvG0k1GVWIhqL\/WQKF\/FAFFICAI+CLbIEf9woVgV43plK+AXF2qRtFFQEeeDwFfZEtkr+LFi0sQo35BtnZ06q+1SOhWEVAEgoKAL7INctQv9dcG5ZJSPRQBRSASAr7IlhgITAELYtQv9ddGOr2apgj4RUDLJwsBX2SLEsmM+sVS4HPOOUeqVasmjRo1ksmTJ9NlXMGFoP7auDBpAUVAEUgjAr7JFl0JFJ7oqF9bt26Vfv36CcuBFy9ebJYCDxw4UNat+zOoDP1GE8jW5qm\/1iKhW0VAEQgSAgUi22QMYNWqVbJnzx5j0e63334m0A2kvn79+rjdqb82LkRaIPsR0BEGHIHAkG3t2rXl5ZdflnLlygnhG2fNmiWQbsWKFSNCiKth+fLlgnz45V\/Wb+jXn0wa6Zkkq1evzki942Gs4\/rzGo2HU1Dys+l8wRFIRAJJQ2JgyNaOfcGCBXLyyScLYRybNm0qZcqUsVl5tsuWLZOcnBwjB5YoYfJYzNDwpBomzeZlyrZy5coZqXc8fHVcf16j8XAKSn42nS84AjHkEIA\/vsl2y5Yt8sorr0ivXr3yCP5V8go7pjp16si8efOET+9YidUm\/lp9ORYLIc0LOgKqX9FAwBfZ7tixQ2677TZ58sknjX81kRBNnz5devbsKfRBu5j\/zHxYs2YNh1EFsrWZ+nLMIqFbRUARCBoCvsj2559\/Fr7W8Oyzz8qAAQPM7AFmECBEAStdunSBx1ehQgVj0c6fP19+\/\/13YRrYwoULpW7dujHbXLVpezhfg8+EodAdRUARCBgCvsiW6F+HHnpoUoZQs2ZN85n0G2+8UWrUqCHXXnutXH\/99cKn0mN16LRs8dnGKqt5ikBCEdDGFAEfCPgi25IlS5pQioMHD5YlS5bIjz\/+GBYCh2OR+ug7T9FQKCQXXHCBzJw5U3Bqs+U4FArlKec+cK4cU7J1o6PHioAiEBQEfJHthg0bZNiwYfLOO+9Iy5YthRi3Vi6++GIhP9UD05djqUZc+1MEFIGCIOCLbPmU+ZQpU4zlifXpFNLJL4gSBa3jdCFUKVeioM1ovSKHgA5YEUg9Ar7IFvVYcMBcWF6QMd0LdwLfJiOd\/FSKk2yPKndgKrvWvhQBRUAR8IWAb7Jl7us111wj3377rWDNElCcSGCjR4\/21XEiCusy3USgqG0oAopAKhDwRbYEixkzZow8+uij0rdvX2F1FytfsHLx4zI1LBVKR+rjKHUjRIIl29J0PIpAxiLgi2y3bdsmfK0BgnWOuEqVKubQLkgwByn4ozMRUgCydqEIKAIJQcAX2R588MFy0EEHmelZzmleS5cuFaaFIQnRymMjK\/ctaNBvjnkETIspAopA2hDwRbYsaujatatZOda6dWvhRdmll15qFiNcfvnlhohTNRJejiGp6k\/7KRwCWlsRKOoI+CJbwCJo+Pvvvy+33367tG3b1qz0evfdd4UIXeSnQzQmQjpQ1z4VAUXADwK+yZbGiYHQqlUrYWltu3btJNXza9HBadVqTAQQUVEEFIEgI+CbbNeuXSsQLMG+69evL8cdd5whXZbrpnKgzmlfqey3yPWlA1YEFIGEIOCLbJltcO+998qRRx5pXpIx13batGnC52vuv\/9+M1MhIVp5aGTlxm3hUo2qRw4wHi6gO4qAIqAIpBkBX2TLPNrNmzebuLOERAyFQlKxYkW5+eab5fvvv5eNGzembDirNv4ZWlHn16YMcu1IEVAECoGAL7JlNgILGbBw3X0SepF8d3qyjnXaVzxkNV8RUASChIAvsmUebfv27eXOO+8UZiDgv2XJLl9vaNKkibDogbCLyR7gnpLlxfmCLNn9afuKgCKgCBQWAV9ki5tg+PDhsnLlSmGJbrt27QzxQrqPP\/64EGYRKaxSfurrtC8\/aGlZRUARSBcCvsgWPy2fG586darEkmQPZk\/Jv74WkYU+22TDp+0rAopAGhDwRbbot2vXLnnhhRekefPmwtQvvqZAEBrn8l3KJVNwI9j2q5QtYXd1qwgoAopAYBHwTbbPPfecjB07Vrp37y4jR440roMhQ4bIM888k7JBOsk2ZZ1qR4qAIqAIFAIBX2SLz3bSpElC0PDzzjtP6tSpI1deeaWMGDFCSCe\/ELp4rrq7\/LHhskGaYxtWSncUAUVAEXAhUMx1HPOQ8IqhUEjKli2bpxzHoVAopYsaUED9taCgoggoApmAgC+yPeSQQwzRTpgwQayPls\/h8CVcQi8SgjEVg7YvyDS0YirQ1j4UAUUgEQj4ItsDDjjArBbjaw2nnXaaMLeWl2R8tYHQi6lY1LBy43ZJuM82EUhqG4qAIqAIxEDAF9nSTo0aNWT8+PHCi7Lc3FwZOnSomQZG6EXyUyk6xzaVaGtfioAiUBgEfJMtnRF4BnIlzGKDBg0Ei5f0VAiWbSr60T4UAUVAEUgkAgUi20QqUJi2Gh6j0b4Kg5\/WVQQUgdQhkHFkq3FsU3dxaE+KgCKQOAQyjmydQ9epX040dF8RUASCjEBKyHb16tXm6w7HHHOMnHjiiTJq1ChhypgbmHHjxkm1atXCwnfN3FHEZizdFK6mZBuGQncUAUUg4AgknWwJu8jXHSDOxYsXC4Fsnn\/+eZk1a1Y+aCDlxx57TJYtW2ZkypQpUb9vpkSbDz5NUAQUgQAjkHSy3bp1q\/CFB2YuFC9eXPh2GTMZvvjii3ywrFu3TsqXL58v3ZmgQcOdaOi+IqAIZAoCvsnW78BYyksM3EqVKpmqmzZtEr5dxud0TMK+P7\/++qsQF5fA5MzlZUoZ83nd7gY79WvmXst4+fLlki2CVZ8tY3GOQ8eVWddoNp0v65LcRzFp3ySdbJmTW65cOcGqXbNmjdx0003Cst6zzjorz+B37twppUqVkt69e8vXX39tgt0MHjxYPv\/883A5S7QkXHJuU8nJyckaqVy5ctaMxXledFyZdY1m0\/my7kj4IgiSdLJlkMTAfeqpp+TCCy+U448\/Xp544glDuORZwQIeNmyYNG7c2BAzS4FZEjx79mxbJM\/2qHIH5jnO9AP3j0+mj8fqr+OySGTGNlvPV2rRj9xb0skWN8Czzz5rvln21ltvCUt8I8VQWLRokTz55JOyY8eOPJpiBedJ2HfwaP87w7MW7ONCJm8ZVibrH013Hddfs2uiYRSk9Gw8X4wpCJJ0ssUH9MEHH8iDDz4oPKJEGzRRwyBjIogRUYzP7ixYsEDq1q0brnJUuRKy8cFm8sUdp8vqj180Mxbso4Ju\/5zBoTgoDnoN5L8GwiSSxp2kk+33338vX331lfB44vwFJ+A4c2iZEvbhhx9KlSpVzMcj+\/fvL7wgw3fbq1cvqVmzZj54IN18iZqgCCgCikDhEEhq7aSTLZbpwoUL81mhnTt3NnNomUvL98wYJTMQJk6caMpOnz5dmjVrRrKKIqAIKAIZj0DSyTbjEdIBKAKKgCKQAASUbBMAojahCCgC6UUgE3pXss2Es6Q6KgKKQMYjoGSb8adQB6AIKAKZgEDGkC3zdUeNGmWihh133HFCcJtffvklEzAO6+h1DEx9e+GFF8xYiZTWrl07s5Q53FDAdryOy6k2Hw298sorJajn0M+YPvnkE2nUqJGZ933OOefIl19+6RxqoPa9josAUgMHDhTuNWYHXXbZZcm5BlOADrFZrrjiCmEqaQq6i9pFxpAty3ZfeeUVYS7up59+KitXrpSxY8dGHVgQM7yOgZv31VdfNWPlxmVGBz8u7gUfQRmj13FZfVm2zbxrVhbatKBtvY5p1apVZmn5Qw89JES1O\/\/884XVkrt37w7akIw+XsfFdMy5c+fK\/\/73P5k\/f75UrFhRHn30UdNGpvzBaGHe\/l133WWmn6Zb74wh28mTJwtzcll3X7p0aRMf96OPPhIC2KQbRK\/9ex0D85LPPPNMEyuB1XYtWrQQCIpfaK99pbKc13GhEyTESsEzzjgj35Jt8oMiXsfE4ptTTjlFWFq+3377SadOnaRPnz5myXlQxuLUw+u4bB3Ol90\/\/PDD7W5GbOEGppYSl4X7KN1KZwTZcsJXrFgh1atXD+PFid+4cWPGkK2fMXTo0EGYh2wHu2TJEuFGTuWHNW3f8bZ+xkVbH3\/8sXEdECeD4yCKnzHx5LF9+3Y599xzBZfP5Zdfbq7JUCgUuKH5GRdz3ytUqCD169c38UyYK3\/ppZc6xhT8XYwywgPcfvvtQuyVdGucEWTLxQyxsqQ33YAVtH8\/Y+AiYaw8BuE66devn3Ts2FEOOeSQgnaftHp+xkUIzeeee85Yf\/\/3f\/+XNJ0K27CfMeEKIVgSbgSi1TVs2NC8T8DnWVg9El3fz7hw0bHCc9q0acaNwBdWBgwYIBB2ovUqKu1lBNkSprFkyZLChe08MVh7oVDwLAinjnbf7xiICXvxxRcLQXyefvppYznZtoK09TouXswwFizASEuwM3FMVue2bdtKrVq1BCxYlv7DDz8IQfNtflC26OflPuLdAO4Gnj6IQ80Pf\/v27eW7774T4lEHZTyZpkdGkC2Pz8ROwJdpAeaC5jEHK9CmBXnrZwz4Z2+88UYTkvKdd94RvmwR1LF5HRf+M84fweGJkdG6dWshQBGWYLrfErtrPix7AAAIq0lEQVSx9Tom6vHiCIuRfSvFihWTUCh4RsC+ceV5WRTpPsLHWaJECTuc8BayJi+coDu+ECjmq3QaC\/OWF2c3Fh+\/rkyNuuCCC4QLKI1q+era6xhee+01ExfiqquuCuyLFufAvYwL62j06NEm7gVRqZhVcvbZZ8uMGTOkTp06zuYCse9lTChK\/I5JkyYJPk1cB6+\/\/rrwwize552omw7xMi6eGE8\/\/XR5++23zYtZxsW3AzlPQfB9pgO3RPSZMWR76qmnCg56LCIuhBNOOEGY05gIEFLVRqwx9OzZUxB8YkuXLhU+fMkLF6xAhJkY+NBSpauffryMy097QSjrdUyUu\/766+Xqq68286I3bNggPJWEQsGzbMEVfaPdR1x\/COX++c9\/Ci\/JcPvgr8W1cMsttwTSYkffTJCMIdtQKCRMrGbOHy8ibrvtNuMjywSQrY6hUPQxDBkyRBCsCr5YgfXnFKz6ww47zDYVqG0oFH9cboWxkvhiBxavOy8Ix6GQtzGFQiHhCYv5nMwaeeSRR5L25jsRuIRC3saFy+C6664zL8cYF\/OiowXyT4ReyWyD+4bvGXLNJbOfeG1nDNnGG4jmKwKKgCIQZASUbIN8dlQ3RUARyBoElGyz5lTqQAKLgCqmCOxFQMl2Lwj6XxFQBBSBZCOgZJtshLV9RUARUAT2IqBkuxcE\/a8IREZAUxWBxCGgZJs4LDOupREjRpi5vV4U91PWS3u2DIsBiAVL8B0\/sW2ZD4pOtp0gbAlLyJxoVvwla1Uc7dI+wn4Qxq06eENAydYbTloqSQgQnOaoo44Swi6mes4t8RoI9pPIoV100UUyb968pK2KY64oq+4aNGiQSLW1rRQgoGSbApDT2QUBUbp37y424j77pGGFDRo0yARgt1YlE7+xMrHOCFhOEGyC\/0Qqu2jRIiEAC6vcuPGpC3lFGisT\/lntR7vOsrRLNDPy2botW9qjXerQz3nnnZfnKwgQNV+xII\/2CXdI\/7t27TIBvBkDfTIm2qE9rMFWrVoJYycdyxo8wIUvEtDXww8\/bCKTWX2IfEWAGdqiH\/Sln1hCXXDF+rY4XXLJJcJy80j13JY62FCfdiKV17TMQ0DJNvPOmS+NWd\/ODUvUfVbfESSFKGIsxWQVHpYYMQsgnJEjRwqhAlm5xj7hHVk95C5LEPMePXrIv\/\/9b\/N1gmeeeUaGDh0q06dPz6cbXy8g+Mx\/\/vMf+fbbb8VZlnaxaCE4tm7LlvZolzq0w0otwvwxHjrCwrv77ruNDn\/\/+98FkmRZKSsM3333XeFrF\/TZu3dvgfTWr19PNVm9erUQTPrjjz8Wln0zZkI+ghGf65kzZ044ahfk3KdPH6FfsGAcjAd9TGNx\/rzxxhvC6it0atKkiXHbgHWcapqdhQgo2WbhSXUPCcLh0ZalwCwJvvnmm91FhMAphEBk7TyEhRUIMUd6zIakWAJJIBmiQNWuXdt8g2vWrFn52sUqPPbYY4XoXqFQSCjbsmVLgQzzFXYlUIay1KEfrFi+hMA+RVm3T2hDjtH7t99+M2E4CeHID0rVqlVNIG+Wnu7Zs0cYE\/WOPPJIIcYG6RAwn4ohhCBkT6AVvo3G2ClLXNfGjRubLzHQDz8MRx99tAk8Q348+cc\/\/mG+uEFf9ImOK1asiFdN87MQASXbLDypziFhuXbt2tUEtMaK41HWPm47y0GwxGQgYhUWJ9YYxOAsY\/exzPgczPHHH28+csjj9fPPPy\/ff\/+9LRLe4m6gX4jeJkJquCfssXNr97Fev\/vuO6GsTYMIIT6sUtKceRxb2bJli2Dx8hIJgnvuuecMCdt8dLEhBNEZXQjXafP5CkipUqXMIWMlwhyuCsYJuRPli3qmQJw\/zq+L0Cd9x6mi2VmKgJJtlp5YOyweX4mShpUIyeJzxF9JfFlbhi0hD3mMnjJlimCNYgFDbuS5BSsN\/ymP0rgckC+++MJ8e8tdFusSy9palWw5rlixortonmOIqVy5cnm+DMDjP4FeINM8hV0Hjz\/+uEnBHcB36vg0iiVok+H4c+ihhwpW7ObNm8Op9APJksBYu3TpEg4Nie6fffaZcaGQH0+IF2vLEPeWJwUsZJsWbeusF62MpmcWAkq2mXW+fGsLeUKcWIpUxlolWj8WFkTifLwmHzKg7EsvvWQsVfJJd5bFuvzmm29MLFrIE7LlZQ5kTVmnQPS4FyAoyvLIznGzZs2cxfLtox8uhP\/+97\/mpRIxVfHd0hf656vgSqAvwlUS+xirG4Lm2FXMROg64ogj5MUXXxT6IHA75W05flTee+894zagTcbxr3\/9y3y1wJaJtcUNQZu0PXz4cKlcubJxK7jrEFELa5k+IHpe6LnL6HFmI6Bkm9nnL67211xzjfFV4h5gRgIvle644w4TdB0i5Bjy4G17Tk6OkIYbgX2+6tutWzchji7ptiw+z\/vuu0\/69+9vPnII0Xbq1Mn4bd0K4Uu96667TIxXHsVvuukmufXWW4V0d1n3MVY4Pk9cAcRUxeLESoWI3WWdx4QGxBeL+wJfLO1gYfPGH9JzlsXi7bP3BRizBChPrFd8zLYMMxbAgHi16M\/LMVwU+IVtmVhb6nMOaJs+6Is+wZQYxcw6oP61114rPB3wgUW3Du6ylFfJPASUbDPvnPnSGIsJ\/ytv0nkEJuI+1hWN8OIJK5PZCDzW23JMbWrTpo307dtXsIx5GeYsi6+UF0UTJ040j9eUZ6ZAKJQ\/YHYoFDJBqCmDVcoMA2dZiJ3+aROdnII1DXEyiwL9mbEA0VMGa71z587sGnG2QxnGyXjRkTysVfqpV6+eYDUyJipiSeKyIJ\/y6MfYsJ7pPxQKhePVoj\/tkU9dL8K8WOrQNjqhG\/XoH2zRjWO+9TVu3Dj59NNPzcvDe+65R9AXXNxlKa+SeQgo2WbeOVONE4jAzp07hSlwzCnGhYLbAZJjmlYmfXIpgZBoU0lC4P8BAAD\/\/4y6FhAAAAAGSURBVAMAI1ZRHjV7aMoAAAAASUVORK5CYII=","height":209,"width":347}}
%---
