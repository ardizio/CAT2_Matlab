%% Startup functions
clear
close all
clc
%% General settings
plot_font_size   = 18;
plot_line_width  = 2;
plot_x0     = 500;
plot_y0     = 300;
plot_width  = 500;
plot_height = 400;
%% Transfer function variable
s = tf('s');
%% DC motor parameters
Rm      = 3.2;          % terminal resistance [Ohm]
La      = 4.1e-3;       % armature inductance [H]
Ke      = 0.12;         % voltage constant [V*s/rad]
Kt      = 0.11;         % torque constant [N*m/A]
J       = 0.352e-4;     % rotor inertia [Kg*m^2]
beta    = 4e-4;         % friction coefficient [N*m*s/rad]
V_DC    = 24;           % rated voltage [V]
omega_n = 1500/60*2*pi; % rated speed [rad/s]
V_max   = 8;            % saturation for simulations [V] scelta arvitraria dopo aver visto che 8V gestisce lo steady state, ho scelto 8V per tagliare i picchi gestendoli in maniera opportuna
%% System matrices
% states: x_1: current
%         x_2: angular speed
A   = [-Rm/La   -Ke/La;
         Kt/J  -beta/J];
B   = [1/La;
          0];
C   = [0 1];
D   = 0;
LTI = ss(A, B, C, D);
G_sp   = tf(LTI);
%% Simulink settings
MaxStep  = 5e-5; % solver max step size
RelTol   = 1e-3; % solver relative tolerance
T_f      = 0.3;    % final simulation time
% initial conditions per SIMULINK
I_0      = 0;
w_0      = 0;
theta_0  = 0;
%% Part 1: Speed control PID ANELLO APERTO [calcolo il modello approssimato singolo polo per tuning PID della velocita']
% Calcolo la risposta al gradino, creo y e t separate
[y, t] = step(G_sp);
%% MOSTRO risposta al gradino dello Scenario senza distrunbo (No-noise scenario)
figure(1)
set(gcf,'position',[plot_x0,plot_y0,plot_width,plot_height])
plot(t, y, LineWidth=plot_line_width)
hold on
grid on
box on
xlim([0, t(end)])
%% Static gain
mu = y(end);
%% Method 1
dy        = diff(y);
dt        = diff(t);
[dy_j, j] = max(dy);
m         = dy_j/dt(j);
y_j       = y(j);
t_j       = t(j);

T         = t_j - y_j/m;
tau       = mu/m;

t_val     = linspace(T, T + tau, 1000);
plot(t_val, y_j + m*(t_val - t_j), ...
    LineWidth=plot_line_width, Color=[0.8500, 0.3250, 0.0980])
plot([T+tau t(end)], [mu mu], ...
    LineWidth=plot_line_width, Color=[0.8500, 0.3250, 0.0980]);

G_approx1 = mu*exp(-T*s)/(1 + tau*s);
y1        = step(G_approx1, t);
plot(t, y1, LineWidth=plot_line_width, Color=[0.9290, 0.6940, 0.1250])
%% Method 2
A_1       = trapz(t, mu - y);
T_tau     = A_1/mu;
j         = 1;
for i = 1:length(t)
    if t(j) < T_tau
        j = j + 1;
    end
end
A_2       = trapz(t(1:j), y(1:j));
tau       = A_2*exp(1)/mu;
T         = T_tau - tau;

G_approx2 = mu*exp(-T*s)/(1 + tau*s);
y2        = step(G_approx2, t);
plot(t, y2, LineWidth=plot_line_width, Color=[0.4940, 0.1840, 0.5560])
legend('Data', '', '', 'Method 1', 'Method 2')
%% Noisy scenario
y_n = y + (0.5*rand(size(y)) - 0.25);
figure(2)
set(gcf,'position',[plot_x0,plot_y0,plot_width,plot_height])
plot(t, y_n, LineWidth=plot_line_width)
hold on
grid on
box on
xlim([0, t(end)])
%% Filtraggio del rumore
% moving average
y_avg     = movmean(y_n, 10);
plot(t, y_avg, LineWidth=plot_line_width, Color=[0 0 1])
%% Static gain
mu_n      = y_avg(end);
%% Method 1
dy        = diff(y_avg);
dt        = diff(t);
[dy_j, j] = max(dy);
m         = dy_j/dt(j);
y_j       = y_n(j);
t_j       = t(j);

T         = t_j - y_j/m;
tau       = mu/m;

t_val     = linspace(T, T + tau, 1000);
plot(t_val, y_j + m*(t_val - t_j), ...
    LineWidth=plot_line_width, Color=[0.8500, 0.3250, 0.0980])
plot([T+tau t(end)], [mu mu], ...
    LineWidth=plot_line_width, Color=[0.8500, 0.3250, 0.0980]);

G_approx1 = mu*exp(-T*s)/(1 + tau*s);
y1_n      = step(G_approx1, t);
plot(t, y1_n, LineWidth=plot_line_width, Color=[0.9290, 0.6940, 0.1250])
%% Method 2
A_1       = trapz(t, mu_n - y_n);
T_tau     = A_1/mu_n;
j         = 1;
for i = 1:length(t)
    if t(j) < T_tau
        j = j + 1;
    end
end
A_2       = trapz(t(1:j), y_n(1:j));
tau       = A_2*exp(1)/mu_n;
T         = T_tau - tau;

G_approx2 = mu*exp(-T*s)/(1 + tau*s);
y2_n      = step(G_approx2, t);
plot(t, y2_n, LineWidth=plot_line_width, Color=[0.4940, 0.1840, 0.5560])
legend('Data', 'Filtered data', '', '', 'Method 1', 'Method 2')
%% PID tuning ANELLO APERTO
% using the parameters from the noisy scenario - method 2
%% from Ziegler-Nichols table CAT2_272
K_p_ZN = 1.2*tau/mu/T;
T_i_ZN = 2*T;
T_d_ZN = 0.5*T;

tau_p_ZN = T_d_ZN/10;  %/10 è arbitratio per rendere più veloce il Proporzionale rispetto derivativo
PID_ZN   = K_p_ZN*(1 + 1/T_i_ZN/s + T_d_ZN*s)/(1 + tau_p_ZN*s);  %R(s)= PID function
% Passo allo spazio degli stati
PID_lti = ss(PID_ZN);
A_ZN  = PID_lti.A;
B_ZN  = PID_lti.B;
C_ZN  = PID_lti.C;
D_ZN  = PID_lti.D;
%% Cohen-Coon table table CAT2_272
K_p_CC = (1.35*tau/T + 0.27)/mu;
T_i_CC = tau*(2.5*(T/tau)*(1 + (T/tau)/5))/(1 + 0.6*T/tau);
T_d_CC = 0.37*T/(1 + 0.2*T/tau);

tau_p_CC = T_d_CC/10;
PID_CC   = K_p_CC*(1 + 1/T_i_CC/s + T_d_CC*s)/(1 + tau_p_CC*s); %R(s)= PID function
% Passo allo spazio degli stati
PID_lti = ss(PID_CC);
A_CC  = PID_lti.A;
B_CC  = PID_lti.B;
C_CC  = PID_lti.C;
D_CC  = PID_lti.D;
%% Integral Absolute Error (IAE)
A = 1.086;       %5bis26 IAE PID ->P line, column A
B = -0.869;      %5bis26 IAE PID ->P line, column B
Y = A*(T/tau)^B; %5bis26 IAE PID viola
K_p_IAE = Y/mu;  %5bis26 IAE PID viola

A = 0.740;        %5bis26 IAE PID ->I line, column A
B = -0.130;       %5bis26 IAE PID ->I line, column A
Y_star = A + B*T/tau;
T_i_IAE = tau/Y_star;

A = 0.348;       %5bis26 IAE PID ->D line, column A
B = 0.914;       %5bis26 IAE PID ->D line, column B
Y = A*(T/tau)^B;
T_d_IAE = tau*Y;

tau_p_IAE = T_d_IAE/10; %Proporziionale più veloce del derivativo arbitrariamente
PID_IAE   = K_p_IAE*(1 + 1/T_i_IAE/s + T_d_IAE*s)/(1 + tau_p_IAE*s);%R(s)= PID function
% Passo allo spazio degli stati
PID_lti = ss(PID_IAE);
A_IAE   = PID_lti.A;
B_IAE   = PID_lti.B;
C_IAE   = PID_lti.C;
D_IAE   = PID_lti.D;
%% Part 2: Position control PID tuning with closed-loop method
% from Simulink
k_star  = 104.5;
T_star  = 2*9.907e-3;
% closed-loop tuning table  5bis30
K_p     = 0.6*k_star;
T_i     = 0.5*T_star;
T_d     = 0.12*T_star;

tau_p   = T_d/10;  %fisica realizzabilirà più veloce
PID     = K_p*(1 + 1/T_i/s + T_d*s)/(1 + tau_p*s); %Equazione PID
% Passo allo spazio degli stati
PID_lti = ss(PID);
A_PID   = PID_lti.A;
B_PID   = PID_lti.B;
C_PID   = PID_lti.C;
D_PID   = PID_lti.D;
%% Anti-windup design DA RIGUARDATE
N_R   = K_p*(1 + T_i*s + T_i*T_d*s^2);
D_R   = T_i*s*(1 + tau_p*s);

Gamma = (1 + T_i*s)*(1 + tau_p*s);

Phi   = minreal(N_R/Gamma);
Psi   = minreal((Gamma - D_R)/Gamma);

Phi_lti = ss(Phi);
A_Phi   = Phi_lti.A;
B_Phi   = Phi_lti.B;
C_Phi   = Phi_lti.C;
D_Phi   = Phi_lti.D;

Psi_lti = ss(Psi);
A_Psi   = Psi_lti.A;
B_Psi   = Psi_lti.B;
C_Psi   = Psi_lti.C;
% D_Psi = 0 by construction
%% Alternative simpler formulation
%{
N       = 10; % choose N sufficiently high
% PID structure
R_p     = K_p;
R_i     = K_p/T_i/s;
R_d     = K_p*N*(T_d/N*s/(1 + T_d/N*s));
% overall PID regulator (not used for implementation)
PID2    = R_p + R_i + R_d;
%}