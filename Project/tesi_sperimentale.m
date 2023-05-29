%% Startup functions
clear
close all
clc
%% General setting
plot_font_size   = 18;
plot_line_width  = 2;
plot_x0     = 500;
plot_y0     = 300;
plot_width  = 800;
plot_height = 400;
%% Transfer function variable
s = tf('s');
%% Simulink settings
MaxStep  = 1e-2; % solver max step size
RelTol   = 1e-2; % solver relative tolerance
T_f      = 10;    % final simulation time
%% Inverted pendulum parameters
m_a    = 0.116;  % massa asta mass [kg]
m_b    = 0.05;  % massa posta al termine dell'asta [kg]
m    = m_a + m_b;% approssimo unendo asta e massa piccola [kg]
M    = 0.61;     % massa del carrello [kg]
l    = 0.45;     % lunghezza dell'asta [m]
J_G  = 1/3*m*l^2;% pendulum inertia [kg*m^2]  % 1/3*m*L^2
k    = 0;        % spring stiffness [N/m]
g    = 9.81;     % gravity acceleration [m/s^2]
beta = 0.1;      % damping coefficient [Ns/m]
%% Linearized model
% states: x_1: linear position
%         x_2: angular position
%         x_3: linear speed
%         x_4: angular speed
M_lin = [m*l/2 J_G+m*l^2/4;
         M + m       m*l/2];
A_1   = [0 0 1 0;
         0 0 0 1];
A_2   = (M_lin^-1)*[0 m*g*l/2 0 0;
                   -k  0  -beta 0];
A_lin = [A_1;
         A_2];
B_1   = [0; 0];
B_2   = (M_lin^-1)*[0; 1];
B_lin = [B_1;
         B_2];
C_lin = [0 1 0 0];
D_lin = 0;

LTI_lin = ss(A_lin, B_lin, C_lin, D_lin);
G_lin   = tf(LTI_lin);
disp ('Transfer function of the linearized model:')
zpk(G_lin)
%% Control design
% regulator structure
R    = -(s + 4.5)*(s + 0.4)/s/(s + 9);
figure(1)
rlocus(minreal(R*G_lin))
% gain assignment
R = 18*R;
L = minreal(R*G_lin);
F = minreal(L/(1 + L));
% checking the requirements
mindecay   = 1.5;
mindamping = 0.5;
maxfreq    = 6;
Goals      = TuningGoal.Poles(mindecay, mindamping, maxfreq);
figure(2)
viewGoal(Goals, F);
% state-space realization
R_lti   = ss(R);
A_ctrl  = R_lti.A;
B_ctrl  = R_lti.B;
C_ctrl  = R_lti.C;
D_ctrl  = R_lti.D;
%% Parameters for simulation
% initial conditions
x0        = -5;
theta0    = deg2rad(45);
v0        = 0;
omega0    = 0;
% angular reference
theta_ref = 0;