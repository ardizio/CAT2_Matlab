%% Startup functions
clear
close all
clc
%% General settings
font_size   = 18;
line_width  = 2;
plot_x0     = 500;
plot_y0     = 300;
plot_width  = 600;
plot_height = 400;
%% Simulink Settings
MaxStep  = 1e-2; % 0.01s  solver max step size
RelTol   = 1e-2; % 0.01s  solver relative tolerance
Tf       = 10;   % 10s    final simulation time
%% System parameters
l = 0.3;   % lunghezza asta del pendolo [m] (pendulum length)
M = 0.5;   % massa [kg]
b = 0.05;  % coefficiente di smorzamento [Nms/rad] (damping coefficient)
g = 9.81;  % accellerazione di gravità [m/s^2] (gravity acceleration)
%% Linearized system around theta_star = pi/3
% dove theta_star è il punto di equilibrio
theta_star = pi/3;

C_star     = M*g*l*sin(theta_star);

% ricavo le matrici dal modello lineare
A_lin      = [0                                      1;
              -M*g*l*cos(theta_star)/(M*l^2) -b/(M*l^2)];
B_lin      = [0; 1/(M*l^2)];
C_lin      = [1 0];
D_lin      = 0;

disp('Autovalori del sistema linearizzato:')
disp(eig(A_lin))
lin_lti   = ss(A_lin, B_lin, C_lin, D_lin);
[num, den]= tfdata(lin_lti, 'v');
%% Simulating the system around the equilibria with C_m = 0 (coppia forzante)
t_data = linspace(0, Tf, 10000);
u_data = zeros(size(t_data));
u      = timeseries(u_data, t_data); % convert to timeseries
% stable equilibrium
figure(1)
subplot(1, 2, 1)
hold on
for i = [-pi/6 pi/6]
    for j = [-3 0 3]
        x0 = [i; j];
        SimOut = sim('p54_pendulum_sim.slx', 'SaveOutput', 'on');
        theta_signal = SimOut.theta;
        omega_signal = SimOut.omega;
        theta_val    = theta_signal.Data(:);
        omega_val    = omega_signal.Data(:);
        plot(theta_val, omega_val, LineWidth=line_width)
    end
end
grid on
box on
xlim([-pi/3 pi/3])
ylim([-4 4])
set(gcf,'position',[plot_x0,plot_y0,plot_width,plot_height])
title('Stable equilibrium', FontSize=font_size)
xlabel('$\theta$ [rad]', FontSize=font_size, Interpreter='latex')
ylabel('$\dot{\theta}$ [rad/s]', FontSize=font_size, Interpreter='latex')
% unstable equilibrium
subplot(1, 2, 2)
hold on
for i = pi + [-pi/12 0 pi/12]
    for j = [-0.5 0 0.5]
        x0 = [i; j];
        SimOut = sim('p54_pendulum_sim.slx', 'SaveOutput', 'on');
        theta_signal = SimOut.theta;
        omega_signal = SimOut.omega;
        theta_val    = theta_signal.Data(:);
        omega_val    = omega_signal.Data(:);
        plot(theta_val, omega_val, LineWidth=line_width)
    end
end
grid on
box on
xlim(pi + [-pi/6 pi/6])
ylim([-1 1])
set(gcf,'position',[plot_x0,plot_y0,plot_width,plot_height])
title('Unstable equilibrium', FontSize=font_size)
xlabel('$\theta$ [rad]', FontSize=font_size, Interpreter='latex')
ylabel('$\dot{\theta}$ [rad/s]', FontSize=font_size, Interpreter='latex')
%% Testing the system around [pi/3, 0]
t_data = linspace(0, Tf, 10000);
u_star = C_star*ones(size(t_data));
u_sin  = 0.03*sin(2*t_data);
u      = timeseries(u_star + u_sin, t_data); % convert to timeseries
x0     = [pi/3; 0];

SimOut = sim('p54_pendulum_sim.slx', 'SaveOutput', 'on');
dtheta_signal     = SimOut.dtheta;
time              = dtheta_signal.Time(:);
dtheta_data       = dtheta_signal.Data(:);
dtheta_lin_signal = SimOut.dtheta_lin;
dtheta_lin_data   = dtheta_lin_signal.Data(:);
figure(2)
plot(time, dtheta_data, LineWidth=line_width)
hold on
plot(time, dtheta_lin_data, LineWidth=line_width)
grid on
box on
xlabel('$t$ [s]', FontSize=font_size, Interpreter='latex')
ylabel('output [rad]', FontSize=font_size, Interpreter='latex')
legend('$\theta - \theta^\star$ (nonlinear system)', '$\delta\theta$ (linearized model)', ...
       FontSize=font_size, Interpreter='latex')
title('Nonlinear system vs. linearized model', FontSize=font_size)