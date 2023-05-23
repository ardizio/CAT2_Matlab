%% Startup functions
clear
close all
clc
%% General plot settings
plot_font_size   = 18;
plot_line_width  = 2;
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

% ricavo le matrici dal modello lineare [Pendolo fisico CAT1 foglio 6]
A_lin      = [0                                      1;
              -M*g*l*cos(theta_star)/(M*l^2) -b/(M*l^2)];
B_lin      = [0; 1/(M*l^2)];
C_lin      = [1 0];
D_lin      = 0;

disp('Autovalori del sistema linearizzato:')
disp(eig(A_lin))

% Creo l'oggetto Sistema con la funzione "ss()"
lin_lti   = ss(A_lin, B_lin, C_lin, D_lin);
% get [num, den] coefficients as row vectors rather than cell arrays 
% for a SISO transfer function represented by sys.
[num, den]= tfdata(lin_lti, 'v');
%% Simulating the system around the equilibria with C_m = 0 (coppia forzante)
% vettore da 0 a Tf dostanziato di  0.0010
t_data = linspace(0, Tf, 10000);  
% vettore da 10000 elementi contenente zeri come input di ingresso 
u_data = zeros(size(t_data)); 
% convert to timeseries OBJ
u = timeseries(u_data, t_data); % is a [WORKSPACE INPUT]


% STABLE EQUILIBRIUM [PUNTO 2]
figure(1)
subplot(1, 2, 1) % position DX
hold on
% interessante utilizzo del ciclo for, prende input dagli array senza step
for i = [-pi/6 pi/6]
    for j = [-3 0 3]
        x0 = [i; j]; u_star + u_sin
        SimulationOutput = sim('p54_pendulum_sim.slx', 'SaveOutput', 'on');
        theta_signal = SimulationOutput.theta;
        omega_signal = SimulationOutput.omega;
        theta_val    = theta_signal.Data(:);
        omega_val    = omega_signal.Data(:);
        plot(theta_val, omega_val, LineWidth=plot_line_width)
    end
end
grid on
box on
xlim([-pi/3 pi/3])
ylim([-4 4])
set(gcf,'position',[plot_x0,plot_y0,plot_width,plot_height])
title('Stable equilibrium', FontSize=plot_font_size)
xlabel('$\theta$ [rad]', FontSize=plot_font_size, Interpreter='latex')
ylabel('$\dot{\theta}$ [rad/s]', FontSize=plot_font_size, Interpreter='latex')


% UNSTABLE EQIULIBRIUM
subplot(1, 2, 2) % position SX
hold on
for i = pi + [-pi/12 0 pi/12]
    for j = [-0.5 0 0.5]
        x0 = [i; j];
        SimulationOutput = sim('p54_pendulum_sim.slx', 'SaveOutput', 'on');
        theta_signal = SimulationOutput.theta;
        omega_signal = SimulationOutput.omega;
        theta_val    = theta_signal.Data(:);
        omega_val    = omega_signal.Data(:);
        plot(theta_val, omega_val, LineWidth=plot_line_width)
    end
end
grid on
box on
xlim(pi + [-pi/6 pi/6])
ylim([-1 1])
set(gcf,'position',[plot_x0,plot_y0,plot_width,plot_height])
title('Unstable equilibrium', FontSize=plot_font_size)
xlabel('$\theta$ [rad]', FontSize=plot_font_size, Interpreter='latex')
ylabel('$\dot{\theta}$ [rad/s]', FontSize=plot_font_size, Interpreter='latex')



%% Verifico stabilità locale nel punto di equilibeio theta_star [pi/3, 0]
% vettore da 0 a Tf dostanziato di  0.0010
t_data = linspace(0, Tf, 10000);
% I have c_star as 1.2744 [M*g*l*sin(theta_star)]
% u_star è la serie di ingressi costante quindi creo 10k 
% moltiplicando C_star 10k volte, moltiplicando per uni
% a distanza di tempo t_data
u_star = C_star*ones(size(t_data)); 
% non mi riesco a spiegare u_sin
u_sin  = 0.03*sin(2*t_data);
% come ingresso uso la somma u_star + u_sin al singolo istante dei tempo e
% la converto in una timeseries 
u      = timeseries(u_star + u_sin, t_data); % is a [WORKSPACE INPUT]
x0     = [pi/3; 0]; % is a [WORKSPACE INPUT]


SimulationOutput = sim('p54_pendulum_sim.slx', 'SaveOutput', 'on');
dtheta_signal     = SimulationOutput.dtheta;
time              = dtheta_signal.Time(:);
dtheta_data       = dtheta_signal.Data(:);
dtheta_lin_signal = SimulationOutput.dtheta_lin;
dtheta_lin_data   = dtheta_lin_signal.Data(:);
figure(2)
plot(time, dtheta_data, LineWidth=plot_line_width)
hold on
plot(time, dtheta_lin_data, LineWidth=plot_line_width)
grid on
box on
xlabel('$t$ [s]', FontSize=plot_font_size, Interpreter='latex')
ylabel('output [rad]', FontSize=plot_font_size, Interpreter='latex')
legend('$\theta - \theta^\star$ (nonlinear system)', '$\delta\theta$ (linearized model)', ...
       FontSize=plot_font_size, Interpreter='latex')
title('Nonlinear system vs. linearized model', FontSize=plot_font_size)