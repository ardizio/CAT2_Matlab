%% Startup functions 
clear
close all
clc
%% General settings
plot_font_size   = 18;
plot_line_width  = 2;
plot_x0     = 500;
plot_y0     = 300;
plot_width  = 600;
plot_height = 400;
%% System parameters
k = 1;     % costante elasica [N/m] (stiffness coefficient)
M = 0.01;  % massa [kg]
b = 0.02;  % coefficiente di atrito viscoso [Ns/m] (damping coefficient)
%% Simulation settings
tf = 10;     % tempo finale [s] used Simulink "Stop time" (final sim. time)
x0 = [1; 0]; % initial conditions
% egnero gli inressi alle uscite di tempo terminati da un inout a sinusoide
[u_data, t_data] = gensig('sine', 2*pi/1, tf, 1e-2); % input sinusoid
u = timeseries(u_data, t_data); % convert to timeseries
%% Simulation
SimulationOutput = sim('p51_mass_spring_damper_sim.slx', 'SaveOutput', 'on');
%% Plotting the data
x_signal = SimulationOutput.x;
plot(x_signal)

% extracting data from the timeseries x
t_sim = x_signal.Time;
x_1   = x_signal.Data(:, 1);
x_2   = x_signal.Data(:, 2);

%{

% plotting the input force vs the output position
figure(1)
plot(t_data, u_data, LineWidth=plot_line_width)
set(gcf,'position',[plot_x0,plot_y0,plot_width,plot_height])
hold on
plot(t_sim, x_1, LineWidth=plot_line_width)
grid on
box on
xlabel('$t$ [s]', FontSize=plot_font_size, Interpreter='latex')
legend('$u(t)$ [N]', '$x_1(t)$ [m]', FontSize=plot_font_size, Interpreter='latex')

% plotting the orbit in the phase diagram
figure(2)
plot(x_1, x_2, LineWidth=plot_line_width)
set(gcf,'position',[plot_x0,plot_y0,plot_width,plot_height])
grid on
box on
xlabel('$x_1(t)$ [m]', FontSize=plot_font_size, Interpreter='latex')
ylabel('$x_2(t)$ [m/s]', FontSize=plot_font_size, Interpreter='latex')

%}