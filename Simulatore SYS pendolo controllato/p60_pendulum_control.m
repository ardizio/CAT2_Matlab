%% Startup functions
clear
close all
clc
%% General settings
plot_font_size   = 18;
plot_line_width  = 2;
plot_x0     = 500;
plot_y0     = 300;
plot_width  = 800;
plot_height = 400;
%% Transfer function variable
s = tf('s');
%% Simulink Settings
MaxStep  = 1e-2; % solver max step size
RelTol   = 1e-2; % solver relative tolerance
%% System parameters
l = 0.3;   % lunghezza asta del pendolo [m] (pendulum length)
M = 0.5;   % massa [kg]
b = 0.05;  % coefficiente di smorzamento [Nms/rad] (damping coefficient)
g = 9.81;  % accellerazione di gravità [m/s^2] (gravity acceleration)



%% PARTE 1 - Linearized system around theta_star = 2pi/3

% punto di equilibrio imposto
theta_star_u = 2*pi/3;

% calcolo Coppia Forzante
C_star_u   = M*g*l*sin(theta_star_u);

% Matrici del sistema tenenti conto "theta_star_u"
A_lin_u    = [0                                            1;
              -M*g*l*cos(theta_star_u)/(M*l^2)   -b/(M*l^2)];
B_lin_u    = [0; 1/(M*l^2)];
C_lin_u    = [1 0];
D_lin_u    = 0;

% mostro gli autovalori della matrice A
disp('Eigenvalues of the linearized system (unstable eq.):')
disp(eig(A_lin_u))


% creo la variabile sistema lineare
lin_lti_u   = ss(A_lin_u, B_lin_u, C_lin_u, D_lin_u);
% ricavo la FdT
G_lin_u     = minreal(tf(lin_lti_u));




% PLOT Sistema
figure(1)
set(gcf,'position',[plot_x0,plot_y0,plot_width,plot_height])
% Mostro il luogo delle radici
subplot(1, 2, 1)
% Luogo delle radici
h = rlocusplot(G_lin_u);
% Adjust X axes
p = getoptions(h);
p.XLim = {[-10, 4]};
setoptions(h, p);
grid on

% VARIABILE K di CONTROLLO
% R_u passata al Gain del Workspace
R_u = 0.75;

%Funzione di Sensitività
F_u  = minreal(R_u*G_lin_u/(1 + R_u*G_lin_u)); 

subplot(1, 2, 2)
% Osservo la risposta al gradino
step(F_u)
grid on
legend('$k = 0.75$', FontSize=plot_font_size, Interpreter='latex')
stepinfo(F_u, SettlingTimeThreshold=0.05)

% no state space realization is necessary, the controller is a static gain
%% Testing the controller

% Setto il tempo di simulazione per Simulink
Tf_1     = 20;   
% Definisco gli step di tempo
t_data   = linspace(0, Tf_1, 10000);
u_star_u = C_star_u*ones(size(t_data));
% Converto gli ingressi in Timeseries
u_1      = timeseries(u_star_u, t_data); 



figure(2)
hold on
% NOTA: ogni valore nella [] è un ciclo for diversi
for i = 2*pi/3 + [-1 0.5 0 0.5 1]/180*pi
    for j = [-1 0 1]/180*pi
        x0_1 = [i; j];
        SimOut = sim('pendulum_ctrl_sim_1.slx', 'SaveOutput', 'on');
        theta_signal = SimOut.theta;
        omega_signal = SimOut.omega;
        theta_val    = theta_signal.Data(:);
        omega_val    = omega_signal.Data(:);
        plot(theta_val, omega_val, LineWidth=plot_line_width)
    end
end
grid on
box on
% definisco I LIMITI (dx,sx) della x
xlim(2/3*pi + [-1 1]/180*pi)
% definisco I LIMITI (dx,sx) della y
ylim([-1 1]/180*pi)
% definisco testo asse x
xlabel('$\theta$ [rad]', FontSize=plot_font_size, Interpreter='latex')
% definisco testo asse y
ylabel('$\dot{\theta}$ [rad/s]', FontSize=plot_font_size, Interpreter='latex')
% definisco titolo del plot
title('Controlled system with $\theta^\star = 2\pi/3$', FontSize=plot_font_size, Interpreter='latex')



%% PARTE 2 - Linearized system around theta_star = pi/3

% punto di equilibrio imposto
theta_star_s = pi/3;

% calcolo Coppia Forzante
C_star_s   = M*g*l*sin(theta_star_s);

% Matrici del sistema tenenti conto "theta_star_s"
A_lin_s    = [0                                      1;
              -M*g*l*cos(theta_star_s)/(M*l^2) -b/(M*l^2)];
B_lin_s    = [0; 1/(M*l^2)];
C_lin_s    = [1 0];
D_lin_s    = 0;




% mostro gli autovalori della matrice A
disp('Eigenvalues of the linearized system (stable eq.):')
disp(eig(A_lin_s))

% creo la variabile sistema lineare
lin_lti_s   = ss(A_lin_s, B_lin_s, C_lin_s, D_lin_s);

% ricavo la FdT
G_lin_s     = minreal(tf(lin_lti_s));



% VARIABILE K di CONTROLLO
% R_s passata al Gain del Workspace
% control design with root locus
% inizialemnte è un azzeccarci, bisogna tararla finchè non funziona
R_s = (s^2 +2*0.75*4*s + 4^2)/s/(0.05*s+1);



figure(3)
set(gcf,'position',[plot_x0,plot_y0,plot_width,plot_height])
% Parte 1 del plot
% Luogo delle radici del regolatore + sitema
subplot(1, 2, 1)
% Luogo delle radici L(s) = R(s)*G(s)
h = rlocusplot(minreal(R_s*G_lin_s));
p = getoptions(h);
p.XLim = {[-10, 0]};
setoptions(h, p);
grid on


% Parte 2 del plot
% Risposta al gradino di F(s)
R_s = 0.55*R_s;
F_s = minreal(R_s*G_lin_s/(1 + R_s*G_lin_s));
subplot(1, 2, 2)
step(F_s)
grid on
legend('$k = 0.55$', FontSize=plot_font_size, Interpreter='latex')
stepinfo(F_s, SettlingTimeThreshold=0.05)



% Realizzazione nello Spazio degliStati ss
% del Sistema con controllo
R_s_lti = ss(R_s);
A_ctrl  = R_s_lti.A;
B_ctrl  = R_s_lti.B;
C_ctrl  = R_s_lti.C;
D_ctrl  = R_s_lti.D;


%% Testing the controller

% final simulation time
Tf_2     = 2;   
t_data   = linspace(0, Tf_2, 10000);


% Setto gli ingressi
%Ingresso "u_star_s"
u_star_s = C_star_u*ones(size(t_data));
% timeseries per Workspace
u_2      = timeseries(u_star_s, t_data);
% ingresso di zeri costanti
w_data   = zeros(size(t_data));
% timeseries per Workspace
w        = timeseries(w_data, t_data); 

x0_2     = [0; 0];


% Simulazione
SimOut = sim('pendulum_ctrl_sim_2.slx', 'SaveOutput', 'on');
% Output simulazione
theta_signal     = SimOut.theta;
theta_ref_signal = SimOut.theta_ref;
time             = theta_signal.Time(:);
theta_data       = theta_signal.Data(:);
theta_ref_data   = theta_ref_signal.Data(:);



figure(4)
plot(time, theta_data, LineWidth=plot_line_width)
hold on
plot(time, theta_ref_data, LineWidth=plot_line_width)
grid on
box on
xlabel('$t$ [s]', FontSize=plot_font_size, Interpreter='latex')
ylabel('output [rad]', FontSize=plot_font_size, Interpreter='latex')
legend('$\theta$', '$\theta^\star + w$', ...
       FontSize=plot_font_size, Interpreter='latex')
title('Controlled system response, $\theta^\star = \pi/3$', FontSize=plot_font_size, Interpreter='latex')