%%LTI_a_tempo_continuo
%% Clear Workspace
clear
close all
clc
%% Plot inizializzazione
font_size   = 18;
line_width  = 2;
plot_x0     = 500;
plot_y0     = 300;
plot_width  = 800;
plot_height = 600;
%% Parametri del sitema
Vi  = 10;    %
L   = 1e-4;  % <-- use AeB to denote A*10^B
C   = 1e-4;  %consensatore
R   = 2;     %resistenza
%% Linear system and transfer function definition
A_dcdc = [ 0 -1/L ; 1/C -1/R/C ];
B_dcdc = [ 1/L; 0 ];
C_dcdc = [ 0 1 ];
D_dcdc =  0 ;

% Inizializzazione modello LTI
dcdc_sytem = ss(A_dcdc, B_dcdc, C_dcdc, D_dcdc);
% Inizializzazione FdT (Funzione di Trasferimento)
G_dcdc     = tf(dcdc_sytem);

% Metodo alternativo calcolo FdT
s = tf('s');
G_dcdc2    = 1/L/C/(s^2 + s/R/C + 1/L/C);



%% Segnali di ingresso & Sim
% Segnali di ingresso
f_PWM  = 20e3;
T_PWM  = 1/f_PWM;
Ts     = T_PWM/100;
tf     = 100*T_PWM - Ts;
[u, t] = gensig('square', T_PWM, tf, Ts);
u      = Vi*u;
% Trasformata di Fourier
N            = length(t);           %numero di campioni
F            = fft(u);              %Fourier
FT2          = abs(F/N);
FT1          = FT2(1:N/2 + 1);
FT1(2:end-1) = 2*FT1(2:end-1);      %calcolo dello spettro
f            = (1/Ts)*((0:(N/2))/N);%creazione dei valori di frequenza

%output [y_sim , t_sim] chiedono risultati dalla lsim() LTI
[y_sim,  t_sim] = lsim(G_dcdc, u, t);


% Output
fig1 = figure(1);
set(fig1, 'Name', 'FdT G(s)');
set(gcf,'position',[plot_x0,plot_y0,plot_width,plot_height])
%top graph
ax(1) = subplot(3, 1, 1);
plot(t, u, LineWidth=line_width)
grid on
box on
xlabel('$t$ [s]', FontSize=font_size, Interpreter='latex')
ylabel('$V_D$ [V]', FontSize=font_size, Interpreter='latex')
%center graph
ax(2) = subplot(3, 1, 2);
plot(t_sim, y_sim, LineWidth=line_width)
grid on
hold on
box on
yline(Vi/2, LineWidth=line_width)
xlabel('$t$ [s]', FontSize=font_size, Interpreter='latex')
ylabel('$V_o$ [V]', FontSize=font_size, Interpreter='latex')
linkaxes(ax,'x');
%bottom graph
subplot(3, 1, 3);
plot(f, FT1, LineWidth=line_width)
grid on
box on
xlabel('$f$ [Hz]', FontSize=font_size, Interpreter='latex')
ylabel('$|\mathcal{F}[V_D]|$ [V]', FontSize=font_size, Interpreter='latex')
xlim([0, 2e5])




%% Diagramma di Bode
W = logspace(3, 6, 10000);     %pulsazioni tra 10^-2 e 10^3 (100 punti)
[mag, phase] = bode(G_dcdc, W);%array di dimensione 1x1xlength(W)
Magdb = 20*log10(mag(:));      %grafico decibel
Phase = phase(:);              %grafico fasi

% Output
fig2 = figure(2);
set(fig2, 'Name', 'Diagramma di Bode');
set(gcf,'position',[plot_x0,plot_y0,plot_width,plot_height])

%top graph
ax(1) = subplot(3, 1, 1);
Mag_plot = semilogx(W, Magdb, LineWidth=line_width);
grid on
hold on
box on
xline(2*pi*f_PWM, LineWidth=line_width)
xlabel('$\omega$ [rad/s]', Interpreter='latex', FontSize=font_size)
ylabel('$|G(j\omega)|_{dB}$', Interpreter='latex', FontSize=font_size)
Mag_plot.DataTipTemplate.DataTipRows(1).Label = "\omega = ";
Mag_plot.DataTipTemplate.DataTipRows(2).Label = "|G(j\omega)|_{dB} = ";
datatip(Mag_plot, 2*pi*f_PWM, 20*log10(abs(freqresp(G_dcdc, 2*pi*f_PWM))),...
    Location="northeast", FontSize=font_size*0.7, SnapToDataVertex="off");
hold off
%center graph
ax(2) = subplot(3, 1, 2);
Ph_plot = semilogx(W, Phase, LineWidth=line_width);
grid on
hold on
box on
xline(2*pi*f_PWM, LineWidth=line_width)
xlabel('$\omega$ [rad/s]', Interpreter='latex', FontSize=font_size)
ylabel('$\arg(G(j\omega))$', Interpreter='latex', FontSize=font_size)
Ph_plot.DataTipTemplate.DataTipRows(1).Label = "\omega = ";
Ph_plot.DataTipTemplate.DataTipRows(2).Label = "arg(G(j\omega)) = ";
datatip(Ph_plot, 2*pi*f_PWM, rad2deg(angle(freqresp(G_dcdc, 2*pi*f_PWM))),...
    Location="northeast", FontSize=font_size*0.7, SnapToDataVertex="off");
hold off
%bottom graph
ax(3) = subplot(3, 1, 3);
semilogx(2*pi*f, FT1, LineWidth=line_width)
grid on
box on
xlabel('$\omega$ [rad/s]', FontSize=font_size, Interpreter='latex')
ylabel('$|\mathcal{F}[V_D]|$ [V]', FontSize=font_size, Interpreter='latex')
xlim([1e3, 1e6])
linkaxes(ax,'x');