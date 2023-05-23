%% Clear Workspace
clear; clc; close all;  
%% Plot inizializzazione
font_size=18; line_width=2; plot_x0=500; plot_y0=300; plot_width=600; plot_height=600;
%% Parametri del sitema
k = 1;     % costante elastica [k = 1N/m]
M = 0.01;  % massa [M = 0.01kg]
b = 0.02;  % coeff attrito viscoso [b = 0.02 N*s/m]
%% Sim: LTI [A,B,C,D]
% ricavo A,B,C,D dal modello dello spazio degli stati
A = [  0      1   ; 
      -k/M  -b/M ];
B = [  0 ; 
      1/M ];
C = [ 1  0 ];
D = 0;

% passo le matrici al modello LTI
lti_model = ss(A, B, C, D);

%% OBIETTIVO 1:
%{
realizzare uno script che gereri l'evoluzione
dello stato del sistema per  0 <= t <= 4,
con condizione iniziale x0 = [0; 1] 
e con input sinusoidale avente pulsazione iniziale 10 rad/s

si confrontino u(t), y(t)
%}

% [x0] condizione iniziale
x0 = [ 0 ; 1 ];
% [w_10] pulsazione 10 rad/s
w_10  = 10;  

%da pulsazione devo generare il peziodo [2 * pi / pulsazione]
% [T_10] periodo
T_10  = 2*pi/w_10;

%[ingresso_ad_istante_t, istante_t] = gensig('sin', periodo , tempo_max);
% associamo un ingresso sinusoidale ad uno specifico periodo t_10
% generazione di segnali sinusoidale:
[u_10, t_10] = gensig('sin', T_10, 4); 

%output [y_sim , t_sim, x_sim] chiedono risultati dalla lsim()
[y_sim , t_sim, x_sim] = lsim(lti_model, u_10, t_10, x0);

% Plot
fig1 = figure(1);
set(fig1, 'Name', 'Simulazione del sistema A,B,C,D');
%set figure relatve to screen
set(gcf,'position',[plot_x0,plot_y0,plot_width,plot_height])

%top graph
ax(1) = subplot(2, 1, 1);
% DISPLAYING: t_sim, x_sim
plot(t_sim, x_sim, LineWidth=line_width)
grid on
box on
xlabel('$t$ [s]', FontSize=font_size, Interpreter='latex')
legend('$x_1(t)$ [m]', '$x_2(t)$ [m/s]', FontSize=font_size, Interpreter='latex')

%bottom graph
ax(2) = subplot(2, 1, 2);
% DISPLAYING: t_sim, y_sim
plot(t_sim, y_sim, LineWidth=line_width)
hold on
% DISPLAYING: t_10, u_10
plot(t_10, u_10, LineWidth=line_width)
grid on
box on
xlabel('$t$ [s]', FontSize=font_size, Interpreter='latex')
legend('$y(t)$ [m]', '$u(t)$ [N]', FontSize=font_size, Interpreter='latex')
hold off
linkaxes(ax,'x');



%% OBIETTIVO 2:
%{
dato il sistema LTI SISO a tempo continuo
trovare la corrispondenet Fdt G(s) = N(s)/D(s) -> CAT1

definizione tramite numeratore e denominatore
G = tf(num, den);
%}

% START: METODI EQUIVALENTI
% >>> version 1:
% conversione di un sistema nello spazio degli stati:
%G = tf(lti_model);

% >>> version 2;
% modo piu' veloce per scrivere le FdT:
% creiamo una variabile speciale, la 's' della funzione di trasferimento a
% tempo continuo. Utilizziamo una funzione al posto di enunciare una serie
% di coefficienti polinomiali
s = tf('s'); 
G = (1/M)/(s^2 + b/M*s + k/M);
% END: METODI EQUIVALENTI

%% Operazioni su G(s)

% risposta impulsiva
% impulse(G);

% risposta al gradino
% step(G);

% risposta a input generici
% lsim(G, u, t);

% elenco degli zeri
% zero(G)

% elenco dei poli
% pole(G)

% poli e zeri
% pzmap(G)

%% OBIETTIVO 3:
%{
aggiornare lo script precedente usando la FdT
per calcolare la risposta forzata dell'uscita ai 
seguenti ingressi usando gensig:
[a] sinusoide di ampiezza 1N e pulsazioni in rad/s w_2_dot_5 = 2.5
[b] sinusoide di ampiezza 1N e pulsazioni in rad/s w_10 = 10
[c] sinusoide di ampiezza 1N e pulsazioni in rad/s w_20 = 20
[d] onda quadra di ampiezza 1N e periodo p = 2 sec
%}

%sinusoide di test1 (2.5 rad/s)
w_2p5  = 2.5; 
T_2p5  = 2*pi/w_2p5;  % periodo
[u_2p5, t_2p5] = gensig('sin', T_2p5, 10); %generazione di segnale sinusoidale

%sinusoide di test2 (10 rad/s)
w_10  = 10; 
T_10  = 2*pi/w_10;   % periodo
[u_10, t_10] = gensig('sin', T_10, 10); %generazione di segnale sinusoidale

%sinusoide di test3 (20 rad/s)
w_20  = 20; 
T_20  = 2*pi/w_20;   % periodo
[u_20, t_20] = gensig('sin', T_20, 10); %generazione di segnale sinusoidale

% test4 generazione di segnale a gradino, dove:
[u_square, t_square] = gensig('square', 5, 10, 0.01);


[y_2p5,  t_2p5] = lsim(G, u_2p5, t_2p5);
[y_10 ,   t_10] = lsim(G,  u_10,  t_10);
[y_20 ,   t_20] = lsim(G,  u_20,  t_20);
[y_square , t_square] = lsim(G,  u_square,  t_square);

% Plot: G(s)
fig2 = figure(2);
% title bar name plot
set(fig2, 'Name', 'Funzione di trasferimento');
% position figure in window
set(gcf,'position',[plot_x0,plot_y0,plot_width,plot_height])

%Grafico superiore
ax(1) = subplot(2, 1, 1);
% test1 (2.5 rad/s)
plot(t_2p5, y_2p5, LineWidth=line_width)
hold on
% test2 (10 rad/s)
plot(t_10, y_10, LineWidth=line_width)
% test3 (20 rad/s)
plot(t_20, y_20, LineWidth=line_width)
% Max line
yline(1, LineWidth=line_width)
% Bottom line
yline(-1, LineWidth=line_width)
grid on
box on
xlabel('$t$ [s]', FontSize=font_size, Interpreter='latex')
legend('$y_{2.5}(t)$ [m]', '$y_{10}(t)$ [m]', '$y_{20}(t)$ [m]',...
    FontSize=font_size, Interpreter='latex')
hold off

%Grafico inferiore
ax(2) = subplot(2, 1, 2);
% line1 sinusoidal [y] uscita sinusoidale al gradino
plot(t_square, y_square, LineWidth=line_width)
hold on
% line2 squared [u_square] ingresso al gradino
plot(t_square, u_square, LineWidth=line_width)
grid on
box on
xlabel('$t$ [s]', FontSize=font_size, Interpreter='latex')
legend('$y(t)$ [m]', '$u(t)$ [N]', FontSize=font_size,...
    Interpreter='latex', Location='SouthEast')
hold off