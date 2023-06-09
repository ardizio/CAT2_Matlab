%% Funzioni di inizializzazione
clear      % cancella le variabili nel workspace
close all  % chiude tutte le figure
clc        % pulisce la riga di comando
%% Impostazioni per le figure
font_size   = 15;   % dimensione dei font
marker_size = 12;   % dimensione dei marker
line_width  = 2;    % dimensione delle linee
plot_x0     = 500;  % coordinata x delle figure
plot_y0     = 300;  % coordinata y delle figure
plot_width  = 600;  % larghezza delle figure
plot_height = 400;  % altezza delle figure
%% Variabile per definire funzioni di trasferimento
s = tf('s');   % vedere il blocco 2 delle slide per ulteriori dettagli
%% Impostazioni per Simulink
% IMPORTANTE: ricordarsi di inserire queste variabili nel simulatore
% Parametri del solver: Modeling --> Model Settings --> Solver
MaxStep  = 1e-3; % massimo step di simulazione
                 % in genere, prendere questo parametro almeno 10 volte più
                 % piccolo della costante di tempo più veloce da simulare
RelTol   = 1e-3; % tolleranza relativa del solver
% Tempo di simulazione: Stop Time in alto a destra della finestra
t_f      = 15;   % scegliere un valore tale da catturare tutta la risposta
%% Parametri del sistema
l = 0.3;   % lunghezza del pendolo [m]
M = 0.5;   % massa [kg]
b = 0.1;   % coefficiente d'attrito viscoso [Nms/rad]
g = 9.81;  % accelerazione gravitazionale [m/s^2]
%% Linearizzazione del sistema
% calcolo della coppia di equilibrio
theta_star = 2*pi/3;                %  posizione di equilibrio
C_star     = M*g*l*sin(theta_star); % coppia per imporre l'equilibrio
% modello linearizzato nello spazio degli stati
A       = [                            0           1;
           -M*g*l*cos(theta_star)/(M*l^2) -b/(M*l^2)];
B       = [0; 1/(M*l^2)];
C       = [1 0];
D       = 0;
lti_sys = ss(A, B, C, D);
% funzione di trasferimento
G       = tf(lti_sys);


%% Come esempio, consideriamo un'architettura a due anelli
% loop interno: stabilizzazione tramite luogo delle radici
% loop esterno: imposizione delle specifiche tramite Bode
%% Loop interno: stabilizzazione tramite luogo delle radici

% % generazione di una figura con le impostazioni desiderate
% figure(1)
% set(1,'position',[plot_x0,plot_y0,plot_width,plot_height])
% % LUOGO DELLE RADICI di [ G ]
% rlocusplot(G)
% grid on
% title('Luogo delle radici', 'FontSize', font_size)
% % codice per migliorare la leggibilità della figura
% h = findobj(1, 'Type', 'Line');
% set(h, 'MarkerSize', marker_size, 'LineWidth', line_width)


% TROVATO IL GUADAGNO STABILIZZANTE da rlocus
k_inner = 0.79;            % k scelto per ottenere due poli reali
G_s     = minreal(k_inner*G/(1 + k_inner*G))


%% Proprietà del sistema stabilizzato


% MOSTRO: diagramma di Bode di G_s
figure(2)
set(2,'position',[plot_x0,plot_y0,plot_width,plot_height])
bodeplot(G_s)
grid on
h = findobj(2, 'Type', 'Line');
set(h, 'LineWidth', line_width)


% MOSTRO: risposta al gradino
figure(3)
set(3,'position',[plot_x0,plot_y0,plot_width,plot_height])
[y, t] = step(G_s); % salva in y la risposta e in t il tempo
plot(t, y, 'LineWidth', line_width)
grid on
xlim([0, t(end)])
xlabel('tempo [s]', 'FontSize', font_size)
ylabel('posizione [rad]', 'FontSize', font_size)
title('Risposta al gradino del sistema stabilizzato', 'FontSize', font_size)



%% Specifiche da imporre tramite i diagrammi di Bode
% esempio di specifiche
% 1) stabilità robusta
%    - margine di fase superiore a 70° per garantire robustezza
Mf_r    = 70; % atttraversamento 0 dB sopra, prendo sotto phase deg
% 2) precisione statica
%    - errore a regime nullo --> necessità di un polo nell'origine
% 3) precisione dinamica
%   - sovraelongazione inferiore al 10%
% in questo caso, prendiamo il caso conservativo sul margine di fase
Mf_star = max(Mf_r, 60);
%   - tempo di assestamento all'1% inferiore a 10s
T_star  = 8;
Wc_min  = 460/T_star/Mf_star;  %?????
% 4) inserire qui eventuali specifiche sull'attenuazione dei disturbi
% 5) specifiche sul regolatore
%    - il regolatore deve essere fisicamente realizzabile
%    - è necessario garantire moderazione dell'azione di controllo


%% Scelta del regolatore (Scela di dove mettere i poli)
% elementi strutturali:
% - un polo nell'origine
% - due zeri in cancellazione
% - un polo di fisica realizzabilità
R = 0.06*(s/1.281 + 1)*(s/0.9408 + 1)/s/(s/3.493 + 1);
disp('Regolatore:')
zpk(R)



%% Analisi del sistema di controllo progettato
L = R*G_s;     % funzione d'anello
S = 1/(1 + L); % funzione di sensitività
F = L/(1 + L); % funzione di sensitività complementare
Q = R/(1 + L); % funzione di sensitività del controllo



%% Tracciamento dei poli nel piano complesso
figure(4)
set(4,'position',[plot_x0,plot_y0,plot_width,plot_height])
% tracciamento di poli e zeri del sistema
pzplot(G, minreal(F), G_s)
grid on
ylim([-10 10])
h = findobj(4, 'Type', 'Line');
set(h, 'MarkerSize', marker_size, 'LineWidth', line_width)
title('Posizione di poli e zeri', 'FontSize', font_size)
legend('Impianto', 'Outer Loop', 'Inner Loop', 'FontSize', font_size)



%% VERIFICA DEL REGOLATORE PROGETTATO
%% verifica della specifica sul margine di fase
figure(2);
hold on
margin(L)
h = findobj(2, 'Type', 'Line');
set(h, 'LineWidth', line_width)


%% verifica delle specifiche sulla risposta al gradino
figure(5)
set(5,'position',[plot_x0,plot_y0,plot_width,plot_height])
[y, t] = step(F, 6);
plot(t, y, 'LineWidth', line_width)
grid on
xlim([0, 6])
xlabel('tempo [s]', 'FontSize', font_size)
ylabel('posizione [rad]', 'FontSize', font_size)
hold on
yline(1, ':', 'Color', [0.5 0.5 0.5], 'LineWidth', line_width)
yline(1.01, '-.', 'Color', [0.5 0.5 0.5], 'LineWidth', line_width)
yline(0.99, '-.', 'Color', [0.5 0.5 0.5], 'LineWidth', line_width)
title('Risposta al gradino del sistema controllato', 'FontSize', font_size)
disp('Proprietà della risposta con controllore a tempo continuo')
stepinfo(F, 'SettlingTimeThreshold', 0.01)

%% Realizzazione del regolatore digitale
Ts = 0.03;
% realizzazione di R nello spazio degli stati
R_lti     = ss(R); % FdT --> sistema nello spazio degli stati
A_ctrl    = R_lti.A; B_ctrl = R_lti.B;
C_ctrl    = R_lti.C; D_ctrl = R_lti.D;
% algoritmo per la discretizzazione
alpha     = 0.5; % Discretizzazione tramite Tustin
I         = eye(size(A_ctrl, 1)); % matrice identità
% matrici del regolatore a tempo discreto
A_alpha   = I + Ts*(I - alpha*A_ctrl*Ts)^-1*A_ctrl;
B_alpha_1 = (1 - alpha)*Ts*(I - alpha*A_ctrl*Ts)^-1*B_ctrl;
B_alpha_2 = alpha*Ts*(I - alpha*A_ctrl*Ts)^-1*B_ctrl;