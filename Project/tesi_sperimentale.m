%% Clear Setting

clear 
clc
close all
%% General settings
plot_enable_Display = 1;
plot_figure_Index = 1;
% CHANGE To FIT DEVICE SCREEN
plot_device_maxColumns  = 3;
plot_device_maxRows  = 2;
plot_current_Column  = 1;
plot_current_Row = 1;
% 
plot_style_fontSize   = 18;
plot_style_lineWidth  = 2;
% GET device ScreenSize variables
plot_screen_Size = get(groot, 'ScreenSize');
plot_screen_Width = plot_screen_Size(3);
plot_screen_Height = plot_screen_Size(4);
% SET plot width & height
plot_window_Width = plot_screen_Width/plot_device_maxColumns;
plot_window_Height  = 460;
% DEFINE plot positions in Screen
plot_position_from_left = 0;
plot_position_from_bottom = plot_screen_Height;

%% 
% PROBLEM 1: Stabilizzare intorno all'equilibrio
% PROBLEM 2: T_a5 <= 1,2 sec
% PROBLEM 3: Discretizzazione
% EXTRA 4: Stabilizzare il carrello al centro della rotaia 
%          Sfruttare il controllo in cascata

%% Parametri del carrello su pendolo inverso [p_xxx stands as parameter_xxxx]

p_M = 0.61;          % massa del carrello [kg]
p_ma = 0.116;        % massa asta [kg]
p_mb = 0.05;         % massa posta al termine dell'asta [kg]
p_l = 0.45;          % lunghezza dell'asta [m]
p_k = 0;             % rigidità della molla [N/m]
p_b = 0.1;           % coefficiente di smorzamento [Ns/m]
p_g = 9.81;          % accellerazione di gravità [m/s^2]
p_Ja = (1/12)*p_ma*p_l^2;% Inertia p_ma
p_Jb = 0;            % Inertia p_mb
% Initial Parameters
init_v0 = 0;              % to SLX
init_x0 = -5;             % to SLX
init_omega0 = 0;          % to SLX
init_theta0 = pi/12;      % to SLX
init_thetaRef = 0;        % to SLX
init_positionRef = 0;     % to SLX
Ts = 0.002;               % to SLX
%% Transfer function variable
s = tf('s');
%% Simulink settings
MaxStep  = 1e-2;    % solver max step size
RelTol   = 1e-2;    % solver relative tolerance
simulation_T_f= 10; % final simulation time
%% Linearizated model

% states: x_1: linear position
%         x_2: angular position
%         x_3: linear speed
%         x_4: angular speed
% MATRICE di Masse
Matrix_Masse_lin = [(p_ma/2+p_mb)*p_l   (p_ma/4+p_mb)*p_l^2+p_Ja+p_Jb;
                        p_M+p_ma+p_mb         (p_ma/2+p_mb)*p_l       ];
% MATRICE Dinamica:  A = []
Matrix_A_1 = [0 0 1 0;
              0 0 0 1];
Matrix_A_2 = (Matrix_Masse_lin^-1) * [0      (p_ma/2+p_mb)*p_g*p_l  0    0;
                                      -p_k           0             -p_b  0];
Matrix_A_lin = [Matrix_A_1;
                Matrix_A_2];
% MATRICE di distribuizioine degli ingressi:  B = []
Matrix_B_1 = [0;
              0];
Matrix_B_2 = (Matrix_Masse_lin^-1) * [0;
                                      1];
Matrix_B_lin=[Matrix_B_1;
              Matrix_B_2];
% MATRICE di distribuzione delle uscite:  C = []
Matrix_C_angle_lin = [0 1 0 0];
%Legame algebrico ingresso–uscita:  D = []
Matrix_D_angle_lin = 0;
%Volendo stabilizzare anche il carrello avremo una seconda uscita y = x_1
Matrix_C_pos_lin = [1 0 0 0];
%Legame algebrico ingresso–uscita:  D = []
Matrix_D_pos_lin = 0;

%convert dynamic system models to state-space model form.
LTI_angle_lin = ss(Matrix_A_lin, Matrix_B_lin, Matrix_C_angle_lin, Matrix_D_angle_lin);
LTI_position_lin = ss(Matrix_A_lin, Matrix_B_lin, Matrix_C_pos_lin, Matrix_D_pos_lin);


%% ANALISI IN CATENA APERTA

%% Extract data from Tranfer Function

% angolare - POLI e ZERI 
% f_G_angle: FUNZIONE DI TRASFERIMENTO angolare 
f_G_angle = tf(LTI_angle_lin);
f_G_angle = minreal(zpk(f_G_angle));
% Posizione Poli
% fprintf('\n f_G_angle POLI:');
f_G_angle_Poles = pole(f_G_angle);
% disp(f_G_angle_Poles);
% Posizione Zeri
% fprintf('\n f_G_angle ZERI :');
f_G_angle_Zeros = zero(f_G_angle);
% disp(f_G_angle_Zeros);

% posizionale - POLI e ZERI
% f_G_position: FUNZIONE DI TRASFERIMENTO posizionale
f_G_position = tf(LTI_position_lin);
f_G_position = minreal(zpk(f_G_position))
% Posizione Poli
% fprintf('\n f_G_angle POLI:');
f_G_position_Poles = pole(f_G_position);
% disp(f_G_position_Poles);
% Posizione Zeri
% fprintf('\n f_G_angle ZERI :');
f_G_position_Zeros = zero(f_G_position);
% disp(f_G_position_Zeros);

% NOTA: denominatori di f_G_angle e f_G_position hanno gli stessi zeri,
% ma hanno poli differenti al numeratore. 
% f_G_angle ha un polo nell'origine.

%                     zeri (Z)
% FdT = guadagno (K) ----------
%                     poli (P)


%% Analisi Risposte all impulso e al gradino


% %  f_G_angle
% plot_f_Request = ["Request", "impulse", " [f_G_angle] Risposta impulso"];
% plot_f_Options = ["Grid_on", "Box_on", "edit_xlabel", "edit_ylabel", "edit_legend"];
% displayPlot(plot_f_Request, f_G_angle, plot_f_Options)
% plot_f_Request = ["Request", "stepplot", " [f_G_angle] Risposta gradino"];
% plot_f_Options = ["Grid_on", "Box_on", "edit_xlabel", "edit_ylabel", "edit_legend"];
% displayPlot(plot_f_Request, f_G_angle, plot_f_Options)
% % f_G_position 
% plot_f_Request = ["Request", "impulse", " [f_G_position] Risposta impulso"];
% plot_f_Options = ["Grid_on", "Box_on", "edit_xlabel", "edit_ylabel", "edit_legend"];
% displayPlot(plot_f_Request, f_G_position, plot_f_Options)
% plot_f_Request = ["Request", "stepplot", " [f_G_position] Risposta gradino"];
% plot_f_Options = ["Grid_on", "Box_on", "edit_xlabel", "edit_ylabel", "edit_legend"];
% displayPlot(plot_f_Request, f_G_position, plot_f_Options)

% % NOTA: f_G_angle è instabile sia al gradino, sia all'impulso.
% NOTA: f_G_position è insoddisfacente sia al gradino, sia all'impulso.
% NOTA: è necessario lo sviluppo di un controllore che per prima cosa 
% stabilizzi il sistema poi migliori le prestazioni.



%% PROGETTO DEL CONTROLLORE

% Obiettivi di controllo
% Tempo di Assestamento: T_a5 <= 1.2 secondi
% Fare in modo che il sistema sia in grado di annullare un disturbo d(t) in
% ingresso all'angolo theta

% Sistema è BIBO, 2 uscite e 1 ingresso. (NOT SISO)

% Sviluppiamo un controllore che mantenga l'asta del carrello nella
% posizione di equilibrio instabile, e successivamente un controllore in
% cascata per stabilizzare anche la posizione del carrello. Sarà necessario
% uno studio mediante il luogo delle radici per garantire BIBO stabilità
% CLOSED LOOP [C_ang] per poi progettare un secondo controllore [C_pos]
% inserito in catena diretta al nuovo sitema

% Luogo delle radici negativo, non basta aggiungere un k proporzionale, non
% stabilizza il sistema. E' necessario eliminare lo zero nell'originee
% richiamare il reamo che parte dal semipiano positivo.

%% Controllo dell'Angolo

% f_K_angle_Gain = 28;
% f_R_angle = -(s+0.1289)*(s+5.5)/s/(s+12);

f_K_angle_Gain = 1000;
f_R_angle = -(s+1)*(s+3)/s/(s+100);

% minreal(zpk(f_R_angle))

f_KR_angle = f_K_angle_Gain * f_R_angle;

% Analisi del sistema di controllo progettato 

% Funzione d'Anello
f_L_angle = minreal(f_KR_angle * f_G_angle);

% Funzione di Sensitività [f_Sensitivity_S_angle]
f_Sensitivity_S_angle = minreal(1/(1+f_L_angle));
% Funzione di Sensitività complementare [f_Sensitivity_F_angle]
f_Sensitivity_F_angle = minreal(f_L_angle/(1+f_L_angle));
% Funzione di Sensitività del controllo [f_Sensitivity_Q_angle]
f_Sensitivity_Q_angle = minreal(f_KR_angle/(1+f_L_angle));

% f_T_angle
f_T_angle = minreal(f_G_angle/(1+f_L_angle));

% % Proprietà del sistema stabilizzato
% plot_f_Request = ["Request", "rlocusplot", " [f_L_angle] rlocus"];
% plot_f_Options = ["Grid_on", "Box_off", "edit_xlabel", "edit_ylabel", "edit_legend"];
% displayPlot(plot_f_Request, f_Sensitivity_F_angle, plot_f_Options)
% pzplot(f_G_angle, f_Sensitivity_F_angle, f_L_angle)
% % MOSTRO
% plot_f_Request = ["Request", "step", " Risposta CLOSED LOOP"];
% plot_f_Options = ["Grid_on", "Box_off", "edit_xlabel", "edit_ylabel", "edit_legend"];
% displayPlot(plot_f_Request, f_Sensitivity_F_angle, plot_f_Options)
% 
% plot_f_Request = ["Request", "step", " Risposta Sistema T"];
% plot_f_Options = ["Grid_on", "Box_off", "edit_xlabel", "edit_ylabel", "edit_legend"];
% displayPlot(plot_f_Request, f_T_angle, plot_f_Options)

f_KR_angle_LTI = ss(f_KR_angle);
A_CTRL_angle  = f_KR_angle_LTI.A;
B_CTRL_angle  = f_KR_angle_LTI.B;
C_CTRL_angle  = f_KR_angle_LTI.C;
D_CTRL_angle  = f_KR_angle_LTI.D;

% algoritmo per la discretizzazione
alpha_angle = 0.5; % Discretizzazione tramite Tustin
I_angle     = eye(size(A_CTRL_angle, 1)); % matrice identità
% matrici del regolatore a tempo discreto
A_alpha_angle = I_angle + Ts * (I_angle - alpha_angle * A_CTRL_angle * Ts)^-1 * A_CTRL_angle;
B_alpha_1_angle = (1 - alpha_angle) * Ts * (I_angle - alpha_angle * A_CTRL_angle * Ts)^-1 * B_CTRL_angle;
B_alpha_2_angle = alpha_angle * Ts * (I_angle - alpha_angle * A_CTRL_angle * Ts)^-1 * B_CTRL_angle;


%% Controllo della posizione

f_K_position_Gain = 300;
f_R_position = -(s+0.5)*(s+0.5)/s/(s+100);

plot_f_Request = ["Request", "rlocus", " [f_G_position] rlocus"];
plot_f_Options = ["Grid_on", "Box_off", "edit_xlabel", "edit_ylabel", "edit_legend"];
displayPlot(plot_f_Request, f_G_position, plot_f_Options)

minreal(zpk(f_R_position))

f_KR_position = f_K_position_Gain * f_R_position;


% Analisi del sistema di controllo progettato 

% Funzione d'Anello
f_L_position = minreal(f_KR_position * f_G_position);

plot_f_Request = ["Request", "rlocus", " [f_L_position] rlocus"];
plot_f_Options = ["Grid_on", "Box_off", "edit_xlabel", "edit_ylabel", "edit_legend"];
displayPlot(plot_f_Request, f_L_position, plot_f_Options)


% Funzione di Sensitività [f_Sensitivity_S_position]
f_Sensitivity_S_position = minreal(1/(1+f_L_position));
% Funzione di Sensitività complementare [f_Sensitivity_F_position]
f_Sensitivity_F_position = minreal(f_L_position/(1+f_L_position));
% Funzione di Sensitività del controllo [f_Sensitivity_Q_position]
f_Sensitivity_Q_position = minreal(f_KR_position/(1+f_L_position));

% f_T_position
f_T_position = minreal(f_G_position/(1+f_L_position));

% rlocus L(s)
plot_f_Request = ["Request", "rlocus", " [f_L_angle] rlocus"];
plot_f_Options = ["Grid_on", "Box_off", "edit_xlabel", "edit_ylabel", "edit_legend"];
displayPlot(plot_f_Request, f_Sensitivity_F_position, plot_f_Options)
% pzplot(f_G_position, f_Sensitivity_F_position, f_L_position)
% Risposta CLOSED LOOP F(s)
plot_f_Request = ["Request", "step", " Risposta CLOSED LOOP [f_Sensitivity_F_position]"];
plot_f_Options = ["Grid_on", "Box_off", "edit_xlabel", "edit_ylabel", "edit_legend"];
displayPlot(plot_f_Request, f_Sensitivity_F_position, plot_f_Options)
% Risposta a; gradino T(s)
plot_f_Request = ["Request", "step", " Risposta Sistema [f_T_position]"];
plot_f_Options = ["Grid_on", "Box_off", "edit_xlabel", "edit_ylabel", "edit_legend"];
displayPlot(plot_f_Request, f_T_position, plot_f_Options)


f_KR_position_LTI = ss(f_KR_position);
A_CTRL_position = f_KR_position_LTI.A;
B_CTRL_position = f_KR_position_LTI.B;
C_CTRL_position = f_KR_position_LTI.C;
D_CTRL_position = f_KR_position_LTI.D;



%% END Segment
% Create a figure
if plot_figure_Index > 1
    plot_closeAll = figure;
    % Create a button uicontrol
    plot_closeAll_button = uicontrol('Style', 'pushbutton', 'String', 'Close all Plots', ...
        'Position', [20 20 100 30], 'Callback', @clear_plots);
    % position in screen and dimensions
    set(gcf,'position',[800,500,140,50])
end
function clear_plots(~, ~)
    close all
end




%% Analisi del sistema di controllo progettato


% % MOSTRO: Margini con Bode di f_L_angle {margin(f_L_angle))}
% plot_f_Request = ["Request", "margin", " margin di f_L_angle"];
% plot_f_Options = ["Grid_on", "Box_off", "edit_xlabel", "edit_ylabel", "edit_legend"];
% displayPlot(plot_f_Request, f_L_angle, plot_f_Options)
% % MOSTRO: risposta al gradino di f_G_e_angle
% plot_f_Request = ["Request", "step", " Risposta al gradino del sistema stabilizzato"];
% plot_f_Options = ["Grid_on", "Box_off", "tempo [s]", "posizione [rad]", "edit_legend"];
% displayPlot(plot_f_Request, f_G_e_angle, plot_f_Options)
% [y, t] = step(f_G_e_angle); % salva in y la risposta e in t il tempo


% 
% plot_f_Request = ["Request", "step", " [Sensitivity_F] risposta al gradino"];
% displayPlot(plot_f_Request, f_Sensitivity_F_angle, plot_f_Options)

% plot_f_Request = ["Request", "rlocusplot", " [Sensitivity_F] rlocus"];
% displayPlot(plot_f_Request, f_Sensitivity_F_angle, plot_f_Options)
% Tracciamento dei poli nel piano complesso
% pzplot(f_G_angle_ZPK, f_Sensitivity_F_angle, f_G_e_angle)


% 
% % STUDIO DELLA STABILITA' ROBUSTA del sistema in retroazione
% plot_f_Request = ["Request", "blank", " [G_e_angle] bode"];
% displayPlot(plot_f_Request, 0, plot_f_Options)
% [mag, phase, wout] = bode(f_G_e_angle);     % Assign the plot data to variables
% [~, pm, ~, gm] = margin(f_G_e_angle);
% setoptions = bodeoptions;               % Get the default plot options
% setoptions.Grid = 'on';                 % Turn on the grid
% bode(f_G_e_angle, setoptions);              % Plot the Bode plot with custom options
% hold on;
% % Display phase margin and gain margin on the Bode plot
% text(0.5, -180+pm, sprintf('Pm = %.2f deg', pm), 'Color', 'red');
% text(0.5/gm, 0, sprintf('Gm = %.2f dB', 20*log10(gm)), 'Color', 'blue');

% Goals
%{
% Requirements
mindecay   = 1.2;
mindamping = 0.7;
maxfreq    = inf;
Goals      = TuningGoal.Poles(mindecay, mindamping, maxfreq);
plot_f_Request = ["Request", "blank", " [f_Sensitivity_F_angle] Show Goals"];
displayPlot(plot_f_Request, f_Sensitivity_F_angle, plot_f_Options)
viewGoal(Goals, f_Sensitivity_F_angle);
%}
