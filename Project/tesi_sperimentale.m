%% Clear Setting
clear 
clc
close all
%% General settings
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
%{
PROBLEM 1: Stabilizzare intorno all'equilibrio
PROBLEM 2: T_ae < 1,2 sec
PROBLEM 3: Discretizzazione
EXTRA 4: Stabilizzare il carrello al centro della rotaia 
         Sfruttare il controllo in cascata
%}
%% Parametri del carrello su pendolo inverso
p_M = 0.61;          % massa del carrello [kg]
p_ma = 0.116;        % massa asta [kg]
p_mb = 0.05;         % massa posta al termine dell'asta [kg]
p_l = 0.45;          % lunghezza dell'asta [m]
p_k = 0;             % rigidità della molla [N/m]
p_b = 0.1;           % coefficiente di smorzamento [Ns/m]
p_g = 9.81;          % accellerazione di gravità [m/s^2]
p_Ja = (1/12)*p_ma*p_l^2;% Inertia p_ma
p_Jb = 0;            % Inertia p_mb
init_v0 = 0;         % to SLX
init_x0 = 0;         % to SLX
init_omega0 = 0;     % to SLX
init_theta0= pi/12;  % to SLX
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
Matrix_C_theta_lin = [0 1 0 0];
%Legame algebrico ingresso–uscita:  D = []
Matrix_D_theta_lin = 0;
%Volendo stabilizzare anche il carrello avremo una seconda uscita y = x_1
Matrix_C_pos_lin = [1 0 0 0];
%Legame algebrico ingresso–uscita:  D = []
Matrix_D_pos_lin = 0;

%convert dynamic system models to state-space model form.
LTI_theta_lin = ss(Matrix_A_lin, Matrix_B_lin, Matrix_C_theta_lin, Matrix_D_theta_lin);
LTI_position_lin = ss(Matrix_A_lin, Matrix_B_lin, Matrix_C_pos_lin, Matrix_D_pos_lin);



%% EXTRACT DATA FROM Tranfer Function
% G_ang: FUNZIONE DI TRASFERIMENTO angolare 
G_ang = tf(LTI_theta_lin);
% check: FISICA REALIZZABILITA'
[FdT_orderNumerator, FdT_orderDenominator] = getFunctionOrdersNumDen(G_ang);
% Display the orders
if FdT_orderNumerator <= FdT_orderDenominator
    disp('G_ang è FISICAMENTE REALIZZABILE');
end
% check: STABILITA'
FdT_isStable = isstable(G_ang);
if FdT_isStable
    disp('G_ang Stabilità: STABLE');
else
    disp('G_ang Stabilità: UNSTABLE');
end


% Fattorizzazione ZPK della FdT - MINIMIZZATA
%G_ang = minreal(zpk(G_ang));





%{
% realization or pole-zero cancellation
fprintf('\nMINREAL (realization or pole-zero):');

% Posizione Poli
fprintf('\n G_ang POLES:');
poles = pole(G_ang_minreal);
disp(poles);
% Posizione Zeri
fprintf('\n G_ang ZEROS :');
zeros = zero(G_ang_minreal);
disp(zeros);
%}



% G_ang rlocus [OPEN LOOP]
Request = ["Request", "rlocus", " [G_ang OPEN LOOP] Luogo delle radici "];
Options = ["Grid_on", "Box_on", "edit_xlabel", "edit_ylabel", "edit_legend"];
plotWithCustomOptions(Request, G_ang, Options)
%% Bode over G_ang
%Demo Regolatore
R_ang = -(s+0.4)*(s+4.5)/s/(s+13);
%Gain
K = 23;
% Gain * Regolatore
R_ang = K * R_ang;
% Minreal pole-zero
G_e_ang = minreal(G_ang * R_ang);
% G_e_ang rlocus [CLOSED LOOP]
Request = ["Request", "rlocus", " [G_e_ang CLOSED LOOP] Luogo delle radici"];
Options = ["Grid_on", "Box_off", "edit_xlabel", "edit_ylabel", "edit_legend"];
plotWithCustomOptions(Request, G_e_ang, Options)

%{
Request = ["Request", "step", " [G_e_ang] Step response"];
plotWithCustomOptions(Request, G_e_ang, Options)
%}



%% Sentitivity functions 
L_ang = minreal(G_ang * R_ang);

% Funzione di Sensitività [S]
S = minreal(1/(1+L_ang));
% Funzione di Sensitività complementare [F]
F = minreal(L_ang/(1+L_ang));
% Funzione di Sensitività del controllo [Q]
Q = minreal(R_ang/(1+L_ang));


Request = ["Request", "step", " [F] risposta al gradino"];
plotWithCustomOptions(Request, F, Options)

Request = ["Request", "rlocus", " [F] rlocus"];
plotWithCustomOptions(Request, minreal(F), Options)


% STUDIO DELLA STABILITA' ROBUSTA del sistema in retroazione
Request = ["Request", "blank", " [G_e_ang] bode"];
plotWithCustomOptions(Request, 0, Options)
[mag, phase, wout] = bode(G_e_ang);     % Assign the plot data to variables
[~, pm, ~, gm] = margin(G_e_ang);
setoptions = bodeoptions;               % Get the default plot options
setoptions.Grid = 'on';                 % Turn on the grid
bode(G_e_ang, setoptions);              % Plot the Bode plot with custom options
hold on;
% Display phase margin and gain margin on the Bode plot
text(0.5, -180+pm, sprintf('Pm = %.2f deg', pm), 'Color', 'red');
text(0.5/gm, 0, sprintf('Gm = %.2f dB', 20*log10(gm)), 'Color', 'blue');


%% Goals
%{
% Requirements
mindecay   = 1.2;
mindamping = 0.7;
maxfreq    = inf;
Goals      = TuningGoal.Poles(mindecay, mindamping, maxfreq);
Request = ["Request", "blank", " [F] Show Goals"];
plotWithCustomOptions(Request, F, Options)
viewGoal(Goals, F);
%}

%{


% PASSARE IN RISPOSTA IN FREQUENZA


%risposta al grdino di G
figure(1);
step(minreal(G_lin));

figure(2)
rlocus(G_lin);

[r, p_k] = rlocus(G_lin);


plotWithCustomOptions()

disp ('Transfer function of the linearized model (Angle):')
zpk(G_lin)

% Trova M_fase
% Trova omega_c Pulsazione di attraversamento.


% Controlla la sensitività


%Antipiatrice
% Bode
%G_pos_Out




p_k=29
R_ang = -(s+0.4)*(s+4.5)/s/(s+12);




%% Controller 


%FdT posizione
G_lin_pos = tf(LTI_position_lin);
disp ('Transfer function of the linearized model (Cart Position):')
zpk(G_lin_pos);

%Inseriamo il meno poichè è impossibile stabilizzarlo per il LUOGO DIRETTO
%{
%R = (s+4)(s+4.5)/s/(s+8); -> causa di oscillazioni

        -(s+0.4)*(s+4.5)
R_ang = ----------------
             s(s+9)
%}

%Regolatore ANGOLARE
R_ang = -(s+0.4)*(s+4.5)/s/(s+9);
%Gain per entrare nella zona di Re>0
K = 21;
R_ang = K * R_ang;
%Sistema esteso
G_e_ang = minreal(G_lin_ang*R_ang);
rlocus(G_e_ang);
grid on;


%% Tuning Rete anticipatrice
%Trovo lo smorzamento per una sovraelongazione minore di 10
delta_star = 0.52834; % per una sovraelong max di 10
delta = 0.55;
Mf = delta*100;

[M_a,P,W]=bode(G_e_ang);
[V,i]=min(abs(W-100));
GeWcd=M_a(i);
ArgGeWcd=P(i);
Pd=-180+60-ArgGeWcd;
Md=1/GeWcd;
Pd=Pd*pi/180; 



tau=(Md-cos(Pd))/(100*sin(Pd));
alpha=(cos(Pd)-1/Md)/(100*sin(Pd))/tau; 

%Prova 

R_ant = (1+tau*s)/(1+alpha*tau*s);

R = minreal(R_ang*R_ant);
%% Controllore in the space state sistem

R_lti   = ss(R);

A_ctrl  = R_lti.A;
B_ctrl  = R_lti.B;
C_ctrl  = R_lti.C;
D_ctrl  = R_lti.D;

%% Sentitivity functions 
L_ang = minreal(G_lin_ang*R);

S = minreal(1/(1+L_ang));
F = minreal(L_ang/(1+L_ang));
Q = minreal(R_ang/(1+L_ang));

%Tempo di assestamento massimo (entro il 5%) = 1.2s


% checking the requirements
% mindecay   = 2.5;
% mindamping = 0;
% maxfreq    = inf;
% Goals      = TuningGoal.Poles(mindecay, mindamping, maxfreq);
% figure(2)
% viewGoal(Goals, F);


%%
%{



[k_d, k_p, k_d] = coef_pid(1,2,3)

R_pid_s = (k_p*s + k_d*s + k_i)/s; %Equazione PID
G_s = ()


G_cl_s = (R_pid_s * G_s) / (1 + (R_pid_s * G_s))


% devo scegliere i 3 poli in anello chiusp

 INPULSE(G_LIN_ANG)
MINREAL
%}

%}


%% END Segment
% Create a figure
if plot_figure_Index > 1
    fig = figure;

    % Create a button uicontrol
    btn = uicontrol('Style', 'pushbutton', 'String', 'Chiudi Tutto', ...
        'Position', [20 20 100 30], 'Callback', @clear_plots);
    
    set(gcf,'position',[800,500,140,50])
end
function clear_plots(~, ~)
    close all
end