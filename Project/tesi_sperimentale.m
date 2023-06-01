%% Clear Setting
clear 
clc
close all

blank = 0;
%% General settings
plot_figureIndex = 1;
% CHANGE To FIT DEVICE SCREEN
plot_MaxColumns  = 3;
plot_max_rows  = 2;
plot_CurrentColumn  = 1;
plot_CurrentRow = 1;
% 
plot_font_size   = 18;
plot_line_width  = 2;
% GET device ScreenSize variables
screen_Size = get(groot, 'ScreenSize');
screen_Width = screen_Size(3);
screen_Height = screen_Size(4);
% SET plot width & height
plot_width = screen_Width/plot_MaxColumns;
plot_height  = 460;
% DEFINE plot positions in Screen
plot_from_left = 0;
plot_from_bottom = screen_Height;
%%
%{
PROBLEM 1: Stabilizzare intorno all'equilibrio
PROBLEM 2: T_ae < 1,2 sec
PROBLEM 3: Discretizzazione
EXTRA 4: Stabilizzare il carrello al centro della rotaia 
         Sfruttare il controllo in cascata
%}
%% Parametri del carrello su pendolo inverso
M = 0.61;          % massa del carrello [kg]
ma = 0.116;        % massa asta [kg]
mb = 0.05;         % massa posta al termine dell'asta [kg]
l = 0.45;          % lunghezza dell'asta [m]
k = 0;             % rigidità della molla [N/m]
b = 0.1;           % coefficiente di smorzamento [Ns/m]
g = 9.81;          % accellerazione di gravità [m/s^2]
Ja = (1/12)*ma*l^2;% Inertia ma
Jb = 0;            % Inertia mb
v0 = 0;            % to SLX
x0 = 0;            % to SLX
omega0 = 0;        % to SLX
theta0= pi/12;     % to SLX
%% Transfer function variable
s = tf('s');
%% Simulink settings
MaxStep  = 1e-2;  % solver max step size
RelTol   = 1e-2;  % solver relative tolerance
T_f      = 10;    % final simulation time
%% Linearizated model
% states: x_1: linear position
%         x_2: angular position
%         x_3: linear speed
%         x_4: angular speed
% MATRICE di Masse
M_lin = [(ma/2+mb)*l (ma/4+mb)*l^2+Ja+Jb;
           M+ma+mb      (ma/2+mb)*l     ];
% MATRICE Dinamica:  A = []
A_1 = [0 0 1 0;
       0 0 0 1];
A_2 = (M_lin^-1) * [0  (ma/2+mb)*g*l  0  0;
                    -k      0        -b  0];
A_lin = [A_1;
         A_2];
% MATRICE di distribuizioine degli ingressi:  B = []
B1 = [0;
      0];
B2 = (M_lin^-1) * [0;
                   1];
B_lin=[B1;
       B2];
% MATRICE di distribuzione delle uscite:  C = []
C_lin_theta = [0 1 0 0];
%Legame algebrico ingresso–uscita:  D = []
D_lin_theta = 0;
%Volendo stabilizzare anche il carrello avremo una seconda uscita y = x_1
C_lin_pos = [1 0 0 0];
%Legame algebrico ingresso–uscita:  D = []
D_lin_pos = 0;

%convert dynamic system models to state-space model form.
LTI_lin_theta = ss(A_lin, B_lin, C_lin_theta, D_lin_theta);
LTI_lin_position = ss(A_lin, B_lin, C_lin_pos, D_lin_pos);


%% EXTRACT DATA FROM Tranfer Function
% G_ang: FUNZIONE DI TRASFERIMENTO angolare 
G_ang = tf(LTI_lin_theta)
% Fattorizzazione ZPK della FdT - MINIMIZZATA
%G_ang = minreal(zpk(G_ang));

% check: FISICA REALIZZABILITA'
[order_num, order_den] = getFunctionOrdersNumDen(G_ang);

% Display the orders
disp(['Order of numerator: ' num2str(order_num)]);
disp(['Order of denominator: ' num2str(order_den)]);

% check: STABILITA'
is_G_ang_Stable = isstable(G_ang);
if is_G_ang_Stable
    fprintf('\nG_ang Stability: STABLE\n\n');
else
    fprintf('\nG_ang Stability: UNSTABLE\n\n');
end





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
plotWithCustomOptions(Request, blank, Options)
[mag, phase, wout] = bode(G_e_ang);     % Assign the plot data to variables
[~, pm, ~, gm] = margin(G_e_ang)
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

[r, k] = rlocus(G_lin);


plotWithCustomOptions()

disp ('Transfer function of the linearized model (Angle):')
zpk(G_lin)

% Trova M_fase
% Trova omega_c Pulsazione di attraversamento.


% Controlla la sensitività


%Antipiatrice
% Bode
%G_pos_Out




k=29
R_ang = -(s+0.4)*(s+4.5)/s/(s+12);




%% Controller 


%FdT posizione
G_lin_pos = tf(LTI_lin_position);
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
if plot_figureIndex > 1
    fig = figure;

    % Create a button uicontrol
    btn = uicontrol('Style', 'pushbutton', 'String', 'Chiudi Tutto', ...
        'Position', [20 20 100 30], 'Callback', @clear_plots);
    
    set(gcf,'position',[800,500,140,50])
end
function clear_plots(~, ~)
    close all
end