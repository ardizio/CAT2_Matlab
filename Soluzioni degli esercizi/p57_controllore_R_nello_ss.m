%% Clear Workspace
clear;
clc;
close all;  
%% Parametri del sitema
s = tf('s');
% FdT dell'impianto
G = 1/(s + 1);
% PI in cancellazione
R = (s + 1)/s;
%% Parametri del sitema
figure(1)
rlocus(minreal(R*G));
%ispeziono il luogo per trovare k
%R = 15*R  TDB
figure(2);
%risposta al gradino
step(minreal(R*G/(1+R*G)));
stepinfo(minreal(R*G/(1+R*G)), SettlingTimeThreshold=0.05);
%% Sintesi del controllore

%R_lti = ss(R);
lti = ss(R);
% R è una FdT
R = tf(lti);

%Sistema nello spazio degli stati
R_lti1 = ss(R);
ctrl_A = R_lti1.A;
ctrl_B = R_lti1.B;
ctrl_C = R_lti1.C;
ctrl_D = R_lti1.D;

% Forma 'Compagna' di CONTROLLABILITA'
R_lti2 =  canon(R, 'companion');


% Forma MODALE
R_lti3 =  canon(R, 'modal');

% ricaviamo modelli matriciali differenti
% R_lti1
% R_lti2
% R_lti3