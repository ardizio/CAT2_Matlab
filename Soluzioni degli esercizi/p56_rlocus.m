%% Clear Workspace
clear;
clc;
close all; 
%% Parametri del sistema
s = tf('s');
G = 1/(s+1);
% sono entrambe funzioni continue nel tempo

% s G fosse complessa potrei migliorare la stabilità numerica usand minreal
%G = minreal(G);

%% Luogo delle radici (analisi gradica di poli e zeri)
% Opzione 1
figure(1);
rlocus(G);
% dal Luogo delle Radici get: [ r: radici, k: guadagni ]
[r, k] = rlocus(G);

grid on

% Opzione 2
figure(2)
% traccia il luogo delle radici
h = rlocusplot(minreal(G)); 
% wants as input a plot and stores all its data!!
p = getoptions(h); 

% setto la x limitando la vista tra [-3, 0]
p.XLim = {[-3, 0]}
setoptions(h,p)
grid on





