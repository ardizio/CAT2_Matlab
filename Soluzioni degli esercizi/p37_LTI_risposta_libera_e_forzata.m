
%% Clear Workspace
clear      %cancella workspace
close all  %chiude le figure
clc        %cancella la riga di comando
%% Matrici del sistema
A = 0;
B = 1;
C = 1;
D = 0;
%% Creo l'oggetto Sistema con la funzione "ss()"
lti_model = ss(A, B, C, D);

%% Get simulated response data: "lsim()"

%% Evoluzione libera:
% - ingresso nullo
% - condizione iniziale non nulla

% genero intervallo temporale
t = 0:0.1:10;
% setto l'ingresso temporalmente nullo == 0  per ogni t
u_free = zeros(length(t), 1);
% condizione iniziale != 0
x0_free = 5;

% simulo la risposta libera
[y_free, t_free, x_free] = lsim(lti_model, u_free, t, x0_free);
fig1 = figure(1);
set(fig1, 'Name', 'Evoluzione libera');
plot(t_free, x_free);

%% Evoluzione forzata:
% - ingresso non nullo
% - condizione iniziale nulla

% genero intervallo temporale
t = 0:0.1:10;
% setto l'ingresso temporalmente nullo == 0  per ogni t
u_forced = ones(length(t), 1);
% condizione iniziale != 0
x0_forced = 0;

% simulo la risposta libera
[y_forced, t_forced, x_forced] = lsim(lti_model, u_forced, t, x0_forced);

fig2 = figure(2);
set(fig2, 'Name', 'Evoluzione forzata');
plot(t_free, x_free);
plot(t_forced, x_forced);