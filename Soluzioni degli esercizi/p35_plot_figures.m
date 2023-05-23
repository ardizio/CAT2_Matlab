function p35_plot_figures()
%GENERATE_CHARTS generate_4_charts(5,0.5,0.2)    Hz = 1/s
%% Startup functions
clear
close all
clc
%% General settings
font_size   = 18;
line_width  = 2;
plot_x0     = 500;
plot_y0     = 300;
plot_width  = 600;
plot_height = 400;
%% Parameters
t0        = 0;
tf        = 10;
N_samples = 1000;
a   = 5;
f   = 0.5;
phi = 0.2;
%% Generating data
% time
t  = linspace(t0, tf, N_samples);
% signals
y1 = exp(-t);
y2 = a*sin(2*pi*f*t + phi);
y3 = a*t.*sin(2*pi*f*t + phi);
y4 = a*t.^2.*sin(2*pi*f*t + phi).*exp(-t);
%% Plotting data
% For all labels, this code uses the Latex interpreter:
% $$ is the delimiter for the math environment
%    use it for variables, not for measurement units!
% common commands in the math environment:
% {}^{}: superscript
% {}_{}: subscript
% \frac{}{}: fraction
% \alpha, \beta etc.: greek letters
% \sin, \cos, etc.: common functions
% 
% You can search on the internet for other commands, symbols, and examples
%% Version 1:
% individual plots

% CASE1: exp(-t)
fig = figure;  %generate new window
plot(t, y1, LineWidth=line_width)   %append (x,y, linea)
grid on    %griglia interna
box on
% label su x
xlabel('$t$ [s]', FontSize=font_size, Interpreter='latex')
% label su y
ylabel('$y(t)$', FontSize=font_size, Interpreter='latex')
% inserisco la legenda
legend('$e^{-t}$', FontSize=font_size, Interpreter='latex')
% definisco le grandezze
set(gcf,'position',[plot_x0,plot_y0,plot_width,plot_height])
% cambio il nome alla finestra
set(fig, 'Name', 'exp(-t)');

% CASE2: a(sin(2*pi*f*t + phi))
fig2 = figure(2);
plot(t, y2, LineWidth=line_width)
grid on
box on
xlabel('$t$ [s]', FontSize=font_size, Interpreter='latex')
ylabel('$y(t)$', FontSize=font_size, Interpreter='latex')
legend('$a \sin(2\pi f t + \varphi)$', ...
        FontSize=font_size, Interpreter='latex')
set(gcf,'position',[plot_x0,plot_y0,plot_width,plot_height])
set(fig2, 'Name', 'a*sin(2*pi*f*t + phi)');

% CASE3: a(t(sin(2*pi*f*t + phi)))
fig3 = figure(3);
plot(t, y3, LineWidth=line_width)
grid on
box on
xlabel('$t$ [s]', FontSize=font_size, Interpreter='latex')
ylabel('$y(t)$', FontSize=font_size, Interpreter='latex')
legend('$a t \sin(2\pi f t + \varphi)$', ...
        FontSize=font_size, Interpreter='latex', Location='NorthWest')
set(gcf,'position',[plot_x0,plot_y0,plot_width,plot_height])
set(fig3, 'Name', 'a*t.*sin(2*pi*f*t + phi)');

% CASE4: a(t^2(sin(2*pi*f*t + phi)))*exp(-t)
fig4 = figure(4);
plot(t, y4, LineWidth=line_width)
grid on
box on
xlabel('$t$ [s]', FontSize=font_size, Interpreter='latex')
ylabel('$y(t)$', FontSize=font_size, Interpreter='latex')
legend('$a t^2 \sin(2\pi f t + \varphi) e^{-t}$', ...
        FontSize=font_size, Interpreter='latex')
set(gcf,'position',[plot_x0,plot_y0,plot_width,plot_height])
set(fig4, 'Name', 'a*t.^2.*sin(2*pi*f*t + phi).*exp(-t)');

%% Version 2: 
% all plots in the same figure with different colors
fig5 = figure(5);
% tutti i grafici nella stessa figura, inserisco il primo e faccio tener
% duro inserendo poi gli altri.
plot(t, y1, LineWidth=line_width)
hold on
plot(t, y2, LineWidth=line_width)
plot(t, y3, LineWidth=line_width)
plot(t, y4, LineWidth=line_width)
grid on
box on
xlabel('$t$ [s]', FontSize=font_size, Interpreter='latex')
ylabel('$y(t)$', FontSize=font_size, Interpreter='latex')
% una legenda molto lunga
legend('$e^{-t}$',...
       '$a \sin(2\pi f t + \varphi)$',...
       '$a t \sin(2\pi f t + \varphi)$',...
       '$a t^2 \sin(2\pi f t + \varphi) e^{-t}$', ...
       FontSize=font_size, Interpreter='latex', Location='SouthWest')
hold off
set(gcf,'position',[plot_x0,plot_y0,plot_width,plot_height])
set(fig5, 'Name', 'AIO Plots different colors');
%% Version 3: 
% figure with four subplots
fig6 = figure(6);
set(gcf,'position',...
    [plot_x0,plot_y0 - plot_height*0.5,plot_width,plot_height*1.5])

ax(1) = subplot(4, 1, 1);
plot(t, y1, LineWidth=1.5)
grid on
box on
ylabel('$y(t)$', FontSize=font_size, Interpreter='latex')
legend('$e^{-t}$', FontSize=font_size, Interpreter='latex')

ax(2) = subplot(4, 1, 2);
plot(t, y2, LineWidth=line_width)
grid on
box on
ylabel('$y(t)$', FontSize=font_size, Interpreter='latex')
legend('$a \sin(2\pi f t + \varphi)$', ...
        FontSize=font_size, Interpreter='latex')

ax(3) = subplot(4, 1, 3);
plot(t, y3, LineWidth=line_width)
grid on
box on
ylabel('$y(t)$', FontSize=font_size, Interpreter='latex')
legend('$a t \sin(2\pi f t + \varphi)$', ...
        FontSize=font_size, Interpreter='latex')

ax(4) = subplot(4, 1, 4);
plot(t, y4, LineWidth=line_width)
grid on
box on
xlabel('$t$ [s]', FontSize=font_size, Interpreter='latex')
ylabel('$y(t)$', FontSize=font_size, Interpreter='latex')
legend('$a t^2 \sin(2\pi f t + \varphi) e^{-t}$', ...
        FontSize=font_size, Interpreter='latex')
set(fig6, 'Name', 'AIO different plots');

linkaxes(ax,'x'); % <-- use this to navigate along x simultaneously
end