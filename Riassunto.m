%{
[LINKS]
https://osulp.github.io/git-advanced/06_integrate_matlab_and_git/index.html
p57 => https://it.mathworks.com/help/ident/ug/canonical-state-space-realizations.html

---------------------------------------------------------------------------

[Block000]

%% Init

p9 per avere info sugli operatori e caratteri speciali
help ops 

p10 matrici
p12 operazioni con matrici
p18 informazioni e manipolazione matrici
p20 matrici e indici
p21 algebra lineare
p22 sistemi lineari

p11 clear, format decimals, print methods

p15 funzioni aritmetiche
p17 numeri complessi
p19 autogenerazione matrici, array, vettori, random

p24 figure e grafici

---------------------------------------------------------------------------

[Block001]

%% Script e funzioni

p30 operatori logici
p31 controllo di flusso switch, if, fow, while

EXERCISES
p34 [M,S] = max_of_2_numbers_and_sum(a,b)
    [o] = scalar_product(vec1,vec2)
p35 plot_figures()
---------------------------------------------------------------------------

[Block002]

%% Simulazione di sistemi lineari tempo-invarianti

p37 LTI_risposta_libera_e_forzata
p39 Sistema massa molla smorzatore

p46 DC_DC_converter_buck

---------------------------------------------------------------------------

[Block003]

%% Introduzione a Simulink

p51 Simulatore SYS massa-molla-smorzatore

p54 Simulatore SYS pendolo

---------------------------------------------------------------------------

[Block004]

p56 visualizzazione luogo delle radici
p57 controllore R nello Spazio degli Stati (ss)
    G, R, rlocus, risposta al gradino con treshold
    sintesi del controllore, con forme:
    - di CONTROLLABILITA', - MODALE

p59 come agire su Ta, S%, 

p60 pendulum_control e realizzo del regolatore per sentimento
p60 Linearized system around theta_star = 2pi/3
p61 Linearized system around theta_star = pi/3

p62_inverted_pendulum_control

p63_MarginiStabilita_FdS
p69_Taratura_PID

p70_DC [big]

---------------------------------------------------------------------------

[Block005]

---------------------------------------------------------------------------

[Block006]

---------------------------------------------------------------------------

[Block007]


%}