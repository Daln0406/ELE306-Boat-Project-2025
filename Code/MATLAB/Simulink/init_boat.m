
%% --- Velg modus ---
useSnake = true;    % true = snake-mode, false = lattice-mode


% Båtparametre
b        = 0.30;      % avstand propeller
Dv       = 0.6;       % surge-demping
Dr       = 0.8;       % yaw-demping
K_T      = 12;        % thrust-gain
betaT    = 0.25;      % thrust linear/quad mix (0..1)
deadband = 0.05;      % aktuator dødgang
tau_m    = 0.10;      % motorlag [s]

Ts        = 0.02;   % diskret sampletid for kontroller
Tstop     = 2280;     % simtid

% Kommandoskala (v,omega -> uL,uR)
vmax     = 1.2;  omegamax = 4.0;
kvu      = 1.0 / vmax;
kou      = 1.6 / omegamax;

% Strøm 
v_cx = 0.00;  v_cy = 0.10;  % 0.1 m/s langs +y



% --- Planlegging ---
if useSnake
    %% === SLANGE-DEKNING ===
    disp('MODE: Snake coverage');
    W = m_snake_coverage(5.0, 2.0, ...
        'LaneSpacing', 0.25, ...
        'Margin',      0.10, ...
        'TurnRadius',  0.125, ...
        'ds',          0.05);
    
    figure; plot(W(:,1),W(:,2),'-'); axis equal; grid on; title('Snake-path');
    hold on; plot(W(1,1),W(1,2),'go','markerfacecolor','g');

    x0   = W(1,1);
    y0   = W(1,2);
    psi0 = atan2(W(2,2)-W(1,2), W(2,1)-W(1,1));

    lookahead = 0.20;  v_des = 0.25;  omega_max = 2.0;  k_v_goal = 1.5;

else
   %% === LATTICE-NAVIGASJON ===
disp('MODE: Lattice target');

init_g = [1 1 0];        % grid-indekser (0.1 m/cell)
goal_g = [50 20 0];

[W, lp] = m_lattice_pool_nav(init_g, goal_g);   

x0   = W(1,1);
y0   = W(1,2);
psi0 = atan2(W(2,2)-W(1,2), W(2,1)-W(1,1));

lookahead = 0.40;  v_des = 0.30;  omega_max = 2.0;  k_v_goal = 1.5;
end

%% --- Eksponer til Simulink ---
assignin('base','W',W);
assignin('base','x0',x0);
assignin('base','y0',y0);
assignin('base','psi0',psi0);
assignin('base','lookahead',lookahead);
assignin('base','v_des',v_des);
assignin('base','omega_max',omega_max);
assignin('base','k_v_goal',k_v_goal);