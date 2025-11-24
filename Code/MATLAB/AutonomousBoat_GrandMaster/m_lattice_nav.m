function out = m_lattice_nav(init, goal)
% Simulerer en Lattice-basert navigasjonsplanlegger i et 5 m x 2 m område.
% Merk: 'goal' må være en gyldig node i lattice-strukturen. 
% F.eks. posisjoner som ikke samsvarer med lattice-steget (3 celler)
% vil ikke kunne nås av planleggeren.
%% Definer avgrenset område
length = 5;   % Lengde på navigasjonsområdet [m]
width  = 2;   % Bredde på navigasjonsområdet [m]
area = length * width; 
disp(['Avgrenset område: ', num2str(area), ' m^2']);

%% Danner occupancy grid 
resolution = 0.1;                    % Grid-oppløsning [m/cell], gir tett nok nett
nx = round(width / resolution);
ny = round(length / resolution);
occupancy = zeros(nx, ny);                % 0 = fri celle (ingen hindringer)

%% ---- Initialiser Lattice planner ----
% 'grid', 3    → Lattice bruker et 3-cells translasjonssteg
% 'root'       → Start-node for generering av hele lattice-strukturen
lp = Lattice(occupancy, 'grid', 3, 'root', [1 1 0]);  % Konstruerer navigasjonsobjekt 

lp.plan();  % Genererer hele lattice-strukturen (noder + kanter)

%% ---- Figur 1: kun lattice-strukturen ----
fig1 = figure('Name','Lattice oppløsning: 0.1 m/cell','NumberTitle','off');
lp.plot();                        % Plotter selve lattice-griden uten path
ax0 = gca;
title(ax0,'Lattice planner — bare grid');
xlabel('X [cells]');
ylabel('Y [cells]');
axis equal; grid on;

%% ---- Figur 2: standard cost, path-planlegging ----
fig2 = figure('Name','Lattice oppløsning: 0.1 m/cell, low cost','NumberTitle','off');
lp.plot();                                % Plotter griden igjen før path
ax1 = gca;                                        
title(ax1,'Lattice planner (0.1 m/cell) — lattice');

disp('Original path: ')
lp.query(init, goal);         % Utfører ruteplanlegging mellom init og goal
lp.plot();                    % Visualiserer beregnet path                   
ax1 = gca;                                       
title(ax1,'Lattice planner (0.1 m/cell) — orginal path');

%% ---- Figur 3: modifisert kostnadsmodell ----
fig3 = figure('Name','Lattice oppløsning: 0.1 m/celle, high cost','NumberTitle','off');
% cost = [forward turn-left turn-right]
% Høyere kostnader for svinger → planlegger foretrekker rette segmenter
lp.plan('cost', [1 10 10]);
disp('Increased cost path: ')
lp.query(init, goal);       % Beregner ny path under endret kostnadsmodell
lp.plot();                  % Visualiserer ny rute
ax2 = gca;
title(ax2,'Lattice planner — økt turn cost');

out = lp;                   % Returnerer planleggerobjekt for videre bruk
end
