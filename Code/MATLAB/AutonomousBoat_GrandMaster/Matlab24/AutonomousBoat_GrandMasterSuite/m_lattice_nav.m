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

%% ---- SNAP TIL NÆRMESTE LATTICE-NODE ----
thetaSet = [0 pi/2 pi -pi/2]; % juster hvis du bruker andre heading-tilstander
gridStep = 3;                 % samme som i lp-konstruktøren
rootPose = [1 1 0];           % samme som i lp-konstruktøren

% 1) Snap start og mål til nærmeste gitterposisjon (x,y,theta)
init_snapped = snap_to_lattice(init, rootPose, gridStep, thetaSet, [1 ny; 1 nx]);
goal_snapped = snap_to_lattice(goal, rootPose, gridStep, thetaSet, [1 ny; 1 nx]);

% 2) Finn nærmeste eksisterende node i grafen
[sIdx, ~] = lp.graph.closest(init_snapped, 1e9); % stor radius → finn alltid
[gIdx, ~] = lp.graph.closest(goal_snapped, 1e9);

assert(~isempty(sIdx) && ~isempty(gIdx), 'Fant ikke gyldige noder i lattice.');

qstart = lp.graph.vertexlist(:, sIdx).'; % [ix iy th] fra grafen
qgoal = lp.graph.vertexlist(:, gIdx).';

%{
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
%}

%% ---- Figur 3: modifisert kostnadsmodell ----
fig3 = figure('Name','Lattice oppløsning: 0.1 m/celle, high cost','NumberTitle','off');
% cost = [forward turn-left turn-right]
% Høyere kostnader for svinger → planlegger foretrekker rette segmenter
lp.plan('cost', [1 4 4]);
disp('Increased cost path: ')
lp.query(init, goal);       % Beregner ny path under endret kostnadsmodell
lp.plot();                  % Visualiserer ny rute
ax2 = gca;
title(ax2,'Lattice planner — økt turn cost');

out = lp;                   % Returnerer planleggerobjekt for videre bruk
end

%% ===== HJELPERFUNKSJON (lokal) =====
function p2 = snap_to_lattice(p, rootPose, step, thetaSet, boundsXY)
% p: [ix iy] eller [ix iy theta] i celler.
% Snapper til nærmeste gyldige lattice-node gitt rootPose og step.

if numel(p) < 3
    p = [p(:).' 0];
end

ix = p(1);
iy = p(2);
th = p(3);

% Snap posisjon til gitteret:
ix2 = rootPose(1) + step * round((ix - rootPose(1)) / step);
iy2 = rootPose(2) + step * round((iy - rootPose(2)) / step);

% Klipp innen kartgrenser
ix2 = min(max(ix2, boundsXY(1,1)), boundsXY(1,2));
iy2 = min(max(iy2, boundsXY(2,1)), boundsXY(2,2));

% Snap heading til nærmeste i thetaSet
th = mod(th + pi, 2*pi) - pi; % wrap til [-pi, pi]
[~,k] = min(abs(mod((thetaSet - th) + pi, 2*pi) - pi));
th2 = thetaSet(k);

p2 = [ix2 iy2 th2];
end

