function out = m_lattice_nav_snake()
% Simulerer en Lattice-basert navigasjonsplanlegger med slangemønster/lawn-mower
% Trenger ikke oppgi start og goal, kun for visualisering med søkemønster
% Ikke snap to node, bare visuelt søkemønster over lattice plan. 

clc; clear all; close all;
%% Definer avgrenset område
length = 5;   % Lengde på navigasjonsområdet [m]
width  = 2;   % Bredde på navigasjonsområdet [m]
area = length * width;
disp(['Avgrenset område: ', num2str(area), ' m^2']);

%% Danner occupancy grid 
resolution = 0.1;                    % Grid-oppløsning [m/cell], gir tett nok nett
nx = round(width / resolution);
ny = round(length / resolution);
occupancy = zeros(nx, ny);                % 0 = free space

%% Initialiser Lattice planner 
lp = Lattice(occupancy, 'grid', 3, 'root', [1 1 0]); % Konstruerer navigasjonsobjekt 
lp.plan(); % Genererer hele lattice-strukturen (noder + kanter)

%% Plotter lattice grid
figure('Name','Lattice grid med slangemønster','NumberTitle','off');
lp.plot(); hold on;
axis equal; grid on;
xlabel('X [cells]');
ylabel('Y [cells]');
title('Lattice planner — grid med slangemønster');

%% Konstruerer slangemønster
n_stripes = 7;   % Antall horisontale 
R = 0.35;         % Svingradius 
snake_width = 2;   % ønsket bredde på slangemønsteret (m)
P = generateSnakePath(length, snake_width, n_stripes, R);

%% Plotter slangemønster skalert
scale = 1 / resolution;              % Konverterer meter til celle units
plot(P(:,1)*scale, P(:,2)*scale, 'k--', 'LineWidth', 2.0);
legend('Lattice grid','Snake pattern','Location','best');
hold off;

out = lp;

end

function P = generateSnakePath(L, W, n_stripes, R)
% Genererer en frem-og-tilbake (lawn-mower) bane over et rektangulært område.
% L = lengde (x-retning), W = bredde (y-retning)
% n_stripes = antall passeringer, R = svingradius
% margin = innvendig avstand fra bassengkanten [m]

if nargin < 4 || isempty(R)
    R = W / (2 * n_stripes);
else
    R = min(R, W / (2 * n_stripes));
end

margin = 0.1;    % for å justere: forkorte eller forlenge hver passering (meter)

y_levels = R + (0:n_stripes-1) * 2*R;
P = []; 
dir = 1;

for i = 1:n_stripes
    y = y_levels(i);

    if dir > 0
        % Fremover (venstre → høgre)
        x_line = linspace(R + margin, L - R - margin, 150);
        y_line = y * ones(size(x_line));
        P = [P; [x_line(:), y_line(:)]];

        if i < n_stripes
            theta = linspace(-pi/2, pi/2, 100);
            xc = L - R - margin;
            yc = y + R;
            x_turn = xc + R*cos(theta);
            y_turn = yc + R*sin(theta);
            P = [P; [x_turn(:), y_turn(:)]];
        end

    else
        % Bakover (høgre → venstre)
        x_line = linspace(L - R - margin, R + margin, 150);
        y_line = y * ones(size(x_line));
        P = [P; [x_line(:), y_line(:)]];

        if i < n_stripes
            theta = linspace(3*pi/2, pi/2, 100);
            xc = R + margin;
            yc = y + R;
            x_turn = xc + R*cos(theta);
            y_turn = yc + R*sin(theta);
            P = [P; [x_turn(:), y_turn(:)]];
        end
    end

    dir = -dir;   % Snu rettning ved hver ende
end
end
