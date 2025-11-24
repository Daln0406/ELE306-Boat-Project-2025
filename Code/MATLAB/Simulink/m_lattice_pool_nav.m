
%======================= lattice fnc ======================================

function [W, lp] = m_lattice_pool_nav(init, goal)
% Planlegg rute i et 5 m x 2 m basseng med Lattice og returner waypoints i meter.
% INN:  init, goal = [ix iy ith] (grid-indekser, 0.1 m per celle, ith=0..2)
% UT:   W (Nx2) i meter, lp = Lattice-objekt

%% Parametre (kart)
pool_length = 5;         % [m]
pool_width  = 2;         % [m]
res         = 0.1;       % [m/cell]
nx = round(pool_length / res);
ny = round(pool_width  / res);
pool = zeros(ny, nx, 'uint8');  % 0 = fri

%% Bygg planner + plan
lp = Lattice(pool, 'grid', 3, 'root', [1 1 0]);
lp.plan();

%% Forespør rute (kolonne-sikkert + fallback)
nori = 3;                       % 'grid',3
init(3) = mod(init(3), nori);
goal(3) = mod(goal(3), nori);
init_col = init(:);             % 3x1
goal_col = goal(:);             % 3x1

try
    % prøv direkte
    pth = lp.query(init_col, goal_col);
catch
    % finn nærmeste noder i grafen og prøv igjen
    vlist = lp.graph.vertexlist;           
    if size(vlist,1) == 3, V = vlist.'; else, V = vlist; end   
    [~, iStart] = min(sum((V(:,1:2) - init_col(1:2).').^2, 2));
    [~, iGoal ] = min(sum((V(:,1:2) - goal_col(1:2).').^2,  2));
    s = V(iStart,:).';                      
    g = V(iGoal,:).';                       
    pth = lp.query(s, g);
end

assert(~isempty(pth), 'Fant ingen sti fra latticen.');

%% Grid -> meter
xy = (pth(:,1:2) - 1) * res;   % [m]

%% Fortett for jevn følge (ca. 0.1 m mellom punkter)
W = densify_xy(xy, 0.1);
end

function W2 = densify_xy(W, ds)
% Lineær fortetting slik at punkter ligger ca. ds meter fra hverandre
if size(W,1) < 2, W2 = W; return; end
s = [0; cumsum(vecnorm(diff(W,1,1),2,2))];
snew = (0:ds:s(end)).';
W2 = [interp1(s, W(:,1), snew, 'linear'), interp1(s, W(:,2), snew, 'linear')];
if norm(W2(end,:)-W(end,:))>1e-9, W2 = [W2; W(end,:)]; end
end
