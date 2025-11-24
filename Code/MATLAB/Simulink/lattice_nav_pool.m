function out = m_lattice_pool_nav(init, goal)
% Simulates the Lattice planner in a 10 m^2 pool
%Call must be multiple by 3 to be in the lattice [1 1 0] to [30 15 0] not
%in lattice --> 30-1 = 29 not multiple by 3
%disp('lp = m_lattice_pool_nav([1 1 0], [4 2 0])')

%% Define pool parameters
pool_length = 5;  % meters
pool_width  = 2;  % meters

area = pool_length * pool_width;
disp(['Pool area: ', num2str(area), ' m^2']);

%% Create occupancy grid 
resolution = 0.1;                      % meters per cell (5 cm)
nx = round(pool_length / resolution);
ny = round(pool_width / resolution);
pool = zeros(ny, nx);                   % 0 = free space

%% Navigation using lattice planner
lp = Lattice(pool, 'grid', 3, 'root', [1 1 0]);  % Construct the navigation object

lp.plan();                                       % Plan lattice
figure;                          
lp.plot();                                       % Plot lattice
title('Lattice planner, 0.1 scale');
disp('Original path: ')
lp.query(init, goal);                            % Compute path between init and goal
lp.plot();
title('0.1 skalering, orginal kostplan:  ')

%% Modify turning cost
figure;
lp.plan('cost', [1 10 10]);
disp('Increased cost path: ')
lp.query(init, goal);
lp.plot();
title('0.1 skalering, økt kostplan: ')
out = lp;
end
