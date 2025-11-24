function W = m_snake_coverage(L, Wpool, varargin)
% m_snake_coverage  Lager "slange"-waypoints som dekker et rektangel.

p = inputParser;
p.addParameter('LaneSpacing', 0.25, @(x)isnumeric(x)&&x>0);
p.addParameter('Margin',      0.10, @(x)isnumeric(x)&&x>=0);
p.addParameter('TurnRadius',  0.20, @(x)isnumeric(x)&&x>0);
p.addParameter('ds',          0.05, @(x)isnumeric(x)&&x>0);
p.parse(varargin{:});
op = p.Results;

% Sikkerhetsjekker
assert(2*op.TurnRadius <= op.LaneSpacing+1e-9, ...
    'TurnRadius må være <= LaneSpacing/2.');
assert(2*op.Margin < min(L,Wpool), 'Margin for stor for basseng.');

x_min = op.Margin;  x_max = L - op.Margin;
y_min = op.Margin;  y_max = Wpool - op.Margin;

% Lag alle "baner" (y-verdier)
y_lanes = y_min:op.LaneSpacing:y_max;
if abs(y_lanes(end)-y_max) > 1e-9
    y_lanes(end+1) = y_max;  
end

W = [];

% Start i venstre-nedre hjørne, kjør mot høyre først
dirRight = true;
for k = 1:numel(y_lanes)
    yk = y_lanes(k);

    % Rett segment
    if dirRight
        x_line = x_min:op.ds:x_max;
    else
        x_line = x_max:-op.ds:x_min;
    end
    W = [W; [x_line(:) repmat(yk,numel(x_line),1)]]; 

    % U-sving til neste bane (hvis ikke siste bane)
    if k < numel(y_lanes)
        y_next = y_lanes(k+1);

        % Halvsirkel-bue: sentrum ligger "bak" endepunktet i x-retning
        if dirRight
            % Siste punkt var (x_max, yk) -> bue mot venstre
            cx = x_max - op.TurnRadius;
            if y_next > yk
                % Sving oppover (mot +y): vinkel fra -pi/2 til +pi/2
                cy = yk + op.TurnRadius;
                th = linspace(-pi/2, +pi/2, max(3,ceil(pi*op.TurnRadius/op.ds)));
            else
                % Sving nedover (mot -y)
                cy = yk - op.TurnRadius;
                th = linspace(+pi/2, -pi/2, max(3,ceil(pi*op.TurnRadius/op.ds)));
            end
        else
            % Siste punkt var (x_min, yk) -> bue mot høyre
            cx = x_min + op.TurnRadius;
            if y_next > yk
                % Sving oppover
                cy = yk + op.TurnRadius;
                th = linspace(pi+pi/2, pi-pi/2, max(3,ceil(pi*op.TurnRadius/op.ds)));
            else
                % Sving nedover
                cy = yk - op.TurnRadius;
                th = linspace(pi-pi/2, pi+pi/2, max(3,ceil(pi*op.TurnRadius/op.ds)));
            end
        end

        xb = cx + op.TurnRadius*cos(th);
        yb = cy + op.TurnRadius*sin(th);

        % Trim bue innen marginer (sjelden nødvendig, men safe)
        mask = xb>=x_min & xb<=x_max & yb>=y_min & yb<=y_max;
        xb = xb(mask);  yb = yb(mask);

        % Koble inn buen + kort overgang til neste bane
        if ~isempty(xb)
            W = [W; [xb(:) yb(:)]]; 
        end
        % Liten rett "kobling" vertikalt til eksakt y_next
        if abs(yb(end) - y_next) > 5e-3
            y_conn = linspace(yb(end), y_next, max(3,ceil(abs(yb(end)-y_next)/op.ds)));
            W = [W; [repmat(xb(end),numel(y_conn),1) y_conn(:)]]; 
        end
    end

    dirRight = ~dirRight; % snu kjøreretning for neste bane
end

% Sørg for siste punkt er eksakt på enden av siste rettlinje
if size(W,1) >= 2
    W(end,:) = [dirRight, ~dirRight];  
end
end
