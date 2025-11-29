function simulate_boat_master_liveplots_current()
% Autonom båt med PID + LOS, IMU, extended kalman-filter og støy.
% 

clc; clear all;
fprintf('Starter båt-simulering med IMU, Extended Kalman-filter og strømvektor...\n');

%% ---------- PARAMETRE ----------
dt               = 0.005;      % tidssteg [s], 0.005 for "smoothere" visualisering i Gazebo 
snake_speed      = 0.5;        % fremdrift [m/s] (konstant i denne modellen)
plot_update_sec  = 0.005;      % oppdater plott hvert 0.05 s
max_time         = 1200;       % maks simuleringstid [s]
USE_EKF_IN_CONTROL = false;    % true = PID bruker theta_est, false = bruker sann theta

% PID
Kp = 6.5;                      % forsterkning 
Ki = 0.35;                     % integrasjonstid
Kd = 0.10;                     % derrivasjonstid

% LOS
L_look = 0.45;                 % LOS-avstand [m]
K_ct   = 2.0;                  % kryss-spor forsterkning

% Båtgeometri / aktuator
w = 0.30;                      % avstand thrustere [m]
omega_max = 6.0;               % rotasjonsmetning [rad/s] 
tau_act   = 0.30;              % tidkonstant aktuator 

% Strøm (konstant)
v_current = [0.0, -0.1];       % [x,y] --> sqr(x^2 + y^2) = ||a|| m/s 

%% ---------- KAMERA / ARM-GEOMETRI ----------
[T_BA, T_BC] = frames_boat_arm_cam();

% Tilstand knyttet til plast / lattice
plastic_world = [];      % tom til vi "oppdager" noe
mode = "snake";          % "snake" eller "goto_plastic"

% ------ ARM-MODELL (opprettes én gang) ------
robot = boat_arm_model();   % Vår robotarm


%% ---------- SLANGEBANE ----------
% Parameter for slangemønster
NX = 5; NY = 2; turn_radius = 0.3; n_stripes = 4;
snake_path = generateSnakePath(NX, NY, n_stripes, turn_radius); 
s_path = cumulativeArcLength(snake_path);

% Kopi av original snake-bane for senere gjenopptak
snake_path_saved = snake_path;
s_path_saved     = s_path;
s_prog_saved     = 0;   % vi oppdaterer denne når vi avbryter
boatPose_at_snake_interrupt = [];

% Starttilstand
x = snake_path(1,1);
y = snake_path(1,2);
theta = 0.0;

% PID-minne oppdateres undervegs i simuleringen
ei = 0; e_prev = 0; s_prog = 0;

% Aktuator tilstand
omega_act = 0.0;         

%% ---------- IMU + EKF ----------
Q = diag([0.01 0.01 0.001 0.05]); % Prosess-støy, usikkerhet i modellens dynamikk 

% Måle-støy: 
R_mag_heading  = 0.02;                % varians for magnetometer-heading

P = eye(4)*0.1;                       % Kovariansmatrise start usikkerheten i estimatet
x_hat = [x; y; theta; snake_speed];   % Startverdien for tilstandsestimatet i Kalman-filteret (pos og heading) 

gyro_bias  = 0.01;   % Bias konstant systematisk feil i målingen
acc_noise  = 0.05;   % Støy på akselerometeret 
gyro_noise = 0.01;   % Støy på gyroskopet 
mag_noise  = 0.02;   % Støy på magnetometeret 

%% ---------- FIGUR 1: BANE + STRØM ----------
figure(1); clf; hold on; axis equal; grid on;
xlabel('x [m]'); ylabel('y [m]');
title('Autonom båt - PID + LOS + IMU');
hPath  = plot(snake_path(:,1), snake_path(:,2),'m--','LineWidth',1.0);
hBoat  = plot(x,y,'bo','MarkerFaceColor','b');
hTrail = plot(x,y,'b-');

% Plast-markør 
hPlastic = plot(nan, nan, 'rx', 'MarkerSize', 10, 'LineWidth', 2);

legend([hPath, hBoat, hTrail, hPlastic], ...
       {'Snake path','Boat','Trail','Plastic'}, 'Location','best');


% --- Strømpil for plott 1 ---
x_arrow = 0.5;
y_arrow = mean(snake_path(:,2));

% Definer strømvektor og skaler visuelt
v_vec = v_current;
norm_v = norm(v_vec);
if norm_v < 1e-6
    v_vec = [0 0];
    norm_v = 0;
end
arrow_scale = 3;   
vx_plot = v_vec(1) * arrow_scale;
vy_plot = v_vec(2) * arrow_scale;

% Tegn selve pila
hCurrent = quiver(x_arrow - 0.1, y_arrow + 0.15, vx_plot, vy_plot, 0, ...
    'Color',[0 0.6 1], 'LineWidth',2, 'MaxHeadSize',2.5, 'ShowArrowHead','on');
     set(get(get(hCurrent, 'Annotation'), 'LegendInformation'), 'IconDisplayStyle', 'off'); %For å fjerne unødvendig info fra legend

% Tekst ved pilen 
text(x_arrow + vx_plot*0.3, y_arrow + vy_plot*0.3 + 0.15, ...
    sprintf('Strøm: %.2f m/s', norm_v), ...
    'Color',[0 0.6 1], 'FontSize',9, 'FontWeight','bold');

trailX = x; trailY = y;

%% ---------- FIGUR 2: HEADING ----------
figure(2); clf; set(gcf,'Name','2 – Headingestimat','NumberTitle','off');
subplot(2,1,1); hold on; grid on;
hHeadingTrue = plot(nan,nan,'b','LineWidth',1.2);
hHeadingEst  = plot(nan,nan,'r--','LineWidth',1.2);
legend('Sann','Estimert');
ylabel('\theta [rad]'); title('Kalman-estimert heading');

subplot(2,1,2); hold on; grid on;
hHeadingErr = plot(nan,nan,'k');
xlabel('Tid [s]'); ylabel('Feil [rad]');
title('Estimeringsfeil');

%% ---------- FIGUR 3: IMU ----------
figure(3); clf; set(gcf,'Name','3 – IMU-signaler','NumberTitle','off');
subplot(3,1,1); hold on; grid on;
hGyroRaw  = plot(nan,nan,'r:');
hGyroFilt = plot(nan,nan,'r','LineWidth',1.2);
hGyroTrue = plot(nan,nan,'k--','LineWidth',1.2);
legend('Gyro rå','Gyro filt','Gyro sann');
ylabel('\omega [rad/s]'); title('Gyroskopmåling');

subplot(3,1,2); hold on; grid on;
hAxRaw  = plot(nan,nan,'b:');
hAxFilt = plot(nan,nan,'b','LineWidth',1.2);
legend('a_x rå','a_x filt'); ylabel('a_x [m/s^2]');
title('Akselerometer – X-retning');

subplot(3,1,3); hold on; grid on;
hAyRaw  = plot(nan,nan,'g:');
hAyFilt = plot(nan,nan,'g','LineWidth',1.2);
legend('a_y rå','a_y filt'); ylabel('a_y [m/s^2]');
xlabel('Tid [s]'); title('Akselerometer – Y-retning');

%% ---------- LOGGING ----------
theta_log = [];         % Sann (simulert) heading
theta_est_log = [];     % Estimert heading fra Kalman-filter 
t_log = [];             % Simulert tid [s]
omega_true_log = [];    % Faktisk yaw-rate fra modellen
omega_meas_log = [];    % Målt yaw-rate fra gyroskopet
ax_meas_log = [];       % For logging av akselerasjonsmåling x-retning med støy
ay_meas_log = [];       % For logging av akselerasjonsmåling y-retning med støy

%Initialisering av tidsvariabler
t = 0;              % Nåværende tid i simuleringen
k = 0;              % Tidssteg teller
t_last_plot = 0;    % Hvor ofte plottet oppdateres

%% ---------- HOVEDLØKKE ----------
while t < max_time
    k = k + 1; t = t + dt;

   % ---------- STYRING + KINEMATIKK ----------
    if mode == "snake" || mode == "goto_plastic" || mode == "return_to_snake"
        % --- LOS og cross-track ---
        [xL, yL, s_prog] = losTarget(snake_path, s_path, [x y], s_prog, L_look);
        [e_ct, ~] = crossTrackError(snake_path, s_path, [x y]); 
        chi_d_raw = atan2(yL - y, xL - x);
        chi_d = chi_d_raw - atan(K_ct * e_ct / max(L_look, 1e-6));

        % --- Velg heading for kontroll (sann eller estimert) ---
        if USE_EKF_IN_CONTROL
            theta_ctrl = wrapToPi(x_hat(3));
        else
            theta_ctrl = theta;
        end

        % --- PID på kursfeil ---
        e_psi = wrapToPi(chi_d - theta_ctrl);
        [omega_cmd, ei] = pid_update_aw(e_psi, e_prev, ei, Kp, Ki, Kd, dt, -omega_max, omega_max);
        e_prev = e_psi;

        % --- Aktuator (1. ordens) faktisk yaw-rate ---
        domega = (omega_cmd - omega_act) * (dt / tau_act);
        omega_act = omega_act + domega;  % faktisk rotasjonshastighet

        % --- Kinematikk (KUN omega_act brukes videre) ---
        vR = snake_speed + 0.5*w*omega_act;
        vL = snake_speed - 0.5*w*omega_act;
        v  = 0.5*(vR + vL);

        x     = x + (v*cos(theta) + v_current(1)) * dt;
        y     = y + (v*sin(theta) + v_current(2)) * dt;
        theta = wrapToPi(theta + omega_act * dt);

    else
        % mode == "pick" eller "done" → ingen aktiv styring, hold posisjon
        omega_cmd = 0;
        omega_act = 0;
        v         = 0;

    end

    boatPose = [x, y, theta];

        %% ---------- MODUS-LOGIKK (snake / goto_plastic / pick) ----------
    switch mode
        case "snake"
            
            % Simulert kameradeteksjon 
            if t > 56 && isempty(plastic_world)
                % 1) Kameramåling i C-frame 
                pC = [1.0; 1.0; 0.0];

                % 2) Kamera -> World
                plastic_world = camToWorld(pC, boatPose, T_BC);

                fprintf('Plast oppdaget! World-posisjon: [%.2f, %.2f]\n', ...
                        plastic_world(1), plastic_world(2));
                
                set(hPlastic, 'XData', plastic_world(1), 'YData', plastic_world(2));
                
                % Husk hele tilstanden til snake-banen da vi avbrøt
                snake_path_saved           = snake_path;   % banen slik den er nå 
                s_path_saved               = s_path;
                s_prog_saved               = s_prog;       % hvor langt vi var kommet
                boatPose_at_snake_interrupt = boatPose;    % [x y theta] ved avbrudd

                % Planlegg Lattice-bane fra nåværende posisjon til plast
                % Lattice-funksjonen vår m_lattice_pool_nav jobber i grid-indekser:
                % init, goal = [ix iy ith], 0.1 m per celle

                res  = 0.1;   % MÅ matche m_lattice_pool_nav (res i funksjonen)
                nori = 3;     % 'grid',3 → 3 orientasjoner
                
                % Konverter nåværende båtpose (meter) → grid-indekser
                ix0 = round(boatPose(1) / res) + 1;
                iy0 = round(boatPose(2) / res) + 1;
                ith0 = 0;   
                
                % Konverter plastposisjon (meter) → grid-indekser
                ixg = round(plastic_world(1) / res) + 1;
                iyg = round(plastic_world(2) / res) + 1;
                ithg = 0;   
                
                init_idx = [ix0, iy0, ith0];
                goal_idx = [ixg, iyg, ithg];
                
                % Kall Lattice-planleggeren – den returnerer waypoints W i meter
                [W, lp] = m_lattice_pool_nav(init_idx, goal_idx);
                
                % 4) Bytt ut slangebane med Lattice-banen (i meter)
                snake_path = W;                            % W er Nx2 [x y] i meter
                s_path     = cumulativeArcLength(snake_path);
                s_prog     = 0;
                
                % Oppdater plottet for banen
                set(hPath, 'XData', snake_path(:,1), 'YData', snake_path(:,2));
                
                mode = "goto_plastic";
            end

        case "goto_plastic"
            % Her bruker vi nøyaktig samme LOS/PID-kode som i "snake"-modus.
            % Forskjellen er at snake_path nå = Lattice-bane mot plast.

            if ~isempty(plastic_world)
                dist_to_plastic = norm([x - plastic_world(1), y - plastic_world(2)]);
                if dist_to_plastic < 0.2    % f.eks. 20 cm radius
                    fprintf("Nær plasten ved t=%.1fs – går til pick-modus.\n", t);
                    mode = "pick";
                end
            end

        case "pick"
            % Stopp båtens fremdrift mens armen jobber
            omega_cmd = 0;
            omega_act = 0;
            v         = 0;

            fprintf("Starter full arm-sveip (hav-script) ved t=%.1fs ...\n", t);

            % === KJØR FULL ARM-BEVEGELSE ===
            run_arm_hav_sweep();   % eget script for armen

            fprintf("Full arm-sveip ferdig.\n");

            % Etter arm-sveip: gå tilbake til slange-banen
            if ~isempty(boatPose_at_snake_interrupt)

                pos_target = boatPose_at_snake_interrupt(1:2);
                res = 0.1;

                ix0  = round(boatPose(1) / res) + 1;
                iy0  = round(boatPose(2) / res) + 1;
                ith0 = 0;

                ixg  = round(pos_target(1) / res) + 1;
                iyg  = round(pos_target(2) / res) + 1;
                ithg = 0;

                init_idx = [ix0 iy0 ith0];
                goal_idx = [ixg iyg ithg];

                [W_back, ~] = m_lattice_pool_nav(init_idx, goal_idx);

                snake_path = W_back;
                s_path     = cumulativeArcLength(snake_path);
                s_prog     = 0;
    
            % Lagre hvor på original snake vi skal fortsette (bue-lengde)
            % her bruker vi bare s_prog_orig rett fram
            mode = "return_to_snake";
        else
            % fallback: hvis vi ikke har lagret noe, bare gå til done
            mode = "done";
        end
        
        case "return_to_snake"
        % Bruker vanlig LOS/PID mot snake_path (som nå er W_back)
        % Når vi er nær målpunktet -> koble tilbake til original snake
        if ~isempty(boatPose_at_snake_interrupt)
            pos_target = boatPose_at_snake_interrupt(1:2);
            dist_to_target = norm([x - pos_target(1), y - pos_target(2)]);
           if dist_to_target < 0.2
                fprintf("Tilbake på snake-banen – fortsetter søk.\n");

                % Gjenopprett snake-banen slik den var da vi avbrøt
                snake_path = snake_path_saved;
                s_path     = s_path_saved;
                s_prog     = s_prog_saved;
                set(hPath, 'XData', snake_path(:,1), 'YData', snake_path(:,2));
            
                mode = "snake";
            end
        end

        case "done"
            
            omega_cmd = 0;
            omega_act = 0;
            
    end


    % --- IMU målinger ---
    omega_true = omega_act;                                   % Sann rotasjonshastighet
    omega_meas = omega_true + gyro_bias + gyro_noise*randn;   % Gyro-måling med bias og støy

    % Akselerometer i kroppsramme (2D, konstant fart)
    ax_true    = 0.0;             % Konstant fart gir ingen akselerasjon i x-retning
    ay_true    = v * omega_true;  % Konstant fart gir kun sentripital akselerasjon

    ax_meas    = ax_true + acc_noise*randn;                   % Målt akselerasjon med støy
    ay_meas    = ay_true + acc_noise*randn;                   % Målt akselerasjon med støy

    theta_mag_meas = wrapToPi(theta + mag_noise*randn);       % Magnetometer måling av heading med støy

    % --- EKF PREDIKSJON ---
   v_pred  = x_hat(4);                       % estimert fart
   theta_h = x_hat(3);                       % estimert heading
   omega_h = omega_meas - gyro_bias;         % bias-korrigert yaw-rate

   % 4x4 Jacobimatrise
   F = [1 0 -v_pred*dt*sin(theta_h)   dt*cos(theta_h);
       0 1  v_pred*dt*cos(theta_h)   dt*sin(theta_h);
       0 0  1                        0;
       0 0  0                        1];

   % Tilstandsforutsigelse
    x_hat = x_hat + [ ...
    v_pred * cos(theta_h);       % dx
    v_pred * sin(theta_h);       % dy
    omega_h;                     % dtheta
    0                            % dv/dt antas 0 (holdes konstant)
    ] * dt;

    % Kovariansforutsigelse
    P = F * P * F' + Q;

    % --- EKF OPPDATERING 1 (magnetometer-heading) ---
    H = [0 0 1 0];                       % Målematrise: vi måler direkte på theta
    z_mag = theta_mag_meas;            % Faktisk måling fra magnetometer
    y_mag = wrapToPi(z_mag - H*x_hat); % Måleavvik (innovasjon)
    S_mag = H*P*H' + R_mag_heading;    % Måleusikkerhet
    K_mag = P*H'/S_mag;                % Kalman-forsterkning
    x_hat = x_hat + K_mag*y_mag;       % Oppdaterer tilstandsestimat
    P = (eye(4)-K_mag*H)*P;            % Oppdaterer kovarians

    % ---------- EKF OPPDATERING 2: AKSELEROMETER lagt til i ettertid ----------
    z_acc = ay_meas;               % målt tverrakselerasjon
    h_acc = x_hat(4) * omega_h;    % forventet: ay = v * omega

    H_acc = [0 0 0 omega_h];       
    R_acc = 0.05;                  

    y_acc = z_acc - h_acc;         
    S_acc = H_acc * P * H_acc' + R_acc;
    K_acc = P * H_acc' / S_acc;

    x_hat = x_hat + K_acc * y_acc;          % oppdater tilstand
    P = (eye(4) - K_acc * H_acc) * P;       % oppdater kovarians


    % --- Logg ---
    trailX(end+1)=x; trailY(end+1)=y;                        % Logger båtens posisjon (x,y)
    theta_log(end+1)=theta;                                  % Lagrer sann heading
    theta_est_log(end+1)=wrapToPi(x_hat(3));                 % Logger estimert heading
    omega_true_log(end+1)=omega_true;                        % Logger sann rotasjonshastighet
    omega_meas_log(end+1)=omega_meas;                        % Logger målt rotasjonshastighet
    ax_meas_log(end+1)=ax_meas; ay_meas_log(end+1)=ay_meas;  % Logger målt "aks" (x,y)
    t_log(end+1)=t;                                          % Logger simulert tid til plottene

    % --- Oppdater plott ---
    if (t - t_last_plot) >= plot_update_sec
        t_last_plot = t;

        set(hBoat,'XData',x,'YData',y);
        set(hTrail,'XData',trailX,'YData',trailY);

        set(hHeadingTrue,'XData',t_log,'YData',wrapToPi(theta_log));
        set(hHeadingEst, 'XData',t_log,'YData',wrapToPi(theta_est_log));
        set(hHeadingErr, 'XData',t_log,'YData',wrapToPi(theta_log - theta_est_log));

        omega_meas_filt = movmean(omega_meas_log,10);
        ax_meas_filt    = movmean(ax_meas_log,10);
        ay_meas_filt    = movmean(ay_meas_log,10);
        set(hGyroRaw, 'XData',t_log,'YData',omega_meas_log);
        set(hGyroFilt,'XData',t_log,'YData',omega_meas_filt);
        set(hGyroTrue,'XData',t_log,'YData',omega_true_log);
        set(hAxRaw,  'XData',t_log,'YData',ax_meas_log);
        set(hAxFilt, 'XData',t_log,'YData',ax_meas_filt);
        set(hAyRaw,  'XData',t_log,'YData',ay_meas_log);
        set(hAyFilt, 'XData',t_log,'YData',ay_meas_filt);

        drawnow limitrate;
    end

    % --- Snur og fortsetter uten stopp ---
     if s_prog >= s_path(end) - 1e-6 && mode == "snake"
        fprintf("Nådd enden ved t=%.1fs – snur.\n", t);
        snake_path = flipud(snake_path);
        s_path = cumulativeArcLength(snake_path);
        s_prog = 0;
        theta = wrapToPi(theta + pi);
    end
end

fprintf('Simulering ferdig.\n');
end

%% ---------- HJELPEFUNKSJONER ----------
function [u,ei]=pid_update_aw(e,e_prev,ei,Kp,Ki,Kd,dt,umin,umax)  % Funksjon PID-kontroller med anti-windup
% PID med anti-windup
ei = ei + e*dt;                % Oppdaterer integrert feil 
de = (e - e_prev)/dt;          % Beregner ending i derivasjonsfeil
u  = Kp*e + Ki*ei + Kd*de;     % Beregner kontrollsignal (utgang)
u  = max(min(u, umax), umin);  % Begrenser utgang for å unngå metning
if u==umax || u==umin          % anti-windup
    ei = ei - e*dt;           
end
end

function s=cumulativeArcLength(P)
d=sqrt(sum(diff(P,1,1).^2,2));  %Beregner avstanden mellom punktene i banen
s=[0; cumsum(d)];               % Summerer opp delavstander
end

function [xL,yL,s_prog_out]=losTarget(P,S,pos,s_prog_in,L)
[s_closest,~,~]=projectPointToPolyline(P,S,pos);    % Finn nærmeste punkt på banen ift båtens posisjon
s_base=max(s_prog_in,s_closest);                    % Sikrer fremoverbevegelse 
s_des=min(s_base+L,S(end));                         % Bestemmer ønsket s-verdi, L meter fremover langs banen
[xL,yL]=samplePolylineAtS(P,S,s_des);               % Henter koordinatene for LOS punkt
s_prog_out=s_base;                                  % Oppdaterer fremdriftsparameter 
end

function [s_closest,x_proj,y_proj]=projectPointToPolyline(P,S,pos)
x=pos(1);y=pos(2);                                    % Henter båtens posisjon (x,y)
s_closest=0;x_proj=P(1,1);y_proj=P(1,2);best_d2=inf;  % Initialisering for søk
for i=1:size(P,1)-1                                   % Går gjennom alle linjesegmenter i banen
    a=P(i,:);b=P(i+1,:);ab=b-a;ab2=dot(ab,ab);        % Henter punkter og vektor for segmentet
    if ab2==0,continue;end                            % Dropper identiske punkter
    t=dot([x y]-a,ab)/ab2;                            % Beregner projeksjonsparameter langs segmentet
    t=max(0,min(1,t));                                % Sikrer at projeksjonen er innenfor segmentet
    p=a+t*ab;                                         % Beregner projeksjonspunkt på segmentet
    d2=sum(([x y]-p).^2);                             % Avstand mellom båt og segment
    if d2<best_d2                                     % Dersom punktet er nermere enn tidligere
        best_d2=d2;                                   % Oppdaterer minste avstand
        s_closest=S(i)+t*(S(i+1)-S(i));               % Beregner buelengde langs banen
        x_proj=p(1);y_proj=p(2);                      % Lagrer koordinat til nermest punkt
    end
end
end

function [xS,yS]=samplePolylineAtS(P,S,s)
s=max(0,min(S(end),s));                             % Sørger for at s holder seg innenfor banebegrensningen
i=find(S<=s,1,'last');                              % Finner segmentet hvor s ligger mellom to punkter
if i==numel(S),xS=P(end,1);yS=P(end,2);return;end   % Returnerer siste punkt dersom s er på slutten
s0=S(i);s1=S(i+1);                 % Henter start og sluttverdi for buelengde
tau=(s-s0)/(s1-s0+eps);            % Normaliserer posisjonen mellom punktene
p=P(i,:)+tau*(P(i+1,:)-P(i,:));    % Interpolerer punktet langs segmentet
xS=p(1);yS=p(2);                   % Henter koordinater for punktet (x,y)
end

function [e_ct, chi_path] = crossTrackError(P, S, pos)
[x_proj, y_proj, chi_path] = projectHeadingToPolyline(P, S, pos);   % Finner nermeste buevinkel
dx = pos(1) - x_proj;                                               % Differansen x
dy = pos(2) - y_proj;                                               % Differansen y
e_ct = -sin(chi_path)*dx + cos(chi_path)*dy;                        % Beregner tverravvik
end

function [x_proj, y_proj, chi_path] = projectHeadingToPolyline(P, S, pos)
x = pos(1); y = pos(2);                                          % Båtens nåværende posisjon (x,y)
best_d2 = inf; chi_path = 0;                                     % Initialiserer minste avstand og banevinkel
x_proj = P(1,1); y_proj = P(1,2);                                % Setter startverdier for projeksjonspunkt
for i = 1:size(P,1)-1                                            % Går gjennom alle linjesegmenter 
    a = P(i,:); b = P(i+1,:); ab = b - a; ab2 = dot(ab,ab);      % Henter punktene og vektor for segmentet
    if ab2 == 0, continue; end                                   % Hopper over identiske punkt
    t = dot([x y]-a, ab) / ab2;                                  % Beregner projeksjonsfaktor t langs segmentet
    t = max(0, min(1, t));                                       % Sikrer punkter innenfor segmentet
    p = a + t*ab;                                                % Beregner projeksjonspunkt p 
    d2 = sum(([x y]-p).^2);                                      % Avstand mellom båten og segment
    if d2 < best_d2                                              % Sjekker at punktet er nermere enn før 
        best_d2 = d2;                                            % Oppdaterer minste avstand
        x_proj = p(1); y_proj = p(2);                            % Lagrer koordinat (x,y) til nermeste punkt
        chi_path = atan2(ab(2), ab(1));                          % Beregner heading langs banen
    end
end
end

function P = generateSnakePath(L, W, n_stripes, R)
if nargin < 4 || isempty(R)
    R = W / (2 * n_stripes);          % Standard svingradius
else
    R = min(R, W / (2 * n_stripes));  % Sikrer at radius ikke overskrider banebredde
end
y_levels = R + (0:n_stripes-1) * 2*R;            % Beregner y-posisjon for hver linje
P = []; dir = 1;                                 % Initierer banematrise og retning
for i = 1:n_stripes                              % Genererer en stripe om gangen
    y = y_levels(i);                             % Nåværende stripenivå i y-retning
    if dir > 0                                   % Stripe fra høgre til venstre 
        x_line = linspace(R, L - R, 150);        % Rett linje fra start til slutt
        y_line = y * ones(size(x_line));         % Konstant y-verdi for linjen
        P = [P; [x_line(:), y_line(:)]];         % Legger linjen til i banen
        if i < n_stripes                         % Lag sving oppover
            theta = linspace(-pi/2, pi/2, 100);  % Halvsirkel for svingen
            xc = L - R; yc = y + R;              % Senter på svingen
            x_turn = xc + R*cos(theta);          % X-koordinat for sving
            y_turn = yc + R*sin(theta);          % Y-koordinat for sving
            P = [P; [x_turn(:), y_turn(:)]];     % Legger svingen til i banen
        end
    else
        x_line = linspace(L - R, R, 150);        % Rett linje motsatt vei
        y_line = y * ones(size(x_line));         % Konstant y-verdi for linjen
        P = [P; [x_line(:), y_line(:)]];         % Legger linjen til i banen
        if i < n_stripes                         % Lag sving i motsatt rettning 
            theta = linspace(3*pi/2, pi/2, 100); % Halvsirkel motsatt vei
            xc = R; yc = y + R;                  % Senter på svingen
            x_turn = xc + R*cos(theta);          % X-koordinat for sving
            y_turn = yc + R*sin(theta);          % Y-koordinat for sving
            P = [P; [x_turn(:), y_turn(:)]];     % Legger svingen til i banen
        end
    end
    dir = -dir;  % Endrer retning for neste stripe
end
end

%======================= lattice fnc ======================================

function [W, lp] = m_lattice_pool_nav(init, goal)
% Planlegg rute i et 5 m x 2 m med Lattice og returner waypoints i meter.
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
lp.plan('cost', [1 2 2]); % setter costen her for å feks redusere svinger

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
    vlist = lp.graph.vertexlist;           % 3xN eller Nx3
    if size(vlist,1) == 3, V = vlist.'; else, V = vlist; end   % Nx3
    [~, iStart] = min(sum((V(:,1:2) - init_col(1:2).').^2, 2));
    [~, iGoal ] = min(sum((V(:,1:2) - goal_col(1:2).').^2,  2));
    s = V(iStart,:).';                      % 3x1
    g = V(iGoal,:).';                       % 3x1
    pth = lp.query(s, g);
end

assert(~isempty(pth), 'Fant ingen sti fra latticen.');

%% ---------- DEBUG-PLOTT 1: Hele lattice-kartet i indeks-rom ----------
figure(4); clf;
lp.plot();                 % plotter alle noder/buer
hold on;

% A*-pathen vi faktisk bruker (pth: [ix iy ith])
plot(pth(:,1), pth(:,2), 'r-', 'LineWidth', 2);

% marker start og mål
plot(init_col(1), init_col(2), 'ko', 'MarkerSize', 8, 'LineWidth', 2);
plot(goal_col(1), goal_col(2), 'k*', 'MarkerSize', 10, 'LineWidth', 2);

title('Lattice planner — økt turn cost');
xlabel('ix'); ylabel('iy');
axis equal;
xlim([1 nx]); ylim([1 ny]);   % vis hele bassenget (50 x 20 celler)
grid on;
legend({'Lattice','Path','Start','Goal'}, 'Location','best');

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
