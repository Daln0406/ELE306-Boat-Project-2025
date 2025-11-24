clear; clc; close all;

% DH-parametre
[alpha_1, alpha_4] = deal(-pi/2, pi/2);
[alpha_2, alpha_3, alpha_5] = deal(0);
[a1, a2, a3, a4, a5] = deal(0, 0.35, 0.30, 0, 0);
[d1, d2, d3, d4, d5] = deal(0.2, 0, 0, 0.2, 0.05);

%      [theta d a alpha sigma]
L1 = Link([0 d1 a1 alpha_1 0]);
L2 = Link([0 d2 a2 alpha_2 0]);
L3 = Link([0 d3 a3 alpha_3 0]);
L4 = Link([0 d4 a4 alpha_4 0]);
L5 = Link([0 d5 a5 alpha_5 0]);
L2.offset = -pi/2;
L4.offset = pi/2;

% Sett sammen leddene til et SerialLink-objekt
robot = SerialLink([L1 L2 L3 L4 L5],'name','5DOF');
% Håv/endeffektor
robot.tool = transl(0,0,0.20); % Håven stikker 20 cm ut fra håndleddet

% Jacobian
q_fixed = [0.3 -0.5 0.6 -0.2 0.4]; % Faste leddvinkler
J0 = robot.jacob0(q_fixed)
Jv = J0(1:3,:);
Jw = J0(4:6,:);
rankJ = rank(J0);
condJ = cond(J0*J0');
manip = sqrt(det(Jv*Jv.'));
fprintf('Rang=%d, Kondisjon=%.2e, Manipulerbarhet=%.4f\n',rankJ,condJ,manip);

qdot  = [0.1 0 0 0 0]; % Bare ledd 1 beveger seg

xdot  = J0 * qdot.' % 6x1: [v; omega]

joint_idx = 2; % Analyserer ledd 2
theta_range = linspace(-pi/2, pi/2, 200);

cond_values = zeros(size(theta_range));
manip_values = zeros(size(theta_range));

% Itererer gjennom vinklene theta_range
for i = 1:length(theta_range)
    q_fixed(joint_idx) = theta_range(i); % Kun ledd 2 roteres
    J0 = robot.jacob0(q_fixed); % Regner Jacobianen for hele roboten i konfigurasjon q_fixed, ledd 2 varierer
    Jv = J0(1:3,:);

    % Beregner kondisjonstall og manipulability for translasjonsdelen, Jv
    if rank(Jv) < 3
        cond_values(i) = NaN;
        manip_values(i) = 0;
    else
        cond_values(i) = cond(Jv*Jv.');
        manip_values(i) = sqrt(det(Jv*Jv.'));
    end
end

% Plot resultatene
yyaxis left
plot(rad2deg(theta_range), log10(cond_values), 'LineWidth',1.8);
ylabel('log_{10}(Kondisjonsnummer)'); grid on;
yyaxis right
plot(rad2deg(theta_range), manip_values, 'LineWidth',1.8);
ylabel('Manipulerbarhet w');
xlabel(sprintf('Vinkel for ledd %d [°]', joint_idx));
legend('log10(cond(J_vJ_v^T))','Manipulerbarhet, m','Location','best');